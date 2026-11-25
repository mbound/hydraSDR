/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Luigi C.
 */
#include "internal.h"
#include <stdlib.h>
#include <string.h>

/* Controls without a live implementation remain idle-only. Gain and tuning
 * have serialized paths; cancellation uses the atomic acquisition state.
 */
static int require_idle(hydrasdr_dev_t *dev)
{
    if (!dev)
        return HYDRASDR_ERROR_INVALID;
    return atomic_load(&dev->run_state) != BS_IDLE ? HYDRASDR_ERROR_BUSY : 0;
}

/* Keep the ABI version independent of individual firmware image revisions. */
const char *hydrasdr_get_version(void)
{
    return "0.1.0";
}

/* Errors are library-owned values, not ambiguous errno/libusb mixtures. */
const char *hydrasdr_strerror(int error)
{
    switch (error) {
    case 0: return "success";
    case HYDRASDR_ERROR_INVALID: return "invalid argument";
    case HYDRASDR_ERROR_NO_DEVICE: return "device absent or disconnected";
    case HYDRASDR_ERROR_ACCESS: return "USB or device-lock permission denied";
    case HYDRASDR_ERROR_BUSY: return "device, driver, or stream is busy";
    case HYDRASDR_ERROR_IO: return "USB/register I/O failed";
    case HYDRASDR_ERROR_TIMEOUT: return "hardware handshake timed out";
    case HYDRASDR_ERROR_NO_MEMORY: return "allocation failed";
    case HYDRASDR_ERROR_UNSUPPORTED: return "chipset, board, or control not implemented";
    case HYDRASDR_ERROR_COLD_BOOT: return "retained vendor patch: cold power-on required";
    case HYDRASDR_ERROR_PROTOCOL: return "invalid, missing, or stale stream data";
    case HYDRASDR_ERROR_OVERRUN: return "capture continuity lost to device or host queue overrun";
    case HYDRASDR_ERROR_STATE: return "unexpected hardware or receiver state";
    default: return "unknown library error";
    }
}

/* Discovery uses descriptors only, so unsupported backends remain visible. */
int hydrasdr_get_device_count(void)
{
    return bs_usb_discover(0, NULL, NULL);
}

/* Copy descriptive information; do not hold libusb descriptor pointers. */
int hydrasdr_get_device_info(uint32_t index, struct hydrasdr_device_info *info)
{
    if (!info)
        return HYDRASDR_ERROR_INVALID;
    memset(info, 0, sizeof(*info));
    return bs_usb_discover(index, info, NULL);
}

/* Allocate one independent context, then claim a qualified interface. Booting
 * is delayed until read so setters configure the next acquisition atomically.
 */
int hydrasdr_open(hydrasdr_dev_t **out, uint32_t index)
{
    hydrasdr_dev_t *dev;
    int result;
    if (!out)
        return HYDRASDR_ERROR_INVALID;
    *out = NULL;
    dev = calloc(1, sizeof(*dev));
    if (!dev)
        return HYDRASDR_ERROR_NO_MEMORY;
    dev->lock_fd = -1;
    dev->interface_number = -1;
    atomic_init(&dev->run_state, BS_IDLE);
    atomic_init(&dev->frequency, 0);
    if (pthread_mutex_init(&dev->control_mutex, NULL)) {
        free(dev);
        return HYDRASDR_ERROR_NO_MEMORY;
    }
    dev->rx_mode = HYDRASDR_RX_SINGLE0;
    dev->gain = (struct hydrasdr_gain){1, 4, 0};
    result = bs_usb_discover(index, NULL, dev);
    if (result < 0) {
        bs_usb_close(dev);
        pthread_mutex_destroy(&dev->control_mutex);
        free(dev);
        return result;
    }
    *out = dev;
    return 0;
}

/* A read owns every outstanding transfer. Requiring it to return before close
 * prevents transfer callbacks from dereferencing a freed device/context.
 */
int hydrasdr_close(hydrasdr_dev_t *dev)
{
    int result = 0;
    BS_TRY(require_idle(dev));
    if (dev->touched && !dev->usb_lost)
        result = dev->backend->reset(dev);
    bs_usb_close(dev);
    pthread_mutex_destroy(&dev->control_mutex);
    free(dev);
    return result;
}

/* Capabilities describe implemented operations, never silicon marketing limits. */
uint32_t hydrasdr_get_capabilities(const hydrasdr_dev_t *dev)
{
    return dev ? dev->backend->capabilities : 0;
}

/* With the control mutex held, turn a failed hardware update into an orderly
 * stop. Preserve its stage independently of later status/reset procedures.
 */
static void control_failed(hydrasdr_dev_t *dev, int error, enum hydrasdr_stage stage)
{
    unsigned expected = BS_RUNNING;
    dev->control_ready = false;
    dev->control_error = error;
    dev->control_error_stage = stage;
    /* Keep an application cancellation distinct from our internal STOP. */
    atomic_compare_exchange_strong(&dev->run_state, &expected, BS_CONTROL_FAILED);
}

/* Report backend command limits rather than inferring them from a Wi-Fi channel
 * table, narrowed by any measured board bounds. The bounds describe programming
 * support, not measured sensitivity.
 */
int hydrasdr_get_tuning_range(const hydrasdr_dev_t *dev, uint64_t *minimum, uint64_t *maximum)
{
    if (!dev || !minimum || !maximum)
        return HYDRASDR_ERROR_INVALID;
    *minimum = dev->min_frequency ? dev->min_frequency : dev->backend->min_frequency;
    *maximum = dev->max_frequency ? dev->max_frequency : dev->backend->max_frequency;
    return *minimum && *maximum >= *minimum ? 0 : HYDRASDR_ERROR_UNSUPPORTED;
}

/* Report one band narrowed by board bounds. Callers iterate from index 0 until
 * INVALID; a band the board cannot reach returns UNSUPPORTED and is skipped.
 */
int hydrasdr_get_tuning_band(const hydrasdr_dev_t *dev, uint32_t index,
                              uint64_t *minimum, uint64_t *maximum)
{
    struct bs_band band;
    if (!dev || !minimum || !maximum)
        return HYDRASDR_ERROR_INVALID;
    if (dev->backend->bands) {
        if (index >= dev->backend->band_count)
            return HYDRASDR_ERROR_INVALID;
        band = dev->backend->bands[index];
    } else {
        if (index)
            return HYDRASDR_ERROR_INVALID;
        band = (struct bs_band){dev->backend->min_frequency, dev->backend->max_frequency};
    }
    *minimum = dev->min_frequency > band.min_frequency ? dev->min_frequency : band.min_frequency;
    *maximum = dev->max_frequency && dev->max_frequency < band.max_frequency ? dev->max_frequency : band.max_frequency;
    return *minimum && *maximum >= *minimum ? 0 : HYDRASDR_ERROR_UNSUPPORTED;
}

/* Stage a center while idle, or synchronously retune under the same mutex as
 * gain/start/stop/reset. The backend calibrates its synthesizer; it never rounds
 * to a Wi-Fi channel or silently translates this into host-side mixing.
 */
int hydrasdr_set_center_freq(hydrasdr_dev_t *dev, uint64_t frequency)
{
    unsigned state;
    int result = 0;
    if (!dev)
        return HYDRASDR_ERROR_INVALID;
    pthread_mutex_lock(&dev->control_mutex);
    state = atomic_load(&dev->run_state);
    if (state != BS_IDLE && (state != BS_RUNNING || !dev->control_ready)) {
        result = HYDRASDR_ERROR_BUSY;
        goto done;
    }
    result = dev->backend->validate(frequency, dev->rate, &dev->gain);
    if (result < 0)
        goto done;
    if ((dev->min_frequency && frequency < dev->min_frequency) ||
        (dev->max_frequency && frequency > dev->max_frequency)) {
        result = HYDRASDR_ERROR_UNSUPPORTED;
        goto done;
    }
    if (state == BS_RUNNING) {
        if (!(dev->backend->capabilities & HYDRASDR_CAP_LIVE_TUNING) ||
            !dev->backend->change_frequency) {
            result = HYDRASDR_ERROR_UNSUPPORTED;
            goto done;
        }
        result = dev->backend->change_frequency(dev, frequency);
        if (result < 0) {
            control_failed(dev, result, HYDRASDR_STAGE_RF);
            goto done;
        }
    }
    atomic_store(&dev->frequency, frequency);
done:
    pthread_mutex_unlock(&dev->control_mutex);
    return result;
}

/* Read committed metadata atomically, including from sample callbacks while
 * another thread retunes. It does not label the frequency of queued samples.
 */
uint64_t hydrasdr_get_center_freq(const hydrasdr_dev_t *dev)
{
    return dev ? atomic_load(&dev->frequency) : 0;
}

/* True receiver-mode control: one dense ICAP lane versus two RAW2 lanes.
 * Channel1-only is reserved for a separately qualified DFE tap13 mapping.
 */
int hydrasdr_set_rx_mode(hydrasdr_dev_t *dev, enum hydrasdr_rx_mode mode)
{
    int rc = require_idle(dev);
    if (rc) return rc;
    if (mode == HYDRASDR_RX_SINGLE1) return HYDRASDR_ERROR_UNSUPPORTED;
    if (mode != HYDRASDR_RX_SINGLE0 && mode != HYDRASDR_RX_DUAL)
        return HYDRASDR_ERROR_INVALID;
    if (mode == HYDRASDR_RX_DUAL && dev->rate != 40000000)
        return HYDRASDR_ERROR_UNSUPPORTED;
    dev->rx_mode = mode;
    return 0;
}

enum hydrasdr_rx_mode hydrasdr_get_rx_mode(const hydrasdr_dev_t *dev)
{
    return dev ? dev->rx_mode : 0;
}

uint32_t hydrasdr_get_rx_channel_count(const hydrasdr_dev_t *dev)
{
    return dev ? (dev->rx_mode == HYDRASDR_RX_DUAL ? 2u : 1u) : 0u;
}

/* The backend validates the rate and programs the matching capture tap and
 * filter profile at preparation. No host decimator is hidden here.
 */
int hydrasdr_set_sample_rate(hydrasdr_dev_t *dev, uint32_t rate)
{
    BS_TRY(require_idle(dev));
    BS_TRY(dev->backend->validate(dev->frequency, rate, &dev->gain));
    if (dev->rx_mode == HYDRASDR_RX_DUAL && rate != 40000000)
        return HYDRASDR_ERROR_UNSUPPORTED;
    dev->rate = rate;
    return 0;
}

/* The nominal complex rate is configuration metadata, not measured timing. */
uint32_t hydrasdr_get_sample_rate(const hydrasdr_dev_t *dev)
{
    return dev ? dev->rate : 0;
}

/* The caller holds control_mutex across validation, the complete hardware
 * transaction and the settings commit. Startup/stop/status/reset use the same
 * mutex, so even an RF-SPI read/modify/write cannot straddle a device reset.
 * A failed live update stops this acquisition. Its error reaches either the
 * recovery callback or, if no restart succeeds, the read caller.
 */
static int set_gain_locked(hydrasdr_dev_t *dev, bool manual,
                           const struct hydrasdr_gain *gain)
{
    unsigned state = atomic_load(&dev->run_state);
    if (state != BS_IDLE && (state != BS_RUNNING || !dev->control_ready))
        return HYDRASDR_ERROR_BUSY;
    if (!(dev->backend->capabilities & HYDRASDR_CAP_RAW_GAIN))
        return HYDRASDR_ERROR_UNSUPPORTED;
    BS_TRY(dev->backend->validate(dev->frequency, dev->rate, gain));
    if (state == BS_RUNNING) {
        int result;
        if (!(dev->backend->capabilities & HYDRASDR_CAP_LIVE_GAIN) ||
            !dev->backend->change_gain)
            return HYDRASDR_ERROR_UNSUPPORTED;
        result = dev->backend->change_gain(dev, manual, gain);
        if (result < 0) {
            control_failed(dev, result, HYDRASDR_STAGE_GAIN);
            return result;
        }
    }
    dev->manual = manual;
    dev->gain = *gain;
    dev->mt7921.gain_pair_enabled = false;
    return 0;
}

/* Stage mode while idle; during capture, synchronously apply the known force
 * fields. Released mode is not a claim about all internal AGC loops. A gain
 * acknowledgment means readback passed, not that every queued sample is newer.
 */
int hydrasdr_set_tuner_gain_mode(hydrasdr_dev_t *dev, int manual)
{
    int result;
    if (!dev || (manual != 0 && manual != 1))
        return HYDRASDR_ERROR_INVALID;
    pthread_mutex_lock(&dev->control_mutex);
    result = set_gain_locked(dev, manual != 0, &dev->gain);
    pthread_mutex_unlock(&dev->control_mutex);
    return result;
}

/* Reject invalid indices before hardware writes. A live request shares the
 * capture's control lock and retains the original boot baseline for cleanup.
 */
int hydrasdr_set_gain_codes(hydrasdr_dev_t *dev, const struct hydrasdr_gain *gain)
{
    int result;
    if (!dev || !gain)
        return HYDRASDR_ERROR_INVALID;
    pthread_mutex_lock(&dev->control_mutex);
    if (atomic_load(&dev->run_state) != BS_IDLE &&
        (atomic_load(&dev->run_state) != BS_RUNNING || !dev->control_ready))
        result = HYDRASDR_ERROR_BUSY;
    else if (!dev->manual)
        result = HYDRASDR_ERROR_STATE;
    else
        result = set_gain_locked(dev, true, gain);
    pthread_mutex_unlock(&dev->control_mutex);
    return result;
}

/* Read settings under the same lock used for live commits. This is not an
 * instantaneous AGC measurement, and performs no RF-SPI transaction.
 */
int hydrasdr_get_gain_codes(hydrasdr_dev_t *dev, struct hydrasdr_gain *gain, int *manual)
{
    if (!dev || !gain || !manual)
        return HYDRASDR_ERROR_INVALID;
    pthread_mutex_lock(&dev->control_mutex);
    if(dev->mt7921.gain_pair_enabled) {
        pthread_mutex_unlock(&dev->control_mutex);
        return HYDRASDR_ERROR_STATE;
    }
    *gain = dev->gain;
    *manual = dev->manual;
    pthread_mutex_unlock(&dev->control_mutex);
    return 0;
}

/* Missing research must be observable to callers, not a successful no-op. */
static int unsupported_control(hydrasdr_dev_t *dev)
{
    BS_TRY(require_idle(dev));
    return HYDRASDR_ERROR_UNSUPPORTED;
}

/* TODO: measure and implement raw-code to calibrated RF dB conversion. */
int hydrasdr_set_tuner_gain(hydrasdr_dev_t *dev, int tenths_db)
{
    (void)tenths_db;
    return unsupported_control(dev);
}

/* TODO: publish measured stage combinations rather than inventing gain values. */
int hydrasdr_get_tuner_gains(hydrasdr_dev_t *dev, int *gains, size_t capacity)
{
    (void)gains;
    (void)capacity;
    return unsupported_control(dev);
}

/* TODO: identify every relevant AGC loop and validate a complete freeze/release. */
int hydrasdr_set_agc_mode(hydrasdr_dev_t *dev, int enabled)
{
    (void)enabled;
    return unsupported_control(dev);
}

/* TODO: independently selectable filter widths and calibrated RF response.
 * The current backend applies the profile selected by the sample rate during
 * preparation.
 */
int hydrasdr_set_tuner_bandwidth(hydrasdr_dev_t *dev, uint32_t bandwidth)
{
    (void)bandwidth;
    return unsupported_control(dev);
}

/* TODO: establish crystal/PLL correction without changing sample interpretation. */
int hydrasdr_set_freq_correction(hydrasdr_dev_t *dev, int ppm)
{
    (void)ppm;
    return unsupported_control(dev);
}

/* TODO: decode board routing; the ICAP chain selector is a different control. */
int hydrasdr_set_antenna(hydrasdr_dev_t *dev, unsigned antenna)
{
    (void)antenna;
    return unsupported_control(dev);
}

/* TODO: identify writable coefficient fields and their ordering/units. This API
 * reserves volatile application only; it must never become an eFuse writer.
 */
int hydrasdr_set_calibration(hydrasdr_dev_t *dev, const void *data, size_t length)
{
    (void)data;
    (void)length;
    return unsupported_control(dev);
}

/* There is no retained host queue between reads. Clear diagnostics only. */
int hydrasdr_reset_buffer(hydrasdr_dev_t *dev)
{
    BS_TRY(require_idle(dev));
    memset(&dev->stats, 0, sizeof(dev->stats));
    return 0;
}

/* Recovery requires a gap consumer and a finite per-read budget. Configuration
 * is idle-only; a read owns its callback/context for its entire lifetime.
 */
int hydrasdr_set_recovery(hydrasdr_dev_t *dev, uint32_t limit,
                          hydrasdr_recovery_cb_t callback, void *context)
{
    BS_TRY(require_idle(dev));
    if (limit && !callback)
        return HYDRASDR_ERROR_INVALID;
    dev->recovery_limit = limit;
    dev->recovery_callback = limit ? callback : NULL;
    dev->recovery_context = limit ? context : NULL;
    return 0;
}

/* Cancellation and running/idle share one atomic state, so startup cannot erase
 * a concurrent cancellation and idle cancellation cannot poison the next read.
 * The read thread owns STOP/drain; other live controls share its register lock.
 */
int hydrasdr_cancel_async(hydrasdr_dev_t *dev)
{
    unsigned expected;
    if (!dev)
        return HYDRASDR_ERROR_INVALID;
    expected = atomic_load(&dev->run_state);
    while (expected == BS_RUNNING || expected == BS_CONTROL_FAILED)
        if (atomic_compare_exchange_weak(&dev->run_state, &expected, BS_CANCELLING))
            return 0;
    return expected == BS_CANCELLING ? 0 : HYDRASDR_ERROR_STATE;
}

/* Statistics are stable once the read thread has returned. */
int hydrasdr_get_stream_stats(hydrasdr_dev_t *dev, struct hydrasdr_stream_stats *stats)
{
    BS_TRY(require_idle(dev));
    if (!stats)
        return HYDRASDR_ERROR_INVALID;
    *stats = dev->stats;
    return 0;
}

/* The shared read engine owns startup, asynchronous transfers and cleanup. */
int hydrasdr_read_async(hydrasdr_dev_t *dev, hydrasdr_read_async_cb_t callback,
                        void *context, uint32_t depth, uint32_t buffer_samples)
{
    return bs_stream(dev, callback, context, depth, buffer_samples, 0);
}

struct sync_buffer { int16_t *iq; uint32_t count; uint32_t received; };

/* The finite firmware packet count bounds this copy; keep a defensive bound
 * so malformed backend output cannot overflow the caller's allocation.
 */
static void copy_sync(const int16_t *iq, uint32_t count, uint64_t first, void *context)
{
    struct sync_buffer *buffer = context;
    (void)first;
    if (count > buffer->count - buffer->received)
        count = buffer->count - buffer->received;
    memcpy(buffer->iq + 2u * (size_t)buffer->received, iq, 4u * (size_t)count);
    buffer->received += count;
}

/* Finite reads use the same queued USB engine and packet validation as async.
 * Requiring whole payloads avoids discarding unseen samples at the last packet.
 */
int hydrasdr_read_sync(hydrasdr_dev_t *dev, int16_t *iq, uint32_t count, uint32_t *n_read)
{
    struct sync_buffer buffer = {iq, count, 0};
    int result;
    if (n_read)
        *n_read = 0;
    if (!dev || !iq || !n_read || !count || count % BS_PACKET_SAMPLES)
        return HYDRASDR_ERROR_INVALID;
    result = bs_stream(dev, copy_sync, &buffer, 64, 16384, count / bs_packet_samples(dev));
    *n_read = buffer.received;
    if (!result && buffer.received != count)
        return HYDRASDR_ERROR_PROTOCOL;
    return result;
}
