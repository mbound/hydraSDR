/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Luigi C.
 */
#ifndef HYDRASDR_H
#define HYDRASDR_H

#include <stddef.h>
#include <stdint.h>

#if defined(__GNUC__)
#pragma GCC visibility push(default)
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** Opaque receiver. Each handle owns its USB context and one Wi-Fi interface. */
typedef struct hydrasdr_dev hydrasdr_dev_t;

enum hydrasdr_error {
    HYDRASDR_OK = 0,
    HYDRASDR_ERROR_INVALID = -1,
    HYDRASDR_ERROR_NO_DEVICE = -2,
    HYDRASDR_ERROR_ACCESS = -3,
    HYDRASDR_ERROR_BUSY = -4,
    HYDRASDR_ERROR_IO = -5,
    HYDRASDR_ERROR_TIMEOUT = -6,
    HYDRASDR_ERROR_NO_MEMORY = -7,
    HYDRASDR_ERROR_UNSUPPORTED = -8,
    HYDRASDR_ERROR_COLD_BOOT = -9,
    HYDRASDR_ERROR_PROTOCOL = -10,
    HYDRASDR_ERROR_OVERRUN = -11,
    HYDRASDR_ERROR_STATE = -12
};

enum hydrasdr_chipset {
    HYDRASDR_CHIP_UNKNOWN,
    HYDRASDR_CHIP_MT7921,
    HYDRASDR_CHIP_MT7925
};

enum hydrasdr_capability {
    HYDRASDR_CAP_RX = 1u << 0,
    HYDRASDR_CAP_RAW_GAIN = 1u << 1,
    HYDRASDR_CAP_GAIN_DB = 1u << 2,
    HYDRASDR_CAP_AGC = 1u << 3,
    HYDRASDR_CAP_FILTER = 1u << 4,
    HYDRASDR_CAP_ANTENNA = 1u << 5,
    HYDRASDR_CAP_CALIBRATION = 1u << 6,
    HYDRASDR_CAP_LIVE_GAIN = 1u << 7,
    HYDRASDR_CAP_TUNING = 1u << 8,
    HYDRASDR_CAP_LIVE_TUNING = 1u << 9
};

/** Discovery includes recognized but unimplemented devices (capabilities=0).
 * Indices are enumeration snapshots: unplugging a device can change them.
 * No firmware is loaded and no interface is claimed during enumeration.
 */
struct hydrasdr_device_info {
    uint16_t vid, pid;
    uint8_t bus, address;
    enum hydrasdr_chipset chipset;
    uint32_t capabilities;
    char name[96];
};

/** Raw hardware indices, deliberately distinct from calibrated gain in dB. */
struct hydrasdr_gain {
    uint8_t lna;       /**< MT7972 range 0..5; 5 GHz needs 4..5 for full sensitivity. */
    uint8_t lpf;       /**< Filter-stage gain 0..8; does not select bandwidth. */
    uint8_t digital;   /**< Digital gain code 0..12. */
};

/** Last hardware operation reached; useful when a read fails before samples arrive. */
enum hydrasdr_stage {
    HYDRASDR_STAGE_IDLE,
    HYDRASDR_STAGE_RESET,
    HYDRASDR_STAGE_ROM,
    HYDRASDR_STAGE_DOWNLOAD,
    HYDRASDR_STAGE_CAPTURE,
    HYDRASDR_STAGE_RF,
    HYDRASDR_STAGE_GAIN,
    HYDRASDR_STAGE_STREAM,
    HYDRASDR_STAGE_VERIFY
};

/** Summary of the most recent read, available after read_async/read_sync returns.
 * A nonzero read result invalidates continuity of that capture, even if some
 * samples were already delivered. firmware_state 9 means the ICAP ring overran.
 * An automatically recovered read can return zero despite notified gaps.
 * Samples/packets/bytes accumulate across attempts; the remaining fields describe
 * the final attempt. Each recovery event snapshots the current acquisition.
 * Packets count received data events, including the final packet discarded at
 * a device gap, so packets times payload size can exceed delivered samples.
 */
struct hydrasdr_stream_stats {
    uint64_t samples;
    uint64_t packets;
    uint64_t usb_bytes;
    uint32_t firmware_state;
    uint32_t dma_errors;
    enum hydrasdr_stage stage;
    int reset_error;
};

/** Samples are interleaved native-endian int16_t I,Q in [-8192,8191].
 * count and first_sample count COMPLEX samples, not scalar components/bytes.
 * The buffer is borrowed until callback return; copy it to retain it. There
 * is no host-side averaging, decimation, left shifting, or floating-point
 * conversion. At 10 and 20 MSps the device's capture packer keeps one tap
 * sample in four before the samples reach the host.
 * Run callbacks promptly: a stalled consumer can overrun the device's SRAM.
 * The center-frequency setter/getter, tuning-range getter, raw-gain mode/code
 * setters, gain-code getter and cancel_async may be called from the callback;
 * other operations must wait until the read returns.
 * Prefer a control thread for updates so callbacks keep draining data.
 */
typedef void (*hydrasdr_read_async_cb_t)(const int16_t *iq, uint32_t count,
                                        uint64_t first_sample, void *context);

/** Notification at a source gap or before a full acquisition restart.
 * A local source gap keeps firmware, USB and receiver settings running. A full
 * restart drains the preceding acquisition first. Fresh samples follow this
 * event and are NOT continuous with preceding samples. The number of missing
 * samples is unknown; no zeros or invented timestamps are inserted.
 */
struct hydrasdr_recovery_event {
    uint64_t next_sample; /**< Total samples delivered in this read; not elapsed RF time. */
    uint32_t attempt;     /**< Recovery number starting at one, saturating at UINT32_MAX. */
    int error;           /**< Error that caused this discontinuity. */
    struct hydrasdr_stream_stats previous; /**< Acquisition snapshot at the gap, with its fault state. */
};

/** Called on the read thread, with no library lock held and no samples in flight
 * to application callbacks. May call cancel_async or getters allowed during a
 * read. Live setters are also allowed, but return BUSY during full restart or
 * cancellation. A local gap retains the live control window. A notification
 * is not a guarantee of further samples; read_async reports final failure.
 */
typedef void (*hydrasdr_recovery_cb_t)(const struct hydrasdr_recovery_event *event,
                                      void *context);

/* Explicit on-chip capture configurations. Not just output-channel discard.
 * SINGLE0 uses the dense hardware ICAP source 0x13, tap4.
 * DUAL uses verified RAW2 source 0x14, mux6. Independent and calibrated
 * coherent dual mode share a capture layout; phase calibration is separate.
 * SINGLE1 is reserved until tap13/physical antenna mapping is qualified.
 */
enum hydrasdr_rx_mode {
    HYDRASDR_RX_SINGLE0 = 1,
    HYDRASDR_RX_SINGLE1 = 2,
    HYDRASDR_RX_DUAL = 3
};
/* Mode changes require idle device, and take effect next capture.
 * Reject unsupported mode/rate combinations, never silently downgrade.
 */
int hydrasdr_set_rx_mode(hydrasdr_dev_t *dev, enum hydrasdr_rx_mode mode);
enum hydrasdr_rx_mode hydrasdr_get_rx_mode(const hydrasdr_dev_t *dev);
uint32_t hydrasdr_get_rx_channel_count(const hydrasdr_dev_t *dev);

/* Experimental device-owned diagnostics on the SAME USB handle as SDR.
 * Read-only, named register snapshots; currently available only while idle.
 * This API never resets the device, changes firmware, or accesses OTP/eFuse.
 */
struct hydrasdr_diag_register {
    const char *name;
    uint32_t addr;
    const char *description;
};
size_t hydrasdr_diag_count(void);
const struct hydrasdr_diag_register *hydrasdr_diag_info(size_t id);
int hydrasdr_diag_read(hydrasdr_dev_t *dev, size_t id, uint32_t *value);

/* In-firmware MT79xx debug mailbox. Operates through the same USB owner
 * while native 1RX or 2RX reception runs, without firmware replacement.
 * ABI availability must be established by the running firmware handshake;
 * eFuse capability is advertised only when ROM signature and helper callbacks match.
 */
enum hydrasdr_custom_diag_cmd {
    HYDRASDR_DIAG_FW_ABI = 1,
    HYDRASDR_DIAG_FW_STAT = 2,
    HYDRASDR_DIAG_FW_EFUSE = 3,
    HYDRASDR_DIAG_FW_CAPS = 4
};
#define HYDRASDR_DIAG_ABI_MAGIC 0x4d543031u
#define HYDRASDR_DIAG_CAP_STATS 1u
#define HYDRASDR_DIAG_CAP_EFUSE 2u
#define HYDRASDR_DIAG_CAP_TX_WATCHDOG 4u
/* Host-staged cyclic IQ setpoint playback (not a native DAC FIFO). */
#define HYDRASDR_DIAG_CAP_STAGED_IQ 8u
#define HYDRASDR_DIAG_CAP_TX_ACTIVE_SNAPSHOT 16u
#define HYDRASDR_DIAG_CAP_USB_INGRESS_AUDIT 32u
/* No-RF independently timer-paced consumption from the N9 software IQ FIFO. */
#define HYDRASDR_DIAG_CAP_USB_INGRESS_PACED 64u
/* No-RF write-through of host IQ into PHY IQ setpoint, with GO/PA disabled. */
#define HYDRASDR_DIAG_CAP_USB_INGRESS_PHY_DRY 128u
/* Watchdog expiry and disarm cover both PHYDFE GO/path banks. Not 2TX proof. */
#define HYDRASDR_DIAG_CAP_TX_WATCHDOG_DUAL_BANK 256u
/* Paired ch2/ch3 launch to the two qualified PHY IQ latches. Command arg0 is
 * RF-off; arg1 additionally requires the bounded dual-bank watchdog/GO gates. */
#define HYDRASDR_DIAG_CAP_DUAL_PHY_DMA 512u
/* Watchdog shutdown writes and disarm active-bit checks include logical TX
 * path addresses 0..3. Paths2/3 are clock/power gated while DBDC is off, so
 * active-DBDC hardware qualification remains a separate gate. */
#define HYDRASDR_DIAG_CAP_TX_WATCHDOG_PATH4_GUARD 1024u
int hydrasdr_custom_diag_query(hydrasdr_dev_t *dev, uint32_t command,
                                uint32_t argument, uint32_t *result);
/* Resident N9 ROM-backed read-only eFuse access. bank=0..2; offset is a
 * 16-byte-aligned byte address within that bank (0x000..0x3f0).
 * Requires the current RAM firmware to advertise CAP_EFUSE. This API never
 * loads stock firmware, resets USB, or programs OTP. One owner; concurrent
 * with 1RX or 2RX streaming. Device-specific bytes are caller-private.
 */
struct hydrasdr_efuse_block {
    uint16_t bank;
    uint16_t offset;
    uint32_t raw_validity;
    uint8_t data[16];
};
int hydrasdr_custom_efuse_read(hydrasdr_dev_t *dev, unsigned bank,
                                 unsigned offset, struct hydrasdr_efuse_block *out);

/* Experimental native, read-only, pre-GO MT7961/MT7972 RF-state snapshot.
 * Opt in with HYDRA_RF_CAL_SNAPSHOT=1 BEFORE starting a normal RX stream.
 * The host captures all fields AFTER ordinary RF/gain/filter setup, BEFORE
 * firmware GO. The getter copies an immutable cached snapshot, and neither
 * the getter nor these APIs access the USB bus or trigger recalibration.
 *
 * Source-qualified values are RAW registers. Port 0/1 denote PHY banks or
 * physical analog SPI ports in their respective arrays; never infer that
 * a PHY bank equals a physical antenna. No dB/dBm, measured TSSI, PA power,
 * DPD completion or quality-assurance PASS information is supplied.
 */
#define HYDRASDR_RF_CAL_MMIO_COUNT 22u
#define HYDRASDR_RF_CAL_ADIE_COUNT 9u
#define HYDRASDR_RF_CAL_SNAPSHOT_VERSION 1u
#define HYDRASDR_RF_CAL_PHASE_PRE_GO 1u
struct hydrasdr_rf_diag_reg {
    const char *name;
    uint32_t address;
};
struct hydrasdr_rf_cal_snapshot {
    uint32_t schema_version;
    uint32_t phase;
    uint64_t frequency_hz;
    uint32_t sample_rate;
    enum hydrasdr_rx_mode rx_mode;
    uint32_t mmio[HYDRASDR_RF_CAL_MMIO_COUNT];
    uint32_t adie[2][HYDRASDR_RF_CAL_ADIE_COUNT];
};
enum hydrasdr_rf_cal_flag {
    HYDRASDR_RF_CAL_MANUAL0 = 1u << 0,
    HYDRASDR_RF_CAL_MANUAL1 = 1u << 1,
    HYDRASDR_RF_CAL_TXPWR_OVERRIDE0 = 1u << 2,
    HYDRASDR_RF_CAL_TXPWR_OVERRIDE1 = 1u << 3,
    HYDRASDR_RF_CAL_ADIE_GAIN_OVERRIDE_RF0 = 1u << 4,
    HYDRASDR_RF_CAL_ADIE_GAIN_OVERRIDE_RF1 = 1u << 5,
};
const struct hydrasdr_rf_diag_reg *hydrasdr_rf_cal_mmio_info(unsigned index);
const struct hydrasdr_rf_diag_reg *hydrasdr_rf_cal_adie_info(unsigned index);
int hydrasdr_get_rf_cal_snapshot(hydrasdr_dev_t *dev,
                                  struct hydrasdr_rf_cal_snapshot *out);
uint32_t hydrasdr_rf_cal_flags(const struct hydrasdr_rf_cal_snapshot *snapshot);

/* Native physical eFuse per-bank 16B reads are already available above via
 * hydrasdr_custom_efuse_read(). That API is a ROM read, not a stock-WM
 * logical-EEPROM query and NEVER programs OTP.
 *
 * Calibrated PA power and dBm/TSSI/DPD procedures remain UNIMPLEMENTED.
 * Vendor HQA command IDs cannot be sent to our custom streaming firmware:
 * it does not implement the stock-WM RF_AT test handler. Do not synthesize
 * command-accepted/QA-PASS responses.
 */

/* Experimental, explicitly source-qualified relative RX gain.
 * NOT absolute RF gain, input power dBm, noise figure or calibrated RSSI.
 * Never enable HYDRASDR_CAP_GAIN_DB based on a profile inferred from ambient
 * IQ. Never silently reinterpret the existing set_tuner_gain() calibrated ABI.
 * Valid for exactly the recorded frequency, sample rate, receive mode/chain.
 */
#define HYDRASDR_REL_GAIN_PROFILE_VERSION 1u
enum hydrasdr_relative_gain_basis {
    HYDRASDR_REL_GAIN_AMBIENT_PROVISIONAL = 1
};
struct hydrasdr_relative_gain_profile {
    uint32_t version;
    enum hydrasdr_relative_gain_basis source_basis;
    const char *provenance; /* Owned by caller; non-empty immutable identifier. */
    uint64_t frequency_hz;
    uint32_t sample_rate;
    enum hydrasdr_rx_mode rx_mode;
    uint8_t rx_chain;      /* Currently only physical/logical SINGLE0 mapped. */
    uint16_t lna_mask;     /* Bit set means this raw index was measured. */
    uint16_t lpf_mask;
    uint16_t digital_mask; /* Odd digital codes are unmeasured on this Fenvi. */
    int16_t lna_tenths[6];
    int16_t lpf_tenths[9];
    int16_t digital_tenths[13];
};
int hydrasdr_relative_gain_estimate(const struct hydrasdr_relative_gain_profile *p,
                                     const struct hydrasdr_gain *codes,
                                     int *relative_tenths_db);
int hydrasdr_relative_gain_select(const struct hydrasdr_relative_gain_profile *p,
                                   int requested_tenths_db,
                                   const struct hydrasdr_gain *max_codes,
                                   struct hydrasdr_gain *selected,
                                   int *estimated_tenths_db);
int hydrasdr_relative_gain_context_match(const struct hydrasdr_relative_gain_profile *p,
                                           uint64_t frequency_hz,
                                           uint32_t sample_rate,
                                           enum hydrasdr_rx_mode mode,
                                           unsigned rx_chain);

/* These routines perform NO USB I/O and NO register writes. To apply a chosen
 * raw code, the application must explicitly opt into manual mode and invoke
 * hydrasdr_set_gain_codes(). The driver retains its safety/readback/rollback
 * preflight. This profile MUST NOT be treated as transferable to other boards.
 */

/** Return a static library version string; never NULL. */
const char *hydrasdr_get_version(void);

/** Return a static explanation for a library error; never NULL. */
const char *hydrasdr_strerror(int error);

/** Return the number of recognized USB devices, or a negative error. */
int hydrasdr_get_device_count(void);

/** Describe a recognized device without claiming it. Returns 0 or an error. */
int hydrasdr_get_device_info(uint32_t index, struct hydrasdr_device_info *info);

/** Claim a supported Wi-Fi USB interface without loading firmware yet.
 * The interface must be free of a kernel Wi-Fi driver. The library never
 * unloads drivers, power-cycles a hub, downloads vendor firmware, or writes
 * nonvolatile memory. Startup may reset USB to recover a previous failure;
 * a reset that invalidates the handle returns NO_DEVICE. A retained vendor
 * patch is rejected when reading.
 * On failure *dev is NULL. MT7921AUN/MT7961-family boards with MT7972 RF
 * companion and USB identities 3574:6211 or 0e8d:7961 use the implemented
 * backend. Board-level RF
 * qualification still matters: the Fenvi AX1800 0e8d:7961 is the primary
 * current development/advanced-TX target, while other boards may differ in
 * routing, filtering and calibration.
 */
int hydrasdr_open(hydrasdr_dev_t **dev, uint32_t index);

/** Release the handle. Returns BUSY during a read; cancel and join it first.
 * The Wi-Fi kernel driver is not rebound (that could load vendor firmware).
 */
int hydrasdr_close(hydrasdr_dev_t *dev);

/** Return this backend's implemented capability bits, or 0 for NULL. */
uint32_t hydrasdr_get_capabilities(const hydrasdr_dev_t *dev);

/** Return inclusive command limits in Hz. Returns UNSUPPORTED if unknown.
 * These are programming limits, not guaranteed RF sensitivity or passband
 * coverage. The interval may contain gaps; use get_tuning_band for each
 * contiguous band. Safe during a read.
 */
int hydrasdr_get_tuning_range(const hydrasdr_dev_t *dev, uint64_t *minimum_hz,
                              uint64_t *maximum_hz);

/** Return contiguous band `index` in Hz, starting at 0. Returns INVALID past the
 * last band and UNSUPPORTED for a band this board cannot reach. Live retunes
 * may cross bands. Safe during a read.
 */
int hydrasdr_get_tuning_band(const hydrasdr_dev_t *dev, uint32_t index,
                              uint64_t *minimum_hz, uint64_t *maximum_hz);

/** Set a requested center in Hz, without channel-number rounding or host mixing.
 * The MT7921 bands are 2156000000..2610000000 and 4500000000..6200000000 Hz,
 * inclusive, with nominal 28.6102294921875 and 47.6837158203125 Hz hardware
 * steps; frequency accuracy is uncalibrated. Requests in the gap are UNSUPPORTED.
 * While idle, stage the center for the next read. With CAP_LIVE_TUNING, apply it
 * during capture without restarting ICAP/DMA or downloading firmware. Calls
 * serialize with gain and lifecycle controls; startup/shutdown may return BUSY.
 * During capture, success means calibration completed, known diagnostic rails
 * were absent and frequency readback passed. It does not prove PLL lock or settling.
 * Samples during a retune are disturbed; phase is not continuous. Queued data
 * can still belong to the old center after return. No exact sample boundary is
 * reported. A failed hardware retune attempts rollback and stops the acquisition;
 * enabled recovery can then restart with the preceding committed frequency.
 * An out-of-range request returns UNSUPPORTED without disrupting a live stream.
 */
int hydrasdr_set_center_freq(hydrasdr_dev_t *dev, uint64_t frequency_hz);

/** Return the last committed requested center in Hz, including during capture;
 * not a calibrated RF measurement or the center of buffered samples. Zero if NULL.
 */
uint64_t hydrasdr_get_center_freq(const hydrasdr_dev_t *dev);

/** Stage complex samples/second. MT7921 accepts 10000000, 20000000, 40000000
 * and 80000000. The sampled span equals the rate, using nominal 20, 40 or 80 MHz
 * analog and digital settings. 10 MSps uses the 20 MHz analog setting with a
 * narrower, about 10 MHz digital filter, which suppresses, but does not remove,
 * energy from just outside the span. USB load is about 80, 160, 160 and
 * 320 MB/s: the device pads samples at 10 and 20 MSps. Preparation reads back
 * the filter fields it owns and the rate-dependent capture bits, but cannot
 * confirm that the 10, 20 and 80 MSps digital class latch took effect.
 * This is not a calibrated passband or an alias-rejection guarantee.
 */
int hydrasdr_set_sample_rate(hydrasdr_dev_t *dev, uint32_t samples_per_second);

/** Return the nominal configured complex sample rate, or 0 for NULL. */
uint32_t hydrasdr_get_sample_rate(const hydrasdr_dev_t *dev);

/** Set gain mode: 1 forces the documented gain indices, 0 releases overrides.
 * Releasing overrides is not a claim of complete closed-loop AGC control.
 * While idle, settings apply at the next read. With CAP_LIVE_GAIN, a running
 * capture changes registers synchronously without rebooting or stopping ICAP.
 * Calls serialize with startup/cleanup; BUSY means no live control window is
 * available. Backends without live support return UNSUPPORTED during capture.
 * The update can take milliseconds and spans multiple register writes. Success
 * confirms readback, not an exact sample boundary: buffered samples may precede
 * the change and samples during the transition are not gain-calibrated.
 * A hardware update failure stops that acquisition and leaves cleanup to the
 * reader, which may automatically restart if set_recovery was enabled.
 */
int hydrasdr_set_tuner_gain_mode(hydrasdr_dev_t *dev, int manual);

/** Set manual LNA/LPF/digital indices. Requires manual mode. The same idle/live
 * semantics as set_tuner_gain_mode apply. The backend preserves unrelated fields,
 * verifies readbacks and restores the preceding settings if an update fails.
 */
int hydrasdr_set_gain_codes(hydrasdr_dev_t *dev, const struct hydrasdr_gain *gain);

/** Read committed gain codes and mode, including during capture. These are
 * requested values while idle, not an instantaneous AGC or RF gain readback.
 */
int hydrasdr_get_gain_codes(hydrasdr_dev_t *dev, struct hydrasdr_gain *gain,
                            int *manual);

/** Reserved: calibrated tuner gain in tenths of a dB. UNSUPPORTED until the
 * measured stage-gain mapping is known; raw indices must not be labeled dB.
 */
int hydrasdr_set_tuner_gain(hydrasdr_dev_t *dev, int tenths_db);

/** Reserved: enumerate calibrated gains. UNSUPPORTED; does not fill gains. */
int hydrasdr_get_tuner_gains(hydrasdr_dev_t *dev, int *gains, size_t capacity);

/** Reserved: global AGC enable/freeze. UNSUPPORTED because only individual
 * force fields are established; use tuner_gain_mode for those overrides.
 */
int hydrasdr_set_agc_mode(hydrasdr_dev_t *dev, int enabled);

/** Reserved: independently selectable RF filter bandwidth in Hz. The backend
 * applies the nominal profile that matches the sample rate. This setter remains
 * UNSUPPORTED until additional widths and their RF response are qualified.
 */
int hydrasdr_set_tuner_bandwidth(hydrasdr_dev_t *dev, uint32_t bandwidth_hz);

/** Reserved: oscillator correction in ppm. UNSUPPORTED; no software DDC added. */
int hydrasdr_set_freq_correction(hydrasdr_dev_t *dev, int ppm);

/** Reserved: physical antenna routing. UNSUPPORTED; chain != antenna switch. */
int hydrasdr_set_antenna(hydrasdr_dev_t *dev, unsigned antenna);

/** Reserved: apply volatile calibration coefficients. UNSUPPORTED until their
 * format, routing and calibration procedure are decoded. Never programs eFuse.
 */
int hydrasdr_set_calibration(hydrasdr_dev_t *dev, const void *data, size_t length);

/** Clear host statistics while idle. Each read starts a fresh hardware capture;
 * this operation does not pretend to flush an active ICAP stream.
 */
int hydrasdr_reset_buffer(hydrasdr_dev_t *dev);

/** Enable automatic recovery for read_async; configure only while idle.
 * max_restarts is the device/transport fault budget per read call. A nonzero
 * budget requires a callback so discontinuities cannot be hidden. Zero disables
 * recovery and removes the callback (default).
 * Overruns, transient transport errors and failed live configuration updates
 * restart with the last committed settings. Explicit cancellation always wins.
 * With recovery enabled, clean device/host queue overruns report gaps and keep
 * restarting without spending this budget. An overrun with a failed reset,
 * DMA error or live-control failure still spends a bounded retry. The attempt
 * counter includes all recovery notifications, including clean overruns.
 * The MT7921 firmware recovers source overruns without reloading, resetting USB
 * or reapplying RF settings. The host retains one raw packet until the next
 * data event or clean terminal state confirms it. A gap discards that packet
 * and flushes the validated prefix before notifying, so buffers never span gaps.
 * A full host queue pauses resubmission and preserves completed USB bytes in
 * existing transfer buffers. The resulting device backpressure may cause a
 * notified source gap, but does not discard host framing or grow the queue.
 * Device removal, permissions, retained vendor patches and invalid/unsupported
 * requests are not retried. Fault-budget exhaustion returns the last capture error.
 * Finite read_sync calls remain strict and never splice separate acquisitions.
 */
int hydrasdr_set_recovery(hydrasdr_dev_t *dev, uint32_t max_restarts,
                          hydrasdr_recovery_cb_t callback, void *context);

/** Capture an exact finite number of complex samples into I,Q int16_t pairs.
 * count must be a positive multiple of 2048, with room for 2*count int16_t's.
 * *n_read is initialized to zero and reports any prefix even on error. Samples
 * are continuous only within one successful call; consecutive calls restart
 * the acquisition program and must not be concatenated as a continuous stream.
 */
int hydrasdr_read_sync(hydrasdr_dev_t *dev, int16_t *iq, uint32_t count,
                       uint32_t *n_read);

/** Stream until cancel_async, device loss, or an unrecovered hardware/protocol error.
 * Like librtlsdr, this blocks the calling thread while asynchronous USB reads
 * run. transfer_count=0 selects 64 queued transfers, or 256 on macOS.
 * Allowed values are 4..256.
 * buffer_samples=0 selects 16384; otherwise use a multiple of 2048, <=262144.
 * A final callback may be shorter. Cancellation drains submitted device
 * packets before returning; callbacks can still arrive after cancellation.
 * Returns 0 for a clean cancel, otherwise a negative error. Recovery is opt-in
 * through set_recovery and always reports a gap before another acquisition.
 * With recovery enabled, cancellation at a clean overrun also returns 0.
 * first_sample remains a cumulative delivered count across those gaps, not an
 * RF timestamp. Final statistics accumulate samples/packets/bytes over attempts;
 * firmware state, stage, DMA and reset errors describe the final attempt.
 * Without recovery enabled, no samples are silently skipped; check the return
 * value before treating a capture as valid.
 */
int hydrasdr_read_async(hydrasdr_dev_t *dev, hydrasdr_read_async_cb_t callback,
                        void *context, uint32_t transfer_count,
                        uint32_t buffer_samples);

/** Request cancellation concurrently with read_async/read_sync or its callback.
 * Live gain/tuning operations are also allowed as documented above. Not async-signal-safe:
 * signal handlers should set their own flag, then cancel from a normal thread.
 * Repeated cancellation is harmless; returns STATE if no read is active.
 */
int hydrasdr_cancel_async(hydrasdr_dev_t *dev);

/** Copy final statistics while idle; returns BUSY until the read has returned. */
int hydrasdr_get_stream_stats(hydrasdr_dev_t *dev,
                             struct hydrasdr_stream_stats *stats);

/* All functions return 0 on success unless their comments say otherwise.
 * The center-frequency setter/getter, tuning-range getter, raw-gain mode/code
 * setters, gain-code getter and cancel_async may run concurrently with a read
 * and with each other. Callers must serialize all other operations on a handle,
 * including close after joining BOTH the reader and any control threads.
 * Different handles may be used independently from different threads.
 */
#ifdef __cplusplus
}
#endif
#if defined(__GNUC__)
#pragma GCC visibility pop
#endif
#endif
