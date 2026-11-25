/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The HydraSDR Authors
 */
#include "SoapyHydra.hpp"
#include <SoapySDR/Logger.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <shared_mutex>
#include <stdexcept>

/* Construction has no front end to report to, so a failed open throws. */
static void check(int result, const char *what)
{
    if (result != 0)
        throw std::runtime_error(std::string("hydrasdr: ") + what + " failed (" +
                                 std::to_string(result) + ")");
}

/* Test builds can pause a read between taking a chunk and copying it, and make
 * a worker fail to start, to exercise the rate-change boundary. Other builds
 * compile these to nothing.
 */
#ifdef SOAPY_HYDRA_TEST_HOOKS
void soapy_hydra_read_hook();
bool soapy_hydra_fail_worker_start();
#else
static inline void soapy_hydra_read_hook() {}
static inline bool soapy_hydra_fail_worker_start() { return false; }
#endif

/* Log an error as best effort. Logging can allocate, and it runs on failure
 * paths where an exception would terminate the process, so it never throws.
 */
static void logError(const char *message) noexcept
{
    try {
        SoapySDR::log(SOAPY_SDR_ERROR, message);
    } catch (...) {
    }
}

/* Setters called while an application runs log a refusal instead of throwing:
 * some front ends do not catch exceptions from these calls and abort.
 */
static bool report(int result, const char *what) noexcept
{
    if (result != 0) {
        try {
            SoapySDR::logf(SOAPY_SDR_ERROR, "hydra: %s failed: %s (%d)", what,
                           hydrasdr_strerror(result), result);
        } catch (...) {
        }
    }
    return result == 0;
}

/* Open the dongle at its enumeration index. Device arguments are not used.
 * Recovery is configured here because the library accepts it only while idle.
 * It keeps a stream running across overruns and reports each gap.
 */
SoapyHydra::SoapyHydra(uint32_t index, const SoapySDR::Kwargs &)
{
    check(hydrasdr_open(&dev_, index), "open");
    const int rc = hydrasdr_set_recovery(dev_, kMaxRestarts, &SoapyHydra::recoveryCallback, this);
    if (rc != 0) {
        hydrasdr_close(dev_);
        check(rc, "set_recovery");
    }
    rate_ = hydrasdr_get_sample_rate(dev_);
    SoapySDR::logf(SOAPY_SDR_INFO, "hydra: opened device %u", index);
}

/* Stop any stream first: the library requires the reader to be joined before
 * close, and destroying a joinable thread terminates the process.
 */
SoapyHydra::~SoapyHydra()
{
    deactivateStream(nullptr);
    if (dev_)
        hydrasdr_close(dev_);
}

/* Warn front ends that the gains are raw codes. */
SoapySDR::Kwargs SoapyHydra::getHardwareInfo() const
{
    SoapySDR::Kwargs info;
    info["note"] = "experimental; raw gain codes, not dB";
    info["rx_modes"] = "single0:native_dense_0x13,dual:native_RAW2_0x14";
    info["rx1_only"] = "not_qualified";
    info["dual_phase"] = "uncalibrated_per_acquisition";
    return info;
}

/* Named sensors use the existing receiver's USB handle and control lock.
 * Reading registers while active is refused, rather than injecting control
 * transfers into a 2x40-MSps stream with no sample-boundary metadata.
 */
/* MCU diagnostic sensors execute inside the currently loaded research N9
 * firmware using the existing USB handle. Legacy raw MMIO diagnostics remain
 * strictly idle-only. eFuse is never advertised without N9 capability proof.
 */
static bool researchSensor(const std::string &name, uint32_t &cmd, uint32_t &arg)
{
    if(name=="n9_abi"){cmd=HYDRASDR_DIAG_FW_ABI;arg=0;return true;}
    if(name=="n9_caps"){cmd=HYDRASDR_DIAG_FW_CAPS;arg=0;return true;}
    if(name=="n9_packets"){cmd=HYDRASDR_DIAG_FW_STAT;arg=0;return true;}
    if(name=="n9_gaps"){cmd=HYDRASDR_DIAG_FW_STAT;arg=1;return true;}
    if(name=="n9_ticks"){cmd=HYDRASDR_DIAG_FW_STAT;arg=2;return true;}
    return false;
}
std::vector<std::string> SoapyHydra::listSensors(void) const
{
    std::vector<std::string> keys={"n9_abi","n9_caps","n9_packets","n9_gaps","n9_ticks"};
    for(size_t i=0;i<hydrasdr_diag_count();++i)
        keys.emplace_back(hydrasdr_diag_info(i)->name);
    return keys;
}

SoapySDR::ArgInfo SoapyHydra::getSensorInfo(const std::string &key) const
{
    SoapySDR::ArgInfo info;
    info.key=key;
    info.type=SoapySDR::ArgInfo::STRING;
    uint32_t cmd=0,arg=0;
    if(researchSensor(key,cmd,arg)) {
        info.name="Research N9 firmware: "+key;
        info.description="Single-owner in-firmware read-only diagnostic (requires ABI v1)";
        return info;
    }
    for(size_t i=0;i<hydrasdr_diag_count();++i) {
        const auto *r=hydrasdr_diag_info(i);
        if(key==r->name){
            info.name=r->description;
            info.description="Idle-only raw 32-bit MMIO, not calibrated";
            return info;
        }
    }
    throw std::runtime_error("mt79: unsupported diagnostic sensor " + key);
}

std::string SoapyHydra::readSensor(const std::string &key) const
{
    std::shared_lock<std::shared_mutex> control(controlMutex_);
    uint32_t cmd=0,arg=0,val=0;
    if(researchSensor(key,cmd,arg)) {
        check(hydrasdr_custom_diag_query(dev_,cmd,arg,&val),"research N9 diagnostic");
        std::ostringstream out;
        out << "0x" << std::hex << std::setw(8) << std::setfill('0') << val;
        return out.str();
    }
    if(active_ || running_ || restarting_ || streamOpen_)
        throw std::runtime_error("mt79: stop and close stream before raw MMIO diagnostic read");
    for(size_t i=0;i<hydrasdr_diag_count();++i) {
        const auto *r=hydrasdr_diag_info(i);
        if(key==r->name) {
            check(hydrasdr_diag_read(dev_,i,&val),"raw MMIO read");
            std::ostringstream out;
            out << "0x" << std::hex << std::setw(8) << std::setfill('0') << val;
            return out.str();
        }
    }
    throw std::runtime_error("mt79: unsupported diagnostic sensor " + key);
}

/* Experimental MT7961 RAW2 exposes two correlated-in-time RX lanes; no TX. */
size_t SoapyHydra::getNumChannels(const int direction) const
{
    return direction == SOAPY_SDR_RX ? 2 : 0;
}

/* Raw hardware codes (the library has no calibrated dB gain yet). */
std::vector<std::string> SoapyHydra::listGains(const int, const size_t) const
{
    return {"LNA", "LPF", "DIG"};
}

/* Code limits for each gain element, in steps of one. The library accepts LNA
 * 0 to 5, and 5 GHz needs 4 or 5 for full sensitivity.
 */
SoapySDR::Range SoapyHydra::getGainRange(const int, const size_t, const std::string &name) const
{
    if (name == "LNA")
        return SoapySDR::Range(0, 5, 1);
    if (name == "LPF")
        return SoapySDR::Range(0, 8, 1);
    if (name == "DIG")
        return SoapySDR::Range(0, 12, 1);
    throw std::runtime_error("hydra: unknown gain element " + name);
}

/* The library starts in automatic mode and accepts codes only in manual mode,
 * so codes set during AGC are kept and applied when AGC is turned off.
 */
bool SoapyHydra::hasGainMode(const int, const size_t) const
{
    return true;
}

/* Switch modes, then send the stored codes when entering manual mode. The
 * gain lock keeps the codes consistent with concurrent gain calls.
 */
void SoapyHydra::setGainMode(const int, const size_t, const bool automatic)
{
    std::shared_lock<std::shared_mutex> control(controlMutex_);
    std::lock_guard<std::mutex> lock(gainMutex_);
    if (report(hydrasdr_set_tuner_gain_mode(dev_, automatic ? 0 : 1), "set_tuner_gain_mode") &&
        !automatic)
        report(hydrasdr_set_gain_codes(dev_, &gain_), "set_gain_codes");
}

/* Report automatic mode unless the library confirms manual mode. */
bool SoapyHydra::getGainMode(const int, const size_t) const
{
    std::shared_lock<std::shared_mutex> control(controlMutex_);
    hydrasdr_gain gain;
    int manual = 0;
    return hydrasdr_get_gain_codes(dev_, &gain, &manual) != 0 || !manual;
}

/* Round and clamp the code. In automatic mode it is stored for later. In manual
 * mode it replaces only its own element of the library's committed codes, so a
 * stale stored code for another element is never sent. It is stored only if
 * the library accepts it, so a refused change does not look applied.
 */
void SoapyHydra::setGain(const int, const size_t, const std::string &name, const double value)
{
    std::shared_lock<std::shared_mutex> control(controlMutex_);
    const auto range = getGainRange(SOAPY_SDR_RX, 0, name);
    const uint8_t code = static_cast<uint8_t>(
        std::lround(std::clamp(value, range.minimum(), range.maximum())));
    std::lock_guard<std::mutex> lock(gainMutex_);
    hydrasdr_gain committed;
    int manual = 0;
    const bool known = hydrasdr_get_gain_codes(dev_, &committed, &manual) == 0;
    hydrasdr_gain next = known && manual ? committed : gain_;
    if (name == "LNA")
        next.lna = code;
    else if (name == "LPF")
        next.lpf = code;
    else
        next.digital = code;
    if (!(known && manual) || report(hydrasdr_set_gain_codes(dev_, &next), "set_gain_codes"))
        gain_ = next;
}

/* Return the code the library committed in manual mode, or the stored code
 * that manual mode will apply while automatic mode is on.
 */
double SoapyHydra::getGain(const int, const size_t, const std::string &name) const
{
    std::shared_lock<std::shared_mutex> control(controlMutex_);
    std::lock_guard<std::mutex> lock(gainMutex_);
    hydrasdr_gain gain = gain_;
    hydrasdr_gain committed;
    int manual = 0;
    if (hydrasdr_get_gain_codes(dev_, &committed, &manual) == 0 && manual)
        gain = committed;
    if (name == "LNA")
        return gain.lna;
    if (name == "LPF")
        return gain.lpf;
    if (name == "DIG")
        return gain.digital;
    throw std::runtime_error("hydra: unknown gain element " + name);
}

/* Request a center in Hz. The library rejects it without retuning if invalid. */
void SoapyHydra::setFrequency(const int, const size_t, const std::string &, const double frequency,
                               const SoapySDR::Kwargs &)
{
    std::shared_lock<std::shared_mutex> control(controlMutex_);
    report(hydrasdr_set_center_freq(dev_, static_cast<uint64_t>(std::llround(frequency))),
           "set_center_freq");
}

/* Return the committed center, not the frequency of samples already queued. */
double SoapyHydra::getFrequency(const int, const size_t, const std::string &) const
{
    std::shared_lock<std::shared_mutex> control(controlMutex_);
    return static_cast<double>(hydrasdr_get_center_freq(dev_));
}

/* The only tunable element is the RF center. */
std::vector<std::string> SoapyHydra::listFrequencies(const int, const size_t) const
{
    return {"RF"};
}

/* Report one range per contiguous band, so front ends cannot offer the gaps
 * between them. A band this board cannot reach returns UNSUPPORTED and is
 * skipped. INVALID marks the end of the list.
 */
SoapySDR::RangeList SoapyHydra::getFrequencyRange(const int, const size_t,
                                                   const std::string &) const
{
    std::shared_lock<std::shared_mutex> control(controlMutex_);
    SoapySDR::RangeList ranges;
    for (uint32_t index = 0;; ++index) {
        uint64_t minimum = 0, maximum = 0;
        const int rc = hydrasdr_get_tuning_band(dev_, index, &minimum, &maximum);
        if (rc == HYDRASDR_ERROR_INVALID)
            break;
        if (rc == HYDRASDR_ERROR_UNSUPPORTED)
            continue;
        if (rc != 0) {
            report(rc, "get_tuning_band");
            break;
        }
        ranges.push_back(
            SoapySDR::Range(static_cast<double>(minimum), static_cast<double>(maximum)));
    }
    return ranges;
}

/* Rates the hardware produces without resampling. The library cannot list its
 * rates, so add one here when the backend gains it.
 */
static const double kNativeRates[] = {10e6, 20e6, 40e6, 80e6};

/* Accept only a native rate and pass it to the library. The library requires
 * this setter to run alone on the handle, so it holds the control lock
 * exclusively, which also keeps a stream from starting meanwhile.
 */
void SoapyHydra::setSampleRate(const int, const size_t, const double rate)
{
    std::unique_lock<std::shared_mutex> control(controlMutex_);
    /* Match the request within 1 Hz of a native rate and forward that exact
     * rate, so neither a huge value nor a fractional one reaches the library.
     */
    const double *native = std::find_if(std::begin(kNativeRates), std::end(kNativeRates),
                                        [rate](double n) { return std::fabs(rate - n) < 1.0; });
    if (native == std::end(kNativeRates)) {
        SoapySDR::log(SOAPY_SDR_ERROR, "hydra: the sample rate must be a native rate");
        return;
    }
    if (static_cast<uint32_t>(*native) == static_cast<uint32_t>(rate_))
        return;
    /* The library changes the rate only while idle, so a running stream stops,
     * changes rate and restarts. Wake a waiting read so it returns, then take
     * the reader lock, so the change falls between two reads. Samples at the
     * old rate are dropped, and the next read returns OVERFLOW. If the read
     * cannot restart, the stream stays active with no worker, so reads return
     * STREAM_ERROR after the overflow. An active stream whose read already
     * ended restarts too, so queued old-rate samples never get the new rate.
     */
    const bool restart = active_;
    std::unique_lock<std::mutex> reading(readerMutex_, std::defer_lock);
    if (restart) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            restarting_ = true;
        }
        cond_.notify_all();
        reading.lock();
        stopWorker();
    }
    if (report(hydrasdr_set_sample_rate(dev_, static_cast<uint32_t>(*native)),
               "set_sample_rate"))
        rate_ = hydrasdr_get_sample_rate(dev_);
    if (!restart)
        return;
    current_.clear();
    offset_ = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.clear();
        queued_ = 0;
        markGap();
    }
    if (startWorker() != 0)
        logError("hydra: cannot restart the stream after a rate change");
    {
        std::lock_guard<std::mutex> lock(mutex_);
        restarting_ = false;
    }
    cond_.notify_all();
}

/* Return the rate last committed to the library, cached atomically so this
 * needs no library call.
 */
double SoapyHydra::getSampleRate(const int, const size_t) const
{
    return rate_;
}

/* List the native rates from lowest to highest. */
std::vector<double> SoapyHydra::listSampleRates(const int, const size_t) const
{
    return std::vector<double>(std::begin(kNativeRates), std::end(kNativeRates));
}

/* The library delivers CS16. CF32 is a scaled copy. */
std::vector<std::string> SoapyHydra::getStreamFormats(const int, const size_t) const
{
    return {SOAPY_SDR_CF32, SOAPY_SDR_CS16};
}

/* Report the native format and its full-scale value. */
std::string SoapyHydra::getNativeStreamFormat(const int, const size_t, double &fullScale) const
{
    fullScale = 8192.0; /* signed 14-bit samples carried in int16 */
    return SOAPY_SDR_CS16;
}

/* Select the ICAP hardware mode as part of Soapy stream setup, not by env.
 * A true single lane sends 160 MB/s at 40 Msps; dual RAW2 sends 320 MB/s.
 * RX1-only tap13 is deliberately withheld pending physical chain validation.
 */
SoapySDR::Stream *SoapyHydra::setupStream(const int direction, const std::string &format,
                                           const std::vector<size_t> &channels,
                                           const SoapySDR::Kwargs &)
{
    if (direction != SOAPY_SDR_RX || (format != SOAPY_SDR_CS16 && format != SOAPY_SDR_CF32))
        throw std::runtime_error("mt79: only RX CF32 and CS16 streams are supported");
    if (streamOpen_)
        throw std::runtime_error("mt79: close the open stream before selecting a new mode");
    const std::vector<size_t> selected = channels.empty() ? std::vector<size_t>{0} : channels;
    enum hydrasdr_rx_mode mode;
    if (selected == std::vector<size_t>{0})
        mode = HYDRASDR_RX_SINGLE0;
    else if (selected == std::vector<size_t>{0, 1})
        mode = HYDRASDR_RX_DUAL;
    else if (selected == std::vector<size_t>{1})
        throw std::runtime_error("mt79: native RX1-only tap13 mapping not yet qualified");
    else
        throw std::runtime_error("mt79: select RX0 or RX0+RX1; duplicate/invalid channels");
    const int rc = hydrasdr_set_rx_mode(dev_, mode);
    if (rc)
        throw std::runtime_error(std::string("mt79: receiver mode setup: ") +
                                 hydrasdr_strerror(rc));
    cs16_ = format == SOAPY_SDR_CS16;
    streamChannels_ = selected;
    streamOpen_ = true;
    return reinterpret_cast<SoapySDR::Stream *>(this);
}

/* Stop the stream if it is running, then allow a new one. */
void SoapyHydra::closeStream(SoapySDR::Stream *)
{
    deactivateStream(nullptr);
    streamOpen_ = false;
}

/* Forward the library callback to the device instance. No exception may cross
 * into the C library, so a failure here, such as an allocation, stops the read.
 */
void SoapyHydra::asyncCallback(const int16_t *iq, uint32_t count, uint64_t, void *ctx)
{
    auto *self = static_cast<SoapyHydra *>(ctx);
    try {
        self->onSamples(iq, count);
    } catch (...) {
        self->onCallbackFailure();
    }
}

/* Forward the library's gap notification to the device instance, with the same
 * failure handling as the sample callback.
 */
void SoapyHydra::recoveryCallback(const hydrasdr_recovery_event *, void *ctx)
{
    auto *self = static_cast<SoapyHydra *>(ctx);
    try {
        self->onGap();
    } catch (...) {
        self->onCallbackFailure();
    }
}

/* Record the failure and cancel the read, which callbacks may do. The read then
 * returns normally, so the library still runs its cleanup. Callbacks that
 * arrive while it drains are dropped, so no samples follow the lost ones
 * without a gap.
 */
void SoapyHydra::onCallbackFailure()
{
    callbackFailed_ = true;
    hydrasdr_cancel_async(dev_);
}

/* Runs on the read thread: copy and return. Never block here or the device SRAM overruns. */
void SoapyHydra::onSamples(const int16_t *iq, uint32_t count)
{
    if (count == 0 || callbackFailed_)
        return; /* an empty chunk would read as a gap, and a failure ends the stream */
    if (streamChannels_.size() == 2 && (count & 1u)) {
        SoapySDR::log(SOAPY_SDR_ERROR, "mt79: RAW2 callback split at odd lane boundary");
        onCallbackFailure();
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (queued_ + count > kMaxQueuedSamples) {
        markGap(); /* the application is not keeping up, so drop these samples */
        return;
    }
    queue_.emplace_back(iq, iq + 2 * static_cast<size_t>(count));
    queued_ += count;
    cond_.notify_one();
}

/* Runs on the read thread after the library has delivered every sample from
 * before the gap, so a marker queued here sits at the gap's position.
 */
void SoapyHydra::onGap()
{
    if (callbackFailed_)
        return;
    std::lock_guard<std::mutex> lock(mutex_);
    markGap();
}

/* Queue a gap marker unless one is already last, so markers stay bounded by
 * the data chunks between them. The caller holds mutex_.
 */
void SoapyHydra::markGap()
{
    if (queue_.empty() || !queue_.back().empty())
        queue_.emplace_back();
    cond_.notify_one();
}

/* Reset the queue, then run the blocking library read on a worker. A worker
 * whose read already ended on its own is joined before replacing it. Timed
 * starts, triggers and finite bursts are not supported, so any flag is refused.
 */
int SoapyHydra::activateStream(SoapySDR::Stream *, const int flags, const long long,
                                const size_t numElems)
{
    if (flags != 0 || numElems != 0)
        return SOAPY_SDR_NOT_SUPPORTED;
    /* Exclusive, so an idle-only setter cannot overlap the start of the read. */
    std::unique_lock<std::shared_mutex> control(controlMutex_);
    if (running_)
        return 0;
    std::lock_guard<std::mutex> reading(readerMutex_);
    current_.clear();
    offset_ = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.clear();
        queued_ = 0;
    }
    active_ = true;
    if (startWorker() != 0) {
        active_ = false;
        return SOAPY_SDR_STREAM_ERROR;
    }
    return 0;
}

/* Run the blocking library read on a worker. A worker whose read already ended
 * on its own is joined first. The caller holds the control lock exclusively.
 */
int SoapyHydra::startWorker()
{
    if (worker_.joinable())
        worker_.join();
    callbackFailed_ = false;
    running_ = true;
    try {
        if (soapy_hydra_fail_worker_start())
            throw std::runtime_error("injected worker start failure");
        worker_ = std::thread([this] {
            report(hydrasdr_read_async(dev_, &SoapyHydra::asyncCallback, this, 0, 0),
                   "read_async");
            if (callbackFailed_)
                logError("hydra: stopped the stream after a callback failure");
            {
                /* Publish under the lock so a reader cannot miss the wakeup. */
                std::lock_guard<std::mutex> lock(mutex_);
                running_ = false;
            }
            cond_.notify_all();
        });
    } catch (const std::exception &) {
        running_ = false;
        logError("hydra: cannot start the stream thread");
        return SOAPY_SDR_STREAM_ERROR;
    }
    return 0;
}

/* Cancel the library read and join its worker. Right after a start the worker
 * may not have started the read yet, and cancel returns STATE, so keep retrying
 * until the read accepts it or the worker has finished. The caller holds the
 * control lock exclusively.
 */
void SoapyHydra::stopWorker()
{
    if (worker_.joinable()) {
        while (hydrasdr_cancel_async(dev_) == HYDRASDR_ERROR_STATE && running_)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        worker_.join();
    }
    running_ = false;
}

/* Stop the read and discard samples still queued. Timed and triggered stops
 * are not supported.
 */
int SoapyHydra::deactivateStream(SoapySDR::Stream *, const int flags, const long long)
{
    if (flags != 0)
        return SOAPY_SDR_NOT_SUPPORTED;
    /* Exclusive, so a stop cannot overlap a rate-change restart. */
    std::unique_lock<std::shared_mutex> control(controlMutex_);
    active_ = false;
    stopWorker();
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.clear();
    queued_ = 0;
    return 0;
}

/* Copy what is available, up to numElems, converting to the stream format.
 * Wait only when nothing is ready. A gap ends the read early and returns
 * OVERFLOW on the next call, so no buffer spans it. An inactive stream waits
 * for the timeout and returns TIMEOUT, as SoapySDR requires. An active stream
 * whose read has ended returns STREAM_ERROR once its samples are drained. A
 * read during a rate-change restart waits for it, up to its timeout.
 */
int SoapyHydra::readStream(SoapySDR::Stream *, void *const *buffs, const size_t numElems,
                            int &flags, long long &timeNs, const long timeoutUs)
{
    flags = 0;
    timeNs = 0;
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::microseconds(timeoutUs);
    if (!active_) {
        std::this_thread::sleep_until(deadline);
        return SOAPY_SDR_TIMEOUT;
    }
    for (;;) {
        /* Wait out a rate-change restart without holding the reader lock, so
         * the restart can take it. The deadline still applies.
         */
        {
            std::unique_lock<std::mutex> lock(mutex_);
            if (!cond_.wait_until(lock, deadline, [this] { return !restarting_; }))
                return SOAPY_SDR_TIMEOUT;
        }
        std::lock_guard<std::mutex> reading(readerMutex_);
        if (restarting_)
            continue;
        bool interrupted = false;
        const int result = readLocked(buffs, numElems, deadline, interrupted);
        if (!interrupted)
            return result;
    }
}

/* Copy what is available, up to numElems, with the reader lock held. A restart
 * that begins before anything is copied sets interrupted, so the caller waits
 * for it and reads again. One that begins later ends the read with what was
 * copied.
 */
int SoapyHydra::readLocked(void *const *buffs, const size_t numElems,
                            const std::chrono::steady_clock::time_point deadline,
                            bool &interrupted)
{
    const bool dual = streamChannels_.size() == 2;
    const size_t words_per_index = dual ? 4u : 2u; /* CS16 scalar int16 values */
    size_t count = 0;
    while (count < numElems) {
        if (restarting_) {
            interrupted = count == 0;
            break;
        }
        if (offset_ == current_.size() / words_per_index) {
            std::unique_lock<std::mutex> lock(mutex_);
            if (queue_.empty()) {
                if (count > 0)
                    break;
                if (running_ && !cond_.wait_until(lock, deadline, [this] {
                        return !queue_.empty() || !running_ || restarting_;
                    }))
                    return SOAPY_SDR_TIMEOUT;
                if (restarting_ && queue_.empty()) {
                    interrupted = true;
                    return 0;
                }
                if (queue_.empty())
                    return active_ ? SOAPY_SDR_STREAM_ERROR : SOAPY_SDR_TIMEOUT;
            }
            if (queue_.front().empty()) {
                if (count > 0)
                    break;
                queue_.pop_front();
                return SOAPY_SDR_OVERFLOW;
            }
            current_ = std::move(queue_.front());
            queue_.pop_front();
            queued_ -= current_.size() / 2;
            offset_ = 0;
        }
        soapy_hydra_read_hook();
        const size_t n = std::min(current_.size() / words_per_index - offset_, numElems - count);
        /* Native SINGLE0: one complex sample per DMA word.
         * Native DUAL: RX0[t], RX1[t] complex pair in each DMA record.
         */
        for (size_t ch = 0; ch < streamChannels_.size(); ++ch) {
            const size_t lane = dual ? ch : 0;
            if (cs16_) {
                auto *dst = static_cast<int16_t *>(buffs[ch]) + 2 * count;
                if (!dual) {
                    std::memcpy(dst, current_.data() + 2 * offset_, 4 * n);
                } else {
                    for (size_t k = 0; k < n; ++k) {
                        const size_t pos = 4 * (offset_ + k) + 2 * lane;
                        dst[2 * k] = current_[pos];
                        dst[2 * k + 1] = current_[pos + 1];
                    }
                }
            } else {
                constexpr float kScale = 1.0f / 8192.0f;
                auto *dst = static_cast<float *>(buffs[ch]) + 2 * count;
                for (size_t k = 0; k < n; ++k) {
                    const size_t pos = words_per_index * (offset_ + k) + 2 * lane;
                    dst[2 * k] = current_[pos] * kScale;
                    dst[2 * k + 1] = current_[pos + 1] * kScale;
                }
            }
        }
        count += n;
        offset_ += n;
    }
    return static_cast<int>(count);
}
