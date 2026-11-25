/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The HydraSDR Authors
 */
#ifndef HYDRASDR_SOAPY_HYDRA_HPP
#define HYDRASDR_SOAPY_HYDRA_HPP

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.h>
#include <hydrasdr.h>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <vector>

/* SoapySDR device for libhydrasdr. Settings that the library supports are
 * wired up, and only native hardware rates are offered. An RX stream runs
 * hydrasdr_read_async on a worker thread into a queue, and the reader side
 * converts to CS16 or CF32 without resampling.
 */
class SoapyHydra : public SoapySDR::Device
{
public:
    SoapyHydra(uint32_t index, const SoapySDR::Kwargs &args);
    ~SoapyHydra() override;

    /* The key applications pass as driver=hydra. */
    std::string getDriverKey() const override { return "hydra"; }
    /* The chipset family that the library backend drives. */
    std::string getHardwareKey() const override { return "MT7921"; }
    SoapySDR::Kwargs getHardwareInfo() const override;

    /* Device-owned, idle-only diagnostic sensors: no second USB claimant. */
    std::vector<std::string> listSensors(void) const override;
    SoapySDR::ArgInfo getSensorInfo(const std::string &key) const override;
    std::string readSensor(const std::string &key) const override;

    size_t getNumChannels(const int direction) const override;

    std::vector<std::string> listGains(const int direction, const size_t channel) const override;
    SoapySDR::Range getGainRange(const int direction, const size_t channel,
                                 const std::string &name) const override;
    bool hasGainMode(const int direction, const size_t channel) const override;
    void setGainMode(const int direction, const size_t channel, const bool automatic) override;
    bool getGainMode(const int direction, const size_t channel) const override;
    void setGain(const int direction, const size_t channel, const std::string &name,
                 const double value) override;
    double getGain(const int direction, const size_t channel,
                   const std::string &name) const override;

    void setFrequency(const int direction, const size_t channel, const std::string &name,
                      const double frequency, const SoapySDR::Kwargs &args) override;
    double getFrequency(const int direction, const size_t channel,
                        const std::string &name) const override;
    std::vector<std::string> listFrequencies(const int direction,
                                             const size_t channel) const override;
    SoapySDR::RangeList getFrequencyRange(const int direction, const size_t channel,
                                          const std::string &name) const override;

    void setSampleRate(const int direction, const size_t channel, const double rate) override;
    double getSampleRate(const int direction, const size_t channel) const override;
    std::vector<double> listSampleRates(const int direction, const size_t channel) const override;

    std::vector<std::string> getStreamFormats(const int direction,
                                              const size_t channel) const override;
    std::string getNativeStreamFormat(const int direction, const size_t channel,
                                      double &fullScale) const override;

    SoapySDR::Stream *setupStream(const int direction, const std::string &format,
                                  const std::vector<size_t> &channels = std::vector<size_t>(),
                                  const SoapySDR::Kwargs &args = SoapySDR::Kwargs()) override;
    void closeStream(SoapySDR::Stream *stream) override;
    int activateStream(SoapySDR::Stream *stream, const int flags = 0, const long long timeNs = 0,
                       const size_t numElems = 0) override;
    int deactivateStream(SoapySDR::Stream *stream, const int flags = 0,
                         const long long timeNs = 0) override;
    int readStream(SoapySDR::Stream *stream, void *const *buffs, const size_t numElems,
                   int &flags, long long &timeNs, const long timeoutUs = 100000) override;

private:
    static void asyncCallback(const int16_t *iq, uint32_t count, uint64_t first, void *ctx);
    static void recoveryCallback(const hydrasdr_recovery_event *event, void *ctx);
    void onSamples(const int16_t *iq, uint32_t count);
    void onGap();
    int readLocked(void *const *buffs, size_t numElems,
                   std::chrono::steady_clock::time_point deadline, bool &interrupted);
    void markGap();
    void onCallbackFailure();
    int startWorker();
    void stopWorker();

    static constexpr size_t kMaxQueuedSamples = 16u << 20; /* complex samples, ~64 MiB */
    static constexpr uint32_t kMaxRestarts = 3; /* device or transport faults per stream */

    hydrasdr_dev_t *dev_ = nullptr;
    /* Library control calls. Calls the library allows to overlap take it shared.
     * The idle-only rate setter and the start of a read take it exclusively.
     * Taken before gainMutex_.
     */
    mutable std::shared_mutex controlMutex_;
    mutable std::mutex gainMutex_; /* guards gain_ across concurrent gain calls */
    hydrasdr_gain gain_{2, 8, 0};
    bool streamOpen_ = false;
    bool cs16_ = false;
    std::vector<size_t> streamChannels_{0}; /* channel order of the active Soapy RX stream */
    std::atomic<double> rate_{0};

    std::thread worker_;
    std::atomic<bool> active_{false};  /* between activateStream and deactivateStream */
    std::atomic<bool> running_{false}; /* the worker's library read has not returned */
    std::atomic<bool> callbackFailed_{false};
    std::atomic<bool> restarting_{false}; /* a rate change is restarting the read */
    /* Held by readStream() for its whole call. A rate change takes it, so it
     * swaps rates between reads and no read mixes two rates. Taken after the
     * control lock and before mutex_.
     */
    std::mutex readerMutex_;
    std::mutex mutex_;
    std::condition_variable cond_;
    std::deque<std::vector<int16_t>> queue_; /* an empty chunk marks a gap */
    size_t queued_ = 0;

    std::vector<int16_t> current_; /* chunk being read, owned by the reader */
    size_t offset_ = 0;            /* complex samples already read from current_ */
};

#endif
