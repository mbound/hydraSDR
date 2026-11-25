/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The HydraSDR Authors
 */

/* Test the module's sample rates without hardware. Device.cpp is built
 * against stub library functions that record the rate the module requests.
 * The module must list exactly the native rates, pass each one through
 * unchanged, forward a request within 1 Hz as the exact native rate and refuse
 * any other value before it reaches the library. A rate change while
 * streaming must restart the read with the library idle, report one overflow
 * at the change and never deliver old-rate samples after it.
 */
#include "../SoapyHydra.hpp"
#include <SoapySDR/Formats.h>
#include <SoapySDR/Logger.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

#define CHECK(test)                                                                      \
    do {                                                                                 \
        if (!(test)) {                                                                   \
            fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #test);                   \
            exit(1);                                                                     \
        }                                                                                \
    } while (0)

static std::atomic<uint32_t> stub_rate{40000000};
static unsigned stub_rate_calls;
static std::atomic<bool> stub_reading{false}, stub_cancel{false};
static std::atomic<unsigned> stub_reads{0}, stub_busy_rate_calls{0};
static std::atomic<bool> stub_fail_read{false}; /* the next read ends at once with an error */
static std::atomic<int> stub_end_after{0};        /* the next read fails after this many chunks */
static std::atomic<bool> stub_slow_rate{false};   /* the next rate change takes 100 ms */
static std::atomic<bool> stub_rate_entered{false};

/* Test hooks compiled into Device.cpp for this test. The read hook pauses one
 * armed read between taking a chunk and copying it, for up to 300 ms, and
 * records whether a rate change completed meanwhile.
 */
static std::atomic<bool> hook_armed{false}, hook_paused{false}, setter_done{false},
    setter_done_during_pause{false}, fail_worker_start{false};
void soapy_hydra_read_hook()
{
    if (!hook_armed.exchange(false))
        return;
    hook_paused = true;
    for (int i = 0; i < 300 && !setter_done; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    setter_done_during_pause = setter_done.load();
}
bool soapy_hydra_fail_worker_start() { return fail_worker_start.exchange(false); }

/* Library stubs: only the rate setter and getter keep state. */
extern "C" {
const char *hydrasdr_strerror(int error) { return error ? "error" : "success"; }
static const hydrasdr_diag_register fake_diag[1]={{"dfe_source",0x830ad440u,"DFE state"}};
size_t hydrasdr_diag_count(void){return 1;}
const hydrasdr_diag_register *hydrasdr_diag_info(size_t i){return i==0?fake_diag:nullptr;}
int hydrasdr_diag_read(hydrasdr_dev_t *,size_t i,uint32_t *v){
    if (i || !v)return HYDRASDR_ERROR_INVALID;
    if (stub_reading)return HYDRASDR_ERROR_BUSY;
    *v=0x80000013u;return 0;
}
int hydrasdr_custom_diag_query(hydrasdr_dev_t *,uint32_t cmd,uint32_t arg,uint32_t *out){
    if(!stub_reading)return HYDRASDR_ERROR_BUSY;
    if(cmd==HYDRASDR_DIAG_FW_ABI)*out=HYDRASDR_DIAG_ABI_MAGIC;
    else if(cmd==HYDRASDR_DIAG_FW_CAPS)*out=HYDRASDR_DIAG_CAP_STATS;
    else if(cmd==HYDRASDR_DIAG_FW_STAT)*out=(arg==0?73:arg==1?2:9999);
    else return HYDRASDR_ERROR_UNSUPPORTED;
    return 0;
}
int hydrasdr_open(hydrasdr_dev_t **dev, uint32_t)
{
    *dev = reinterpret_cast<hydrasdr_dev_t *>(&stub_rate);
    return 0;
}
int hydrasdr_close(hydrasdr_dev_t *) { return 0; }
int hydrasdr_set_recovery(hydrasdr_dev_t *, uint32_t, hydrasdr_recovery_cb_t, void *)
{
    return 0;
}
int hydrasdr_set_sample_rate(hydrasdr_dev_t *, uint32_t rate)
{
    ++stub_rate_calls;
    if (stub_slow_rate.exchange(false)) {
        stub_rate_entered = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (stub_reading) {
        ++stub_busy_rate_calls; /* the real library refuses this */
        return HYDRASDR_ERROR_BUSY;
    }
    stub_rate = rate;
    return 0;
}
uint32_t hydrasdr_get_sample_rate(const hydrasdr_dev_t *) { return stub_rate; }
static hydrasdr_rx_mode stub_mode = HYDRASDR_RX_SINGLE0;
int hydrasdr_set_rx_mode(hydrasdr_dev_t *, enum hydrasdr_rx_mode m) {
  if (m == HYDRASDR_RX_SINGLE1) return HYDRASDR_ERROR_UNSUPPORTED;
  stub_mode=m; return 0;
}
enum hydrasdr_rx_mode hydrasdr_get_rx_mode(const hydrasdr_dev_t *) { return stub_mode; }

int hydrasdr_set_tuner_gain_mode(hydrasdr_dev_t *, int) { return 0; }
int hydrasdr_set_gain_codes(hydrasdr_dev_t *, const struct hydrasdr_gain *) { return 0; }
int hydrasdr_get_gain_codes(hydrasdr_dev_t *, struct hydrasdr_gain *gain, int *manual)
{
    *gain = hydrasdr_gain{};
    *manual = 0;
    return 0;
}
int hydrasdr_set_center_freq(hydrasdr_dev_t *, uint64_t) { return 0; }
uint64_t hydrasdr_get_center_freq(const hydrasdr_dev_t *) { return 2437000000u; }
int hydrasdr_get_tuning_band(const hydrasdr_dev_t *, uint32_t, uint64_t *, uint64_t *)
{
    return HYDRASDR_ERROR_INVALID;
}
/* Each sample's I component is the rate in MSps, so a reader can tell which
 * rate produced it.
 */
int hydrasdr_read_async(hydrasdr_dev_t *, hydrasdr_read_async_cb_t callback, void *context,
                         uint32_t, uint32_t)
{
    static int16_t iq[2 * 1024];
    ++stub_reads;
    if (stub_fail_read.exchange(false))
        return HYDRASDR_ERROR_IO;
    stub_cancel = false;
    stub_reading = true;
    const int16_t tag = static_cast<int16_t>(stub_rate / 1000000);
    for (unsigned i = 0; i < 1024; ++i) {
        if (stub_mode == HYDRASDR_RX_DUAL) {
            iq[2 * i] = (i % 2u) ? 33 : 11;
            iq[2 * i + 1] = (i % 2u) ? 44 : 22;
        } else {
            iq[2 * i] = tag;
            iq[2 * i + 1] = 0;
        }
    }
    const int end_after = stub_end_after.exchange(0);
    for (int chunks = 0; !stub_cancel; ++chunks) {
        if (end_after && chunks == end_after) {
            stub_reading = false;
            return HYDRASDR_ERROR_IO;
        }
        callback(iq, 1024, 0, context);
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    stub_reading = false;
    return 0;
}
int hydrasdr_cancel_async(hydrasdr_dev_t *)
{
    if (!stub_reading)
        return HYDRASDR_ERROR_STATE;
    stub_cancel = true;
    return 0;
}
}

int main()
{
    static const double native[] = {10e6, 20e6, 40e6, 80e6};
    SoapySDR_setLogLevel(SOAPY_SDR_CRITICAL);
    SoapyHydra dev(0, {});

    /* Same-owner diagnostics use the existing device session; known MMIO
     * reads succeed only when no stream is open or active. */
    const auto diag_names=dev.listSensors();
    CHECK(diag_names.size()==6 && diag_names.at(0)=="n9_abi" && diag_names.at(5)=="dfe_source");
    CHECK(dev.readSensor("dfe_source")=="0x80000013");
    CHECK(dev.getSensorInfo("dfe_source").type==SoapySDR::ArgInfo::STRING);
    bool invalid_sensor=false;
    try { (void)dev.readSensor("not_a_register"); }
    catch(const std::exception &) { invalid_sensor=true; }
    CHECK(invalid_sensor);

    const auto listed = dev.listSampleRates(SOAPY_SDR_RX, 0);
    CHECK(listed.size() == 4);
    const auto ranges = dev.getSampleRateRange(SOAPY_SDR_RX, 0);
    CHECK(ranges.size() == 4);
    for (unsigned i = 0; i < 4; ++i) {
        CHECK(listed[i] == native[i]);
        CHECK(ranges[i].minimum() == native[i] && ranges[i].maximum() == native[i]);
    }

    for (double rate : native) {
        dev.setSampleRate(SOAPY_SDR_RX, 0, rate);
        CHECK(stub_rate == static_cast<uint32_t>(rate));
        CHECK(dev.getSampleRate(SOAPY_SDR_RX, 0) == rate);
    }

    /* A request within 1 Hz of a native rate reaches the library as that exact
     * rate, never rounded to a neighboring value.
     */
    dev.setSampleRate(SOAPY_SDR_RX, 0, 10000000.75);
    CHECK(stub_rate == 10000000);
    dev.setSampleRate(SOAPY_SDR_RX, 0, 79999999.5);
    CHECK(stub_rate == 80000000);

    /* Refused values never reach the library, and the rate stays at 80 MSps. */
    const unsigned calls = stub_rate_calls;
    for (double rate : {5e6, 60e6, 160e6, 4334967296.0, -40e6, 10000001.0, std::nan("")})
        dev.setSampleRate(SOAPY_SDR_RX, 0, rate);
    CHECK(stub_rate_calls == calls);
    CHECK(dev.getSampleRate(SOAPY_SDR_RX, 0) == 80e6);

    /* Change the rate while a reader streams on another thread. */
    dev.setSampleRate(SOAPY_SDR_RX, 0, 40e6);
    auto *stream = dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16);
    bool idle_only=false;
    try { (void)dev.readSensor("dfe_source"); }
    catch(const std::exception &) { idle_only=true; }
    CHECK(idle_only); /* stream is open, hardware reads are blocked */
    CHECK(dev.activateStream(stream) == 0);
    /* Firmware diagnostics remain accessible while I/Q is streaming. */
    for(int spin=0;spin<1000 && !stub_reading;spin++)
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    CHECK(stub_reading);
    CHECK(dev.readSensor("n9_abi")=="0x4d543031");
    CHECK(dev.readSensor("n9_caps")=="0x00000001");
    CHECK(dev.readSensor("n9_packets")=="0x00000049");

    std::atomic<bool> stop{false};
    std::string events;
    std::thread reader([&] {
        static int16_t buf[2 * 4096];
        void *buffs[] = {buf};
        while (!stop) {
            int flags = 0;
            long long time = 0;
            const int n = dev.readStream(stream, buffs, 4096, flags, time, 100000);
            if (n == SOAPY_SDR_OVERFLOW)
                events += 'O';
            else if (n == SOAPY_SDR_STREAM_ERROR)
                events += 'E';
            else if (n > 0)
                for (int i = 0; i < n; ++i)
                    if (events.empty() || events.back() != (buf[2 * i] == 40 ? 'a' : 'b'))
                        events += buf[2 * i] == 40 ? 'a' : 'b';
        }
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    dev.setSampleRate(SOAPY_SDR_RX, 0, 20e6);
    CHECK(dev.getSampleRate(SOAPY_SDR_RX, 0) == 20e6);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    const unsigned reads = stub_reads;
    dev.setSampleRate(SOAPY_SDR_RX, 0, 20e6); /* same rate: no restart */
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    stop = true;
    reader.join();
    CHECK(dev.deactivateStream(stream) == 0);
    dev.closeStream(stream);
    /* 40 MSps samples, one overflow at the change, then only 20 MSps samples. */
    if (events != "aOb")
        fprintf(stderr, "events: %s\n", events.c_str());
    CHECK(events == "aOb");
    CHECK(stub_rate == 20000000 && stub_reads == 2 && reads == 2);
    CHECK(stub_busy_rate_calls == 0);

    /* Toggle the rate rapidly while a reader reads odd sizes, so it often holds
     * part of a chunk. A read that starts after a change has returned must
     * never contain samples at the old rate, and no read may mix two rates.
     * While a change is in progress the expected rate is unknown (zero), and
     * reads that start then are not checked.
     */
    std::atomic<int> committed{20};
    std::atomic<unsigned> violations{0}, overflows{0};
    stream = dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16);
    CHECK(dev.activateStream(stream) == 0);
    stop = false;
    std::thread toggler_reader([&] {
        static int16_t buf[2 * 1500];
        void *buffs[] = {buf};
        while (!stop) {
            int flags = 0;
            long long time = 0;
            const int expected = committed;
            const int n = dev.readStream(stream, buffs, 1500, flags, time, 100000);
            if (n == SOAPY_SDR_OVERFLOW)
                ++overflows;
            for (int i = 1; i < n; ++i)
                if (buf[2 * i] != buf[0])
                    ++violations;
            if (n > 0 && expected != 0 && buf[0] != expected)
                ++violations;
        }
    });
    for (int i = 0; i < 30; ++i) {
        const int next = committed == 20 ? 40 : 20;
        committed = 0;
        dev.setSampleRate(SOAPY_SDR_RX, 0, next * 1e6);
        committed = next;
        std::this_thread::sleep_for(std::chrono::milliseconds(3 + i % 7));
    }
    stop = true;
    toggler_reader.join();
    CHECK(violations == 0);
    CHECK(overflows > 0);
    CHECK(stub_busy_rate_calls == 0);

    /* Pause a read between taking a chunk and copying it, and change the rate
     * meanwhile. The change must wait for that read, so it cannot complete
     * while old-rate samples are still being copied.
     */
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    {
        static int16_t buf[2 * 1500];
        void *buffs[] = {buf};
        int flags = 0;
        long long time = 0;
        const int before = committed;
        hook_armed = true;
        std::thread paused_reader([&] {
            const int n = dev.readStream(stream, buffs, 1500, flags, time, 500000);
            CHECK(n > 0 && buf[0] == before);
        });
        for (int i = 0; i < 2000 && !hook_paused; ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        CHECK(hook_paused);
        std::thread setter([&] {
            dev.setSampleRate(SOAPY_SDR_RX, 0, before == 20 ? 40e6 : 20e6);
            setter_done = true;
        });
        paused_reader.join();
        setter.join();
        CHECK(!setter_done_during_pause);
        committed = before == 20 ? 40 : 20;
        hook_paused = false;
        setter_done = false;
    }

    /* A restart whose worker cannot start must end the stream with an error
     * after the overflow, never leave reads timing out forever.
     */
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    fail_worker_start = true;
    dev.setSampleRate(SOAPY_SDR_RX, 0, dev.getSampleRate(SOAPY_SDR_RX, 0) == 80e6 ? 10e6 : 80e6);
    {
        std::string started;
        for (int i = 0; i < 20 && (started.empty() || started.back() != 'E'); ++i) {
            static int16_t buf[2 * 1024];
            void *buffs[] = {buf};
            int flags = 0;
            long long time = 0;
            const int n = dev.readStream(stream, buffs, 1024, flags, time, 100000);
            started += n == SOAPY_SDR_OVERFLOW      ? 'O'
                       : n == SOAPY_SDR_STREAM_ERROR ? 'E'
                       : n == SOAPY_SDR_TIMEOUT      ? 'T'
                                                     : 'd';
        }
        if (started != "OE")
            fprintf(stderr, "after failed worker start: %s\n", started.c_str());
        CHECK(started == "OE");
    }
    CHECK(dev.deactivateStream(stream) == 0);
    CHECK(dev.activateStream(stream) == 0);
    committed = static_cast<int>(dev.getSampleRate(SOAPY_SDR_RX, 0) / 1e6);

    /* A stream whose read already ended, with samples still queued, must not
     * deliver those samples under the new rate: a rate change restarts it, and
     * the first read is the overflow.
     */
    CHECK(dev.deactivateStream(stream) == 0);
    stub_end_after = 3;
    CHECK(dev.activateStream(stream) == 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    {
        const double next = dev.getSampleRate(SOAPY_SDR_RX, 0) == 20e6 ? 40e6 : 20e6;
        dev.setSampleRate(SOAPY_SDR_RX, 0, next);
        static int16_t buf[2 * 1024];
        void *buffs[] = {buf};
        int flags = 0;
        long long time = 0;
        CHECK(dev.readStream(stream, buffs, 1024, flags, time, 500000) == SOAPY_SDR_OVERFLOW);
        const int n = dev.readStream(stream, buffs, 1024, flags, time, 500000);
        CHECK(n > 0 && buf[0] == static_cast<int>(next / 1e6));
    }

    /* A read that starts during a restart waits for it, up to its timeout,
     * and returns the overflow instead of an early timeout.
     */
    {
        const double next = dev.getSampleRate(SOAPY_SDR_RX, 0) == 20e6 ? 40e6 : 20e6;
        stub_rate_entered = false;
        stub_slow_rate = true;
        std::thread setter([&] { dev.setSampleRate(SOAPY_SDR_RX, 0, next); });
        for (int i = 0; i < 2000 && !stub_rate_entered; ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        CHECK(stub_rate_entered);
        static int16_t buf[2 * 1024];
        void *buffs[] = {buf};
        int flags = 0;
        long long time = 0;
        const auto start = std::chrono::steady_clock::now();
        const int n = dev.readStream(stream, buffs, 1024, flags, time, 1000000);
        const auto waited = std::chrono::steady_clock::now() - start;
        setter.join();
        if (n != SOAPY_SDR_OVERFLOW)
            fprintf(stderr, "read during restart returned %d\n", n);
        CHECK(n == SOAPY_SDR_OVERFLOW);
        CHECK(waited < std::chrono::milliseconds(900));
    }

    /* A restart whose read fails at once must end the stream with an error
     * after the overflow, never leave reads timing out forever.
     */
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    stub_fail_read = true;
    dev.setSampleRate(SOAPY_SDR_RX, 0, dev.getSampleRate(SOAPY_SDR_RX, 0) == 20e6 ? 40e6 : 20e6);
    std::string after;
    for (int i = 0; i < 20 && (after.empty() || after.back() != 'E'); ++i) {
        static int16_t buf[2 * 1024];
        void *buffs[] = {buf};
        int flags = 0;
        long long time = 0;
        const int n = dev.readStream(stream, buffs, 1024, flags, time, 100000);
        if (n == SOAPY_SDR_OVERFLOW)
            after += 'O';
        else if (n == SOAPY_SDR_STREAM_ERROR)
            after += 'E';
        else if (n == SOAPY_SDR_TIMEOUT)
            after += 'T';
    }
    if (after != "OE")
        fprintf(stderr, "after failed restart: %s\n", after.c_str());
    CHECK(after == "OE");
    CHECK(dev.deactivateStream(stream) == 0);
    dev.closeStream(stream);

    /* Native 1RX and native 2RX must have different hardware configurations
     * and unambiguous RAW2 demultiplexing; neither is a display-only mask.
     */
    dev.setSampleRate(SOAPY_SDR_RX, 0, 40e6);
    auto *dual = dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, {0,1});
    CHECK(stub_mode == HYDRASDR_RX_DUAL);
    CHECK(dev.activateStream(dual) == 0);
    {
        int16_t ch0[256] = {}, ch1[256] = {};
        void *b[] = {ch0,ch1};
        int flags = 0;
        long long when = 0;
        const int n = dev.readStream(dual,b,128,flags,when,500000);
        CHECK(n == 128);
        for(int i=0;i<n;i++) {
            CHECK(ch0[2*i] == 11 && ch0[2*i+1] == 22);
            CHECK(ch1[2*i] == 33 && ch1[2*i+1] == 44);
        }
    }
    CHECK(dev.deactivateStream(dual)==0);
    dev.closeStream(dual);
    CHECK(stub_mode == HYDRASDR_RX_DUAL);
    auto *single = dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, {0});
    CHECK(stub_mode == HYDRASDR_RX_SINGLE0);
    CHECK(dev.activateStream(single)==0);
    {
        int16_t ch[256]={};void *b[]={ch};int flags=0;long long when=0;
        const int n=dev.readStream(single,b,128,flags,when,500000);
        CHECK(n==128);
        for(int i=0;i<n;i++)CHECK(ch[2*i]==40 && ch[2*i+1]==0);
    }
    CHECK(dev.deactivateStream(single)==0);
    dev.closeStream(single);
    bool rejected=false;
    try { (void)dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, {1}); }
    catch(const std::runtime_error &) { rejected=true; }
    CHECK(rejected); /* RX1-only requires physically validated tap13. */
    rejected=false;
    try { (void)dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, {1,0}); }
    catch(const std::runtime_error &) { rejected=true; }
    CHECK(rejected); /* No silently reordered lanes. */

    puts("SoapySDR native sample rate tests passed");
    return 0;
}
