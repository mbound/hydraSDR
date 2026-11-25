/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The HydraSDR Authors
 */

/* Test that Ctrl-C stops hydra_udp while a read runs without delivering any
 * sample, and that Ctrl-C during setup closes the device without a read. The
 * example is included directly, with its main renamed, and a stub library
 * replaces libhydrasdr: its read never calls back and ends only when
 * cancelled, or after a five-second limit that marks the test as failed.
 */
#define main hydra_udp_program_main
#include "../examples/hydra_udp.c"
#undef main

enum { STUB_IDLE, STUB_RUNNING, STUB_CANCELLING };
static atomic_uint stub_state = STUB_IDLE;
static atomic_int stub_reads;
static atomic_int stub_closes;
static atomic_bool stub_slow_open; /* when set, open pauses half a second */
static atomic_bool stub_opening;   /* set while that pause runs */

/* Library stubs: everything succeeds except the read, described above. */
const char *hydrasdr_strerror(int error) { return error ? "error" : "success"; }
int hydrasdr_open(hydrasdr_dev_t **dev, uint32_t index)
{
    const struct timespec tick = {.tv_nsec = 10000000};
    (void)index;
    if (atomic_load(&stub_slow_open)) {
        atomic_store(&stub_opening, true);
        for (int i = 0; i < 50; ++i)
            nanosleep(&tick, NULL);
        atomic_store(&stub_opening, false);
    }
    *dev = (hydrasdr_dev_t *)&stub_state;
    return 0;
}
int hydrasdr_close(hydrasdr_dev_t *dev)
{
    (void)dev;
    atomic_fetch_add(&stub_closes, 1);
    return 0;
}
int hydrasdr_set_sample_rate(hydrasdr_dev_t *dev, uint32_t rate)
{
    (void)dev;
    (void)rate;
    return 0;
}
uint32_t hydrasdr_get_sample_rate(const hydrasdr_dev_t *dev)
{
    (void)dev;
    return 40000000;
}
int hydrasdr_set_center_freq(hydrasdr_dev_t *dev, uint64_t frequency)
{
    (void)dev;
    (void)frequency;
    return 0;
}
int hydrasdr_set_tuner_gain_mode(hydrasdr_dev_t *dev, int manual)
{
    (void)dev;
    (void)manual;
    return 0;
}
int hydrasdr_set_gain_codes(hydrasdr_dev_t *dev, const struct hydrasdr_gain *gain)
{
    (void)dev;
    (void)gain;
    return 0;
}
int hydrasdr_set_recovery(hydrasdr_dev_t *dev, uint32_t limit, hydrasdr_recovery_cb_t callback,
                           void *context)
{
    (void)dev;
    (void)limit;
    (void)callback;
    (void)context;
    return 0;
}
int hydrasdr_get_stream_stats(hydrasdr_dev_t *dev, struct hydrasdr_stream_stats *stats)
{
    (void)dev;
    memset(stats, 0, sizeof(*stats));
    return 0;
}
int hydrasdr_read_async(hydrasdr_dev_t *dev, hydrasdr_read_async_cb_t callback, void *context,
                         uint32_t transfer_count, uint32_t buffer_samples)
{
    const struct timespec tick = {.tv_nsec = 10000000};
    (void)dev;
    (void)callback;
    (void)context;
    (void)transfer_count;
    (void)buffer_samples;
    atomic_fetch_add(&stub_reads, 1);
    atomic_store(&stub_state, STUB_RUNNING);
    for (int i = 0; i < 500 && atomic_load(&stub_state) == STUB_RUNNING; ++i)
        nanosleep(&tick, NULL);
    return atomic_exchange(&stub_state, STUB_IDLE) == STUB_CANCELLING ? 0
                                                                       : HYDRASDR_ERROR_TIMEOUT;
}
int hydrasdr_cancel_async(hydrasdr_dev_t *dev)
{
    unsigned expected = STUB_RUNNING;
    (void)dev;
    if (atomic_compare_exchange_strong(&stub_state, &expected, STUB_CANCELLING) ||
        expected == STUB_CANCELLING)
        return 0;
    return HYDRASDR_ERROR_STATE;
}

/* Send SIGINT once the stub read is running. Give up after five seconds, so a
 * setup failure ends the test promptly instead of at the CTest timeout.
 */
static void *interrupt_later(void *unused)
{
    const struct timespec tick = {.tv_nsec = 10000000};
    (void)unused;
    for (int i = 0; i < 500 && atomic_load(&stub_state) != STUB_RUNNING; ++i)
        nanosleep(&tick, NULL);
    if (atomic_load(&stub_state) != STUB_RUNNING)
        return NULL;
    nanosleep(&(struct timespec){.tv_nsec = 200000000}, NULL);
    kill(getpid(), SIGINT);
    return NULL;
}

/* Send SIGINT while the stub open pauses, as a user would during a slow boot.
 * Give up after five seconds, like interrupt_later.
 */
static void *interrupt_during_open(void *unused)
{
    const struct timespec tick = {.tv_nsec = 10000000};
    (void)unused;
    for (int i = 0; i < 500 && !atomic_load(&stub_opening); ++i)
        nanosleep(&tick, NULL);
    if (atomic_load(&stub_opening))
        kill(getpid(), SIGINT);
    return NULL;
}

int main(void)
{
    char *argv[] = {(char *)"hydra_udp", (char *)"--host", (char *)"127.0.0.1",
                    (char *)"--port", (char *)"9", NULL};
    pthread_t interrupter;
    int result;

    if (pthread_create(&interrupter, NULL, interrupt_later, NULL))
        return 1;
    result = hydra_udp_program_main(5, argv);
    pthread_join(interrupter, NULL);
    if (result != 0 || atomic_load(&stub_reads) != 1) {
        fprintf(stderr, "Ctrl-C did not stop a read without callbacks (exit %d)\n", result);
        return 1;
    }

    /* Clear the flag and send a real SIGINT while the device opens. The program
     * must close the device without starting a read. Restore the default action
     * first, since the first run's handler is still installed: a handler that
     * the program installs only after setup would then let the signal terminate
     * this test.
     */
    signal(SIGINT, SIG_DFL);
    atomic_store(&interrupted, false);
    atomic_store(&stub_slow_open, true);
    if (pthread_create(&interrupter, NULL, interrupt_during_open, NULL))
        return 1;
    result = hydra_udp_program_main(5, argv);
    pthread_join(interrupter, NULL);
    if (result != 0 || !atomic_load(&interrupted) || atomic_load(&stub_reads) != 1 ||
        atomic_load(&stub_closes) != 2) {
        fprintf(stderr, "Ctrl-C during setup still started a read (exit %d)\n", result);
        return 1;
    }
    puts("UDP interrupt test passed");
    return 0;
}
