/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Luigi C.
 */
#include "internal.h"
#include "chips/mt7921.h"
#include "chips/mt7921_image.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(test) do { if (!(test)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #test); exit(1); } } while (0)

/* Count parser deliveries without depending on DSP or external USB hardware. */
static int payload(void *context, const uint8_t *raw)
{
    unsigned *count = context;
    int16_t iq[BS_PACKET_SAMPLES * 2];
    CHECK(bs_decode_iq(raw, iq, BS_PACKET_SAMPLES) == 0);
    CHECK(bs_get_le32(raw + BS_PAYLOAD_BYTES - 4) == 0x12340567);
    ++*count;
    return 0;
}

/* Construct a protocol fixture independently of any USB transfer boundaries. */
static void make_frame(uint8_t frame[BS_FRAME_BYTES], uint32_t sequence)
{
    memset(frame, 0, BS_FRAME_BYTES);
    bs_put_le32(frame, 0x38000000u | (BS_FRAME_BYTES - 4));
    bs_put_le32(frame + 24, 0x38000014u + BS_PAYLOAD_BYTES);
    frame[28] = 0xed;
    frame[32] = 0xf5;
    bs_put_le32(frame + 36, sequence);
    bs_put_le32(frame + 40, 0x12345678);
    bs_put_le32(frame + BS_HEADER_BYTES - 4, 0xffffffff); /* Padding is not I/Q. */
    bs_put_le32(frame + BS_HEADER_BYTES + BS_PAYLOAD_BYTES - 4, 0x12340567); /* Last word is real IQ. */
}

/* Every split point must produce exactly one packet, including bytewise headers.
 * Then verify gaps, stale nonces, duplicates, truncation and malformed lengths.
 */
static void test_framing(void)
{
    uint8_t frame[BS_FRAME_BYTES];
    make_frame(frame, 0);
    for (size_t split = 0; split <= sizeof(frame); ++split) {
        unsigned count = 0;
        struct bs_framer f = {.nonce = 0x12345678, .payload = payload, .context = &count};
        CHECK(bs_framer_feed(&f, frame, split) == 0);
        CHECK(bs_framer_feed(&f, frame + split, sizeof(frame) - split) == 0);
        CHECK(bs_framer_finish(&f, 1) == 0 && count == 1);
        CHECK(bs_framer_feed(&f, frame, sizeof(frame)) == HYDRASDR_ERROR_PROTOCOL);
    }
    for (unsigned kind = 0; kind < 4; ++kind) {
        unsigned count = 0;
        struct bs_framer f = {.nonce = 0x12345678, .payload = payload, .context = &count};
        make_frame(frame, 0);
        if (kind == 0) frame[36] = 1;
        if (kind == 1) frame[40] ^= 1;
        if (kind == 2) frame[0] = 0;
        if (kind == 3) frame[32] = 0xf4;
        CHECK(bs_framer_feed(&f, frame, sizeof(frame)) == HYDRASDR_ERROR_PROTOCOL);
        CHECK(count == 0);
    }
    {
        unsigned count = 0;
        struct bs_framer f = {.nonce = 0x12345678, .payload = payload, .context = &count};
        make_frame(frame, 0);
        CHECK(bs_framer_feed(&f, frame, sizeof(frame) - 1) == 0);
        CHECK(bs_framer_finish(&f, 1) == HYDRASDR_ERROR_PROTOCOL);
    }
}

struct gap_observation { unsigned count; uint32_t state; uint64_t packets; };

/* Markers carry no samples and must precede the next packet at this sequence. */
static int observe_gap(void *context, uint32_t state, uint64_t packets)
{
    struct gap_observation *o = context;
    ++o->count;
    o->state = state;
    o->packets = packets;
    return 0;
}

/* Exercise every marker split, strict readers, duplicates, missing markers,
 * invalid reasons and both wrapping counters. A marker never advances the
 * data packet count or bypasses the acquisition nonce check.
 */
static void test_gap_events(void)
{
    uint8_t marker[48] = {0};
    bs_put_le32(marker, 0x3800002c);
    bs_put_le32(marker + 8, 9);
    bs_put_le32(marker + 12, 1);
    bs_put_le32(marker + 24, 0x38000014);
    marker[28] = 0xed;
    marker[32] = 0xf6;
    bs_put_le32(marker + 36, 3);
    bs_put_le32(marker + 40, 0x12345678);
    for (size_t split = 0; split <= sizeof(marker); ++split) {
        struct gap_observation o = {0};
        struct bs_framer f = {.nonce = 0x12345678, .packets = 3,
                              .gap = observe_gap, .context = &o};
        CHECK(bs_framer_feed(&f, marker, split) == 0);
        CHECK(bs_framer_feed(&f, marker + split, sizeof(marker) - split) == 0);
        CHECK(o.count == 1 && o.state == 9 && o.packets == 3);
        CHECK(f.packets == 3 && f.gaps == 1 && bs_framer_finish(&f, 3) == 0);
        CHECK(bs_framer_feed(&f, marker, sizeof(marker)) == HYDRASDR_ERROR_PROTOCOL);
    }
    const unsigned corrupt[] = {8, 12, 24, 28, 32, 36, 40};
    for (unsigned n = 0; n < sizeof(corrupt) / sizeof(corrupt[0]); ++n) {
        struct gap_observation o = {0};
        struct bs_framer f = {.nonce = 0x12345678, .packets = 3,
                              .gap = observe_gap, .context = &o};
        marker[corrupt[n]] ^= 2;
        CHECK(bs_framer_feed(&f, marker, sizeof(marker)) == HYDRASDR_ERROR_PROTOCOL);
        CHECK(!o.count);
        marker[corrupt[n]] ^= 2;
    }
    struct bs_framer strict = {.nonce = 0x12345678, .packets = 3};
    CHECK(bs_framer_feed(&strict, marker, sizeof(marker)) == HYDRASDR_ERROR_PROTOCOL);
    struct gap_observation o = {0};
    struct bs_framer f = {.nonce = 0x12345678, .packets = UINT64_C(1) << 32,
                          .gaps = UINT32_MAX, .gap = observe_gap, .context = &o};
    bs_put_le32(marker + 8, 8);
    bs_put_le32(marker + 12, 0);
    bs_put_le32(marker + 36, 0);
    CHECK(bs_framer_feed(&f, marker, sizeof(marker)) == 0);
    CHECK(o.count == 1 && o.state == 8 && o.packets == (UINT64_C(1) << 32));
    CHECK(f.gaps == 0);
}

/* Known signed extrema catch I/Q reversal, sign-extension and accidental scaling. */
static void test_samples(void)
{
    uint8_t raw[8] = {0xff, 0x1f, 0x00, 0x20, 0xff, 0x3f, 0x01, 0x00};
    int16_t iq[4];
    CHECK(bs_decode_iq(raw, iq, 2) == 0);
    CHECK(iq[0] == -8192 && iq[1] == 8191 && iq[2] == 1 && iq[3] == -1);
    raw[7] = 0x40;
    CHECK(bs_decode_iq(raw, iq, 2) == HYDRASDR_ERROR_PROTOCOL);
}

/* The 20 MSps packer keeps two samples and two zero words per 16-byte record. */
static void test_halved_samples(void)
{
    uint8_t raw[32] = {0};
    int16_t iq[8];
    raw[0] = 0xff; raw[1] = 0x1f; raw[2] = 0x00; raw[3] = 0x20;  /* I -8192, Q 8191 */
    raw[4] = 0xff; raw[5] = 0x3f; raw[6] = 0x01; raw[7] = 0x00;  /* I 1, Q -1 */
    raw[16] = 0x05; raw[22] = 0x07;                              /* Q 5, then I 7 */
    CHECK(bs_decode_iq_halved(raw, iq, 8) == 0);
    CHECK(iq[0] == -8192 && iq[1] == 8191 && iq[2] == 1 && iq[3] == -1);
    CHECK(iq[4] == 0 && iq[5] == 5 && iq[6] == 7 && iq[7] == 0);
    CHECK(bs_decode_iq_halved(raw, iq, 6) == HYDRASDR_ERROR_INVALID);
    raw[9] = 0x01;                                               /* non-zero padding */
    CHECK(bs_decode_iq_halved(raw, iq, 8) == HYDRASDR_ERROR_PROTOCOL);
    raw[9] = 0;
    raw[7] = 0x40;                                               /* invalid sample bits */
    CHECK(bs_decode_iq_halved(raw, iq, 8) == HYDRASDR_ERROR_PROTOCOL);
}

/* A start acknowledgment may arrive fragmented beside sample events. Reject
 * duplicate/nonzero ACKs and carry packet sequence through its wire rollover.
 */
static void test_events(void)
{
    uint8_t ack[40] = {0}, frame[BS_FRAME_BYTES];
    unsigned count = 0;
    struct bs_framer framer = {.nonce = 0x12345678, .payload = payload, .context = &count};
    bs_put_le32(ack, 0x38000024);
    ack[28] = 1;
    CHECK(bs_framer_feed(&framer, ack, 17) == 0);
    CHECK(bs_framer_feed(&framer, ack + 17, 23) == 0);
    make_frame(frame, 0);
    CHECK(bs_framer_feed(&framer, frame, sizeof(frame)) == 0);
    CHECK(framer.acknowledgements == 1 && count == 1);
    CHECK(bs_framer_feed(&framer, ack, sizeof(ack)) == HYDRASDR_ERROR_PROTOCOL);
    framer = (struct bs_framer){.nonce = 0x12345678, .payload = payload, .context = &count};
    ack[32] = 1;
    CHECK(bs_framer_feed(&framer, ack, sizeof(ack)) == HYDRASDR_ERROR_PROTOCOL);
    framer = (struct bs_framer){.nonce = 0x12345678, .payload = payload, .context = &count,
                              .packets = UINT32_MAX};
    make_frame(frame, UINT32_MAX);
    CHECK(bs_framer_feed(&framer, frame, sizeof(frame)) == 0);
    make_frame(frame, 0);
    CHECK(bs_framer_feed(&framer, frame, sizeof(frame)) == 0);
    CHECK(bs_framer_finish(&framer, UINT64_C(4294967297)) == 0);
}

/* Every signed 14-bit component must survive decode, including both extrema.
 * Complement Q so reversing the two components cannot accidentally pass.
 */
static void test_all_sample_values(void)
{
    uint8_t raw[4];
    int16_t iq[2];
    for (uint32_t value = 0; value < 16384; ++value) {
        uint32_t q = 16383 - value;
        int expected_i = value < 8192 ? (int)value : (int)value - 16384;
        int expected_q = q < 8192 ? (int)q : (int)q - 16384;
        bs_put_le32(raw, value << 16 | q);
        CHECK(bs_decode_iq(raw, iq, 1) == 0);
        CHECK(iq[0] == expected_i && iq[1] == expected_q);
    }
}

/* Verify the embedded research image's PDA trailer and ROM download bounds.
 * The N9 image evolves as diagnostics are added; hardcoding its historic
 * 1632-byte length silently breaks otherwise valid new firmware builds.
 */
static void test_image(void)
{
    uint32_t state = 0;
    CHECK(sizeof(mt7921_image) >= 64);
    /* The original single-scatter N9 image was limited to 4KiB. The
     * extended native image is intentionally segmented by ROM PDA into
     * <=4080B scatter frames; the PDA checksum covers the entire image.
     * Preserve a conservative 16KiB code +16B trailer bound. */
    CHECK(sizeof(mt7921_image) <= 16384u + 16u);
    CHECK(sizeof(mt7921_image) % 16 == 0);
    size_t scatter_bytes=0,scatter_frames=0;
    while (scatter_bytes < sizeof(mt7921_image)) {
        size_t remain=sizeof(mt7921_image)-scatter_bytes;
        size_t chunk=remain>4080u?4080u:remain;
        CHECK(chunk && chunk<=4080u && !(chunk&3u));
        scatter_bytes+=chunk;
        scatter_frames++;
    }
    CHECK(scatter_bytes==sizeof(mt7921_image) && scatter_frames<=5u);
    for (size_t i = 0; i < sizeof(mt7921_image); i += 4) {
        uint32_t bits = state & 0x10921111, parity = 0;
        while (bits) { parity ^= bits & 1; bits >>= 1; }
        state = (state >> 1) ^ parity << 31 ^ bs_get_le32(mt7921_image + i);
    }
    CHECK(state == 0);
}

/* Unsupported chip/control paths must fail without USB access or pretend values. */
static void test_api(void)
{
    hydrasdr_dev_t dev = {0};
    const struct bs_usb_id *id = bs_lookup_id(0x0e8d, 0x7925);
    struct hydrasdr_gain gain = {2, 8, 0};
    uint64_t minimum, maximum;
    atomic_init(&dev.run_state, BS_IDLE);
    CHECK(pthread_mutex_init(&dev.control_mutex, NULL) == 0);
    dev.backend = &bs_mt7921_backend;
    dev.frequency = dev.backend->default_frequency;
    dev.rate = dev.backend->default_rate;
    CHECK(id && id->backend->chipset == HYDRASDR_CHIP_MT7925 && !id->qualified);
    CHECK(id->backend->prepare(&dev, 0) == HYDRASDR_ERROR_UNSUPPORTED);
    CHECK(hydrasdr_set_tuner_gain(&dev, 100) == HYDRASDR_ERROR_UNSUPPORTED);
    CHECK(hydrasdr_set_agc_mode(&dev, 1) == HYDRASDR_ERROR_UNSUPPORTED);
    CHECK(hydrasdr_set_tuner_bandwidth(&dev, 40000000) == HYDRASDR_ERROR_UNSUPPORTED);
    CHECK(hydrasdr_set_center_freq(&dev, UINT64_C(3000000000)) == HYDRASDR_ERROR_UNSUPPORTED);
    CHECK(hydrasdr_set_center_freq(&dev, UINT64_C(4499999999)) == HYDRASDR_ERROR_UNSUPPORTED);
    CHECK(hydrasdr_set_center_freq(&dev, UINT64_C(6200000001)) == HYDRASDR_ERROR_UNSUPPORTED);
    CHECK(hydrasdr_set_center_freq(&dev, UINT64_C(5180000000)) == 0);
    CHECK(hydrasdr_get_center_freq(&dev) == UINT64_C(5180000000));
    CHECK(hydrasdr_get_tuning_range(&dev, &minimum, &maximum) == 0);
    CHECK(minimum == UINT64_C(2156000000) && maximum == UINT64_C(6200000000));
    CHECK(hydrasdr_get_tuning_band(&dev, 1, &minimum, &maximum) == 0);
    CHECK(minimum == UINT64_C(4500000000) && maximum == UINT64_C(6200000000));
    CHECK(hydrasdr_set_center_freq(&dev, minimum) == 0);
    CHECK(hydrasdr_set_center_freq(&dev, maximum) == 0);
    CHECK(hydrasdr_get_tuning_band(&dev, 2, &minimum, &maximum) == HYDRASDR_ERROR_INVALID);
    CHECK(hydrasdr_get_tuning_band(&dev, 0, &minimum, &maximum) == 0);
    CHECK(hydrasdr_get_capabilities(&dev) & HYDRASDR_CAP_TUNING);
    CHECK(minimum == UINT64_C(2156000000) && maximum == UINT64_C(2610000000));
    CHECK(hydrasdr_set_center_freq(&dev, minimum) == 0);
    CHECK(hydrasdr_set_center_freq(&dev, maximum) == 0);
    CHECK(hydrasdr_set_center_freq(&dev, minimum - 1) == HYDRASDR_ERROR_UNSUPPORTED);
    CHECK(hydrasdr_set_center_freq(&dev, maximum + 1) == HYDRASDR_ERROR_UNSUPPORTED);
    CHECK(hydrasdr_get_center_freq(&dev) == maximum);
    CHECK(hydrasdr_set_center_freq(&dev, UINT64_C(2437123456)) == 0);
    CHECK(hydrasdr_get_center_freq(&dev) == UINT64_C(2437123456));
    id = bs_lookup_id(0x0e8d, 0x7961);
    CHECK(id && id->qualified && id->min_frequency == UINT64_C(2190000000) && !id->max_frequency);
    dev.min_frequency = id->min_frequency;
    CHECK(hydrasdr_get_tuning_band(&dev, 0, &minimum, &maximum) == 0);
    CHECK(minimum == UINT64_C(2190000000) && maximum == UINT64_C(2610000000));
    CHECK(hydrasdr_get_tuning_band(&dev, 1, &minimum, &maximum) == 0);
    CHECK(minimum == UINT64_C(4500000000) && maximum == UINT64_C(6200000000));
    dev.max_frequency = UINT64_C(2610000000);
    CHECK(hydrasdr_get_tuning_band(&dev, 1, &minimum, &maximum) == HYDRASDR_ERROR_UNSUPPORTED);
    CHECK(hydrasdr_set_center_freq(&dev, UINT64_C(5180000000)) == HYDRASDR_ERROR_UNSUPPORTED);
    dev.max_frequency = 0;
    CHECK(hydrasdr_get_tuning_range(&dev, &minimum, &maximum) == 0);
    CHECK(minimum == UINT64_C(2190000000) && maximum == UINT64_C(6200000000));
    CHECK(hydrasdr_set_center_freq(&dev, minimum - 1) == HYDRASDR_ERROR_UNSUPPORTED);
    CHECK(hydrasdr_get_center_freq(&dev) == UINT64_C(2437123456));
    CHECK(hydrasdr_set_center_freq(&dev, minimum) == 0);
    dev.min_frequency = 0;
    CHECK(hydrasdr_get_tuning_range(&dev, &minimum, &maximum) == 0);
    CHECK(minimum == UINT64_C(2156000000));
    CHECK(hydrasdr_set_sample_rate(&dev, 0) == HYDRASDR_ERROR_UNSUPPORTED);
    CHECK(hydrasdr_set_sample_rate(&dev, 5000000) == HYDRASDR_ERROR_UNSUPPORTED);
    CHECK(hydrasdr_set_sample_rate(&dev, 10000000) == 0);
    CHECK(hydrasdr_set_sample_rate(&dev, 60000000) == HYDRASDR_ERROR_UNSUPPORTED);
    CHECK(hydrasdr_set_sample_rate(&dev, 20000000) == 0);
    CHECK(hydrasdr_get_sample_rate(&dev) == 20000000);
    CHECK(hydrasdr_set_sample_rate(&dev, 80000000) == 0);
    CHECK(hydrasdr_get_sample_rate(&dev) == 80000000);
    CHECK(hydrasdr_set_sample_rate(&dev, 40000000) == 0);
    CHECK(hydrasdr_get_sample_rate(&dev) == 40000000);
    CHECK(hydrasdr_set_gain_codes(&dev, &gain) == HYDRASDR_ERROR_STATE);
    CHECK(hydrasdr_set_tuner_gain_mode(&dev, 1) == 0);
    CHECK(hydrasdr_set_gain_codes(&dev, &gain) == 0);
    gain.lna = 5;
    CHECK(hydrasdr_set_gain_codes(&dev, &gain) == 0);
    gain.lna = 6;
    CHECK(hydrasdr_set_gain_codes(&dev, &gain) == HYDRASDR_ERROR_INVALID);
    gain.lna = 9;
    CHECK(hydrasdr_set_gain_codes(&dev, &gain) == HYDRASDR_ERROR_INVALID);
    CHECK(hydrasdr_cancel_async(&dev) == HYDRASDR_ERROR_STATE);
    CHECK(atomic_load(&dev.run_state) == BS_IDLE);
    atomic_store(&dev.run_state, BS_RUNNING);
    CHECK(hydrasdr_cancel_async(&dev) == 0);
    CHECK(atomic_load(&dev.run_state) == BS_CANCELLING);
    CHECK(hydrasdr_cancel_async(&dev) == 0);
    CHECK(hydrasdr_set_tuner_gain_mode(&dev, 0) == HYDRASDR_ERROR_BUSY);
    CHECK(hydrasdr_set_center_freq(&dev, minimum) == HYDRASDR_ERROR_BUSY);
    CHECK(hydrasdr_close(&dev) == HYDRASDR_ERROR_BUSY);
    CHECK(pthread_mutex_destroy(&dev.control_mutex) == 0);
}

/* Tests deliberately run without a dongle, root privileges or vendor firmware. */
int main(void)
{
    test_framing();
    test_gap_events();
    test_samples();
    test_halved_samples();
    test_events();
    test_all_sample_values();
    test_image();
    test_api();
    puts("core tests passed");
    return 0;
}
