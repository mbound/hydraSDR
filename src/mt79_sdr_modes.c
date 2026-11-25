/* SPDX-License-Identifier: MIT */
#include "mt79_sdr_modes.h"
#include <stddef.h>
#include <string.h>

int mt79_sdr_plan(enum mt79_sdr_rx_mode mode,uint32_t rate,
                  struct mt79_sdr_mode_plan *out)
{
    if (!out) return -1;
    struct mt79_sdr_mode_plan plan;
    memset(&plan,0,sizeof(plan));
    plan.mode=mode;
    plan.rate_per_channel=rate;
    if (mode==MT79_SDR_RX1) {
        /* Tap13 carries energy in preliminary tests; physical path identity
         * and phase/gain behavior have not yet been isolated and verified. */
        return -1;
    }
    if (mode==MT79_SDR_RX0) {
        if (rate!=10000000 && rate!=20000000 &&
            rate!=40000000 && rate!=80000000)return -1;
        plan.layout=MT79_CAPTURE_DENSE_QI14;
        plan.channel_count=1;
        plan.dfe_source_word=0x80000013u;
        plan.dfe_mux_word=(rate==20000000||rate==80000000)
                          ? 0x00880400u:0x008c0400u;
        plan.expected_complex_per_channel_per_usb_packet =
                          rate<=20000000?1024u:2048u;
        plan.expected_usb_payload_bytes_per_sec =
                          rate==10000000?80000000ULL:
                          rate==20000000?160000000ULL:
                          4ULL*rate;
        return *out=plan,0;
    }
    if (mode==MT79_SDR_DUAL) {
        /* At present only the full 2x40 Msps RAW2 format is live verified.
         * Never pretend that 2x80 Msps fits the 5Gbps USB link.
         */
        if (rate!=40000000)return -1;
        plan.layout=MT79_CAPTURE_RAW2_INTERLEAVED_QI14;
        plan.channel_count=2;
        plan.dfe_source_word=0x80000014u;
        plan.dfe_mux_word=0x010c0600u;
        plan.expected_complex_per_channel_per_usb_packet=1024;
        plan.expected_usb_payload_bytes_per_sec=320000000ULL;
        plan.rf0_companion_activation=1;
        plan.phase_calibration_required=1; /* Needs per-session delay/phase. */
        return *out=plan,0;
    }
    return -1;
}
