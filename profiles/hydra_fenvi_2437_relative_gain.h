/* SPDX-License-Identifier: MIT
 * SOURCE-QUALIFIED measurement profile, NOT factory calibration.
 * Fenvi 0e8d:7961 / MT7972 single RX0, 2437MHz/10Msps, two
 * source-unknown ambient-IQ power sweeps; no traceable RF power standard.
 * Values are tenths of dB ABOVE raw stage 0, not absolute dBm, and
 * include unqualified separability (one four-combination check +2.8dB).
 * Do not silently apply to generic 0e8d:7961 adapters or 5GHz.
 */
#ifndef HYDRA_FENVI_RELATIVE_GAIN_20261123_H
#define HYDRA_FENVI_RELATIVE_GAIN_20261123_H
#include "hydrasdr.h"
static const struct hydrasdr_relative_gain_profile
hydra_fenvi_2437_provisional = {
    .version = HYDRASDR_REL_GAIN_PROFILE_VERSION,
    .source_basis = HYDRASDR_REL_GAIN_AMBIENT_PROVISIONAL,
    .provenance = "fenvi-ax1800-0e8d7961-20261123-ambient-iq-two-pass-v1",
    .frequency_hz = UINT64_C(2437000000),
    .sample_rate = 10000000u,
    .rx_mode = HYDRASDR_RX_SINGLE0,
    .rx_chain = 0,
    .lna_mask = 0x03fu,
    .lpf_mask = 0x1ffu,
    .digital_mask = 0x1555u,
    .lna_tenths = {0,80,137,196,260,298},
    .lpf_tenths = {0,13,34,62,92,109,122,144,159},
    .digital_tenths = {0,0,27,0,54,0,69,0,94,0,115,0,136},
};
#endif
