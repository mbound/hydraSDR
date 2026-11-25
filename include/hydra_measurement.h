/* SPDX-License-Identifier: MIT */
#ifndef HYDRA_MEASUREMENT_H
#define HYDRA_MEASUREMENT_H
#include "hydrasdr.h"
#ifdef __cplusplus
extern "C" {
#endif
#if defined(__GNUC__)
#pragma GCC visibility push(default)
#endif
/* Sample-domain power. No RF dBm, RCPI, noise floor or SINR is inferred.
 * Full scale is a complex tone with magnitude 8192; a corner sample is +3 dBFS.
 * Each update is atomic on validation failure. Caller owns synchronization.
 * first_sample counts multiplexed complex words, including both lanes in DUAL.
 */
struct hydra_iq_accumulator {
    uint64_t next_word, words;
    unsigned channels;
    struct { uint64_t count, sum_power, clipped; int64_t sum_i, sum_q;
             uint32_t peak_power; } lane[2];
};
struct hydra_iq_result {
    uint64_t count, clipped;
    double mean_i, mean_q, mean_power, ac_power, peak_power;
    double power_dbfs, ac_dbfs, peak_dbfs;
};
int hydra_iq_init(struct hydra_iq_accumulator *a, unsigned channels, uint64_t first_word);
int hydra_iq_update(struct hydra_iq_accumulator *a, const int16_t *iq,
                    uint32_t words, uint64_t first_word);
int hydra_iq_result(const struct hydra_iq_accumulator *a, unsigned lane,
                    struct hydra_iq_result *out);
/* Read the existing allowlisted RF inventory under the stream control lock.
 * Bracketed sequential host reads, NOT an atomic hardware sample or PPDU latch.
 * Never overwrites the cached PRE_GO snapshot. No new MMIO addresses.
 * Call from a control thread; RF-SPI traffic can perturb USB throughput.
 * Available only during native RX, after control_ready, before cancellation.
 */
struct hydra_frontend_observation {
    uint64_t host_begin_ns, host_end_ns;
    struct hydrasdr_gain requested_gain;
    int manual_requested;
    struct hydrasdr_gain requested_gain_pair[2];
    int independent_gain_requested;
    struct hydrasdr_rf_cal_snapshot registers;
};
/* Independent raw RX codes for DUAL. Staged while idle or applied under the
 * existing stream control mutex. Both banks roll back on partial failure.
 * Requires manual mode. Legacy common-gain setters replace this pair on success;
 * legacy getter returns STATE while a pair is selected, preventing ambiguity.
 * Physical antennas and dB gains remain uncalibrated.
 */
int hydra_set_rx_gain_pair(hydrasdr_dev_t *dev, const struct hydrasdr_gain gains[2]);
int hydra_get_rx_gain_pair(hydrasdr_dev_t *dev, struct hydrasdr_gain gains[2], int *manual);
#define HYDRA_RF_PHASE_LIVE_SEQUENTIAL 2u
int hydra_observe_frontend(hydrasdr_dev_t *dev, struct hydra_frontend_observation *out);
#if defined(__GNUC__)
#pragma GCC visibility pop
#endif
#ifdef __cplusplus
}
#endif
#endif
