/* SPDX-License-Identifier: MIT
 * Experimental, explicitly gated MT7961 PHY tone-source control.
 * Raw PHY codes; NOT calibrated RF power, arbitrary-rate IQ or a PA API.
 */
#ifndef HYDRA_TX_CONTROL_H
#define HYDRA_TX_CONTROL_H
#include "hydrasdr.h"
#ifdef __cplusplus
extern "C" {
#endif
#if defined(__GNUC__)
#pragma GCC visibility push(default)
#endif
#define HYDRA_TX_REG_COUNT 20u
struct hydra_tx_iq_step {
    int16_t i, q;            /* signed 12-bit generator codes [-2048,2047] */
    uint32_t dwell_us;       /* host-timed, NOT sample synchronous */
};
struct hydra_tx_burst_config {
    uint8_t chain;           /* only demonstrated chain 0 is currently gated */
    uint8_t raw_power_code;  /* exact 2024 WM gain byte [7:0]; NOT dBm */
};
struct hydra_tx_burst_receipt {
    uint64_t host_start_ns, host_stop_ns;
    unsigned completed_steps;
    uint32_t before[HYDRA_TX_REG_COUNT], after[HYDRA_TX_REG_COUNT];
    /* Exact original Feb-2024 WM 0x0093e27a reads C114 low16, adjusts a
     * 16-bit result by steps of 16. Semantics (TSSI? calibration units?)
     * UNKNOWN; capture source-backed raw words before/during/after GO.
     * Order [phase: pre, TX active, restored][bank:0,1]. */
    uint32_t candidate_c114[3][2];
};
/* Bounded, synchronous TX generator execution while native RX is streaming.
 * RX callbacks run in a separate thread. No independent TX/RX LO is claimed.
 * Requires HYDRA_TX_EXPERIMENTAL=1; any nonzero raw power code also requires
 * HYDRA_TX_ALLOW_UNCALIBRATED_POWER=1. Permitted center: 2437 MHz only.
 * Firmware must advertise the device-polled TX watchdog capability.
 * That watchdog shuts off GO if the host crashes while the N9 loop is alive.
 * It is NOT a hardware interrupt/watchdog against an MCU wedge.
 * The caller must still provide an RF-contained test setup.
 */
int hydra_tx_run_bounded_iq(hydrasdr_dev_t *dev,
                            const struct hydra_tx_burst_config *config,
                            const struct hydra_tx_iq_step *steps, unsigned count,
                            struct hydra_tx_burst_receipt *receipt);
/* Manual-only, source-qualified N9 DMA callback during shielded short RF.
 * Hook runs under original single USB/control owner with N9 watchdog armed.
 * The hook must leave DMA STOPPED before returning; the host TX routine
 * still performs its original exact snapshot/rollback/disarm. Never used
 * for normal generic TX or as proof of actual per-sample DAC timing. */
int hydra_tx_run_bounded_iq_with_hook(hydrasdr_dev_t *dev,
          const struct hydra_tx_burst_config *config,
          const struct hydra_tx_iq_step *steps,unsigned count,
          struct hydra_tx_burst_receipt *receipt,
          int (*on_tx_started)(hydrasdr_dev_t *,void *),void *ctx);
/* Manual same-channel 2T2R start: exact-ROM-derived path bitmap 0x3 and
 * watchdog-covered GO/path gates. Physical RF paths 0/1 and coherent common-IQ
 * 2TX are qualified by internal dual-RX observation; this is NOT independent
 * per-antenna IQ or calibrated antenna-port RF. Only raw gain code 0 and a
 * 1..2.5ms bounded seed are accepted. Requires HYDRA_TX_DUAL_EXPERIMENTAL=1,
 * HYDRA_TEST_DUAL_TX_RF_BURST=1 and the normal containment gates. */
struct hydra_tx_dual_iq_seed {
    int16_t i0,q0,i1,q1;
    uint32_t dwell_us;
};
int hydra_tx_run_bounded_dual_iq_with_hook(hydrasdr_dev_t *dev,
          uint8_t raw_power_code,const struct hydra_tx_dual_iq_seed *seed,
          struct hydra_tx_burst_receipt *receipt,
          int (*on_tx_started)(hydrasdr_dev_t *,void *),void *ctx);
/* Opt-in host-delivered, bounded N9-timer-paced IQ test. Each signed 12-bit
 * complex sample is uploaded exactly once into *volatile* MCU SRAM while
 * TX is OFF; the resident N9 then cyclically replays the table by MMIO writes
 * to the PHY's 0x8308FC08 I/Q setpoint register. This does NOT bypass the
 * tone generator's update stage, and is NOT USB bulk continuous DAC input.
 * All TX safety and baseline guards apply; stage requires
 * HYDRA_TX_IQ_STAGED=1 and HYDRA_RF_SHIELDED=1 in addition to experimental
 * mode and optional uncalibrated power permission. 2437MHz, chain0 only.
 * Timer is MCU's 1us timer; jitter/overruns must be measured, never
 * represented as guaranteed symbol timing. N9 TX watchdog is mandatory.
 */
#define HYDRA_TX_STAGED_IQ_MAX_SAMPLES 128u
struct hydra_tx_iq_sample {int16_t i,q;};
/* N9 sequential MMIO snapshot, bracketed by device timer reads. It is
 * emission-associated, not an atomic RF measurement. First and latest
 * every-256th emission; raw values only, no A-die SPI transactions. */
#define HYDRA_TX_ACTIVE_REG_COUNT 26u
struct hydra_tx_active_snapshot {
    uint32_t sequence, begin_us, end_us, emission;
    uint32_t words[HYDRA_TX_ACTIVE_REG_COUNT];
};
const char *hydra_tx_active_reg_name(unsigned index);
struct hydra_tx_staged_iq_receipt {
    struct hydra_tx_burst_receipt tx;
    uint32_t sample_count,period_us,duration_us;
    uint32_t completed_updates,overruns,max_update_delta_us,final_index;
    uint32_t iq_control_after;
    uint32_t n9_first_emit_us,n9_last_emit_us,n9_span_us;
    struct hydra_tx_active_snapshot active[2];
};
int hydra_tx_run_bounded_staged_iq(hydrasdr_dev_t *dev,
                            const struct hydra_tx_burst_config *config,
                            const struct hydra_tx_iq_sample *samples,
                            unsigned sample_count,unsigned period_us,
                            unsigned duration_us,
                            struct hydra_tx_staged_iq_receipt *receipt,
                            /* Optional callback on host START/STOP epoch.
                             * No USB calls or locking permitted inside. */
                            void (*on_tx_phase)(void *ctx,int active),
                            void *phase_ctx);
/* MANUAL R2 TEST ONLY: existing one-device-owner USB RX1 DMA ring must be
 * source-verified, primed with >=3 checked IQ frames, consumer mode3, RF
 * ARM=0. Caller owns dev->control_mutex throughout this function; a host
 * callback sends the remaining fresh HIF bulk-OUT frames (NO control MMIO).
 * Device 1us timer and a 100ms watchdog bound uncontrolled host stalls;
 * host targets <=20ms live RF. Abort ACK precedes register restoration.
 * This is ~50k/100k N9 PHY IQ SETPOINT writes, NOT a DAC DMA/5MSps sink.
 * Not installed in ordinary RF APIs or automatic tests. */
struct hydra_tx_usb_ingress_receipt {
    struct hydra_tx_burst_receipt tx;
    uint32_t producer_packets, consumed_packets, consumed_words;
    uint32_t rf_setpoint_writes, rf_last_iq, rf_first_us, rf_last_us;
    uint32_t consumer_state_before_abort, consumer_underruns;
    uint32_t fifo_highwater, fifo_fill_after, abort_ack;
    uint32_t generator_go_during, watchdog_state_before_disarm;
};
int hydra_tx_run_bounded_usb_iq_locked(hydrasdr_dev_t *dev,
          const struct hydra_tx_burst_config *cfg,
          unsigned expected_samples, unsigned duration_us,
          int (*send_live_frames)(void *ctx), void *send_ctx,
          struct hydra_tx_usb_ingress_receipt *out,
          void (*on_tx_phase)(void *ctx,int active), void *phase_ctx);

/* Passive raw monitoring of the 20 allowlisted registers (same USB owner).
 * Requires running native RX, no conversion to dBm, EVM, TSSI or SINR.
 */
int hydra_tx_read_raw(hydrasdr_dev_t *dev, uint32_t words[HYDRA_TX_REG_COUNT]);
const char *hydra_tx_reg_name(unsigned index);
/* HydraGen4M passive TX frontend observability. Source-backed raw MMIO and
 * MT7972 A-die register reads. These are NOT TX output power or calibrated
 * TSSI measurement; no SET, RF emission, or OTP access takes place.
 * MMIO order: 0/1 TSSI control, 2/3 power override, 4/5 TX C700 mode/word,
 * 6/7 TX path, 8/9 tone GO, 10/11 tone enable, 12/13 tone-IQ,
 * 14/15 TX mux. A-die reads RF-SPI port0/port1 register 0x768 (raw gain).
 */
#define HYDRA_TX_FE_MMIO_COUNT 16u
struct hydra_tx_fe_status {
    uint64_t host_begin_ns, host_end_ns;
    uint32_t mmio[HYDRA_TX_FE_MMIO_COUNT];
    uint32_t adie_gain_raw[2];
};
const char *hydra_tx_fe_reg_name(unsigned index);
int hydra_tx_fe_observe(hydrasdr_dev_t *dev, struct hydra_tx_fe_status *out);
/* Purely read-only RX gain/source status at the same host control epoch.
 * Eight PHY raw MMIO words ordered [C4,108,150,D0] per bank 0/1 and
 * 0x768 per physical A-die RF SPI port 0/1. No dB/AGC inference.
 */
#define HYDRA_RX_FE_MMIO_COUNT 8u
struct hydra_rx_fe_status {
    uint64_t host_begin_ns,host_end_ns;
    uint32_t phy_mmio[HYDRA_RX_FE_MMIO_COUNT];
    uint32_t adie_gain_raw[2];
};
const char *hydra_rx_fe_reg_name(unsigned index);
int hydra_rx_fe_observe(hydrasdr_dev_t *dev, struct hydra_rx_fe_status *out);
/* Read-only native N9 watchdog status and cumulative expiration counter.
 * Queries the running firmware under the single control mutex. State:
 * 0 disarmed; 1 armed; 2 expired/sticky until WFSYS reset.
 */
int hydra_tx_watchdog_status(hydrasdr_dev_t *dev, uint32_t *state,
                             uint32_t *expirations);
#if defined(__GNUC__)
#pragma GCC visibility pop
#endif
#ifdef __cplusplus
}
#endif
#endif
