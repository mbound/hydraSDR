/* SPDX-License-Identifier: MIT
 * Manual no-RF host-defined IQ ingress test payload contract.
 * Rolling checksum is for accidental corruption only, not authentication.
 */
#ifndef HYDRA_INGRESS_IQ_PAYLOAD_H
#define HYDRA_INGRESS_IQ_PAYLOAD_H
#include <stdint.h>
/* Exact externally observed RX1 and private SRAM ownership contract. */
#define HYDRA_RX1_DMA_SLOTS 4u
#define HYDRA_RX1_DMA_DESCRIPTOR_BASE UINT32_C(0x0203c800)
#define HYDRA_RX1_DMA_PAYLOAD_BASE UINT32_C(0x0203d000)
#define HYDRA_RX1_STAGE_SLOTS 16u
#define HYDRA_RX1_STAGE_PAYLOAD_BASE UINT32_C(0x0203e000)
#define HYDRA_RX0_DMA_DESCRIPTOR_BASE UINT32_C(0x02040000)
#define HYDRA_USB_N9_SLOT_BYTES 512u
#define HYDRA_INGRESS_IQ_CHECKSUM_SEED UINT32_C(0x9e3779b9)
#define HYDRA_INGRESS_IQ_WORDS_PER_PACKET 64u
static inline uint32_t hydra_ingress_iq_word(uint64_t sample,
                                             uint64_t packet,uint32_t nonce)
{
    /* Both signed 12-bit lanes are stored as two's-complement 12-bit words
     * with reserved [15:12] and [31:28] clear. The host chooses every IQ
     * value; neither the N9 nor its DMA fabric generates the sequence. */
    uint32_t bits=(uint32_t)sample ^ (uint32_t)packet ^ nonce;
    bits ^= bits>>16;bits*=UINT32_C(0x7feb352d);
    bits ^= bits>>15;bits*=UINT32_C(0x846ca68b);
    bits ^= bits>>16;
    return (bits&UINT32_C(0x0fff))|
           (((bits>>12)&UINT32_C(0x0fff))<<16);
}
/* R2 first active-RF safety pattern. Fresh host-defined pseudo-random
 * QPSK is limited to +/-64 signed12 per lane, not the full-range ingress
 * stress pattern (+/-2048) that could overload an uncalibrated RF path.
 * Values retain exact 12-bit twos-complement packing and are independently
 * hash-checked by N9 before PHY setpoint writes. */
static inline uint32_t hydra_ingress_rf_qpsk_word(uint64_t sample,
                                                uint64_t packet,uint32_t nonce)
{
    const uint32_t bits=hydra_ingress_iq_word(sample,packet,nonce);
    const uint32_t i=(bits&1u)?64u:0xfc0u;
    const uint32_t q=(bits&0x10000u)?64u:0xfc0u;
    return i|(q<<16);
}
static inline uint32_t hydra_ingress_iq_checksum_step(uint32_t hash,uint32_t word)
{
    return 33u*hash + word; /* 32-bit wrap, exact same algorithm on N9 */
}
#endif
