/* SPDX-License-Identifier: MIT
 * No-device tests of the precise manual host->N9 signed12 IQ wire contract.
 */
#include "ingress_iq_payload.h"
#include <stdio.h>
#include <stdlib.h>

#define CHECK(test) do {if(!(test)){fprintf(stderr,"%s:%d %s\n",__FILE__,__LINE__,#test);exit(1);}}while(0)

int main(void)
{
    /* These source-verified SRAM regions may not overlap the existing RX0
     * descriptors, source-independent 128-point TX table or live MMIO snapshots. */
    CHECK(HYDRA_RX1_DMA_SLOTS==4u);
    CHECK(HYDRA_RX1_STAGE_SLOTS==16u);
    CHECK(HYDRA_RX1_DMA_PAYLOAD_BASE+HYDRA_RX1_DMA_SLOTS*HYDRA_USB_N9_SLOT_BYTES
          <=HYDRA_RX1_STAGE_PAYLOAD_BASE);
    CHECK(HYDRA_RX1_STAGE_PAYLOAD_BASE+HYDRA_RX1_STAGE_SLOTS*HYDRA_USB_N9_SLOT_BYTES
          <=HYDRA_RX0_DMA_DESCRIPTOR_BASE);
    CHECK(HYDRA_RX1_DMA_DESCRIPTOR_BASE+HYDRA_RX1_DMA_SLOTS*16u
          <=HYDRA_RX1_DMA_PAYLOAD_BASE);
    const uint32_t nonce=UINT32_C(0xb1771a5d);
    uint32_t a[HYDRA_INGRESS_IQ_WORDS_PER_PACKET],b[HYDRA_INGRESS_IQ_WORDS_PER_PACKET];
    uint32_t hash_a=HYDRA_INGRESS_IQ_CHECKSUM_SEED,hash_b=HYDRA_INGRESS_IQ_CHECKSUM_SEED;
    unsigned negative_i=0,negative_q=0,positive_i=0,positive_q=0;
    for(unsigned n=0;n<HYDRA_INGRESS_IQ_WORDS_PER_PACKET;n++){
        a[n]=hydra_ingress_iq_word((uint64_t)UINT32_MAX-32u+n,7,nonce);
        b[n]=hydra_ingress_iq_word((uint64_t)UINT32_MAX-32u+n,8,nonce);
        CHECK(!(a[n]&UINT32_C(0xf000f000)));
        CHECK(a[n]==hydra_ingress_iq_word((uint64_t)UINT32_MAX-32u+n,7,nonce));
        int16_t i=(int16_t)((a[n]&0x0fffu)^0x800u)-2048;
        int16_t q=(int16_t)(((a[n]>>16)&0x0fffu)^0x800u)-2048;
        CHECK(i>=-2048 && i<=2047 && q>=-2048 && q<=2047);
        if(i<0)negative_i++;else positive_i++;
        if(q<0)negative_q++;else positive_q++;
        hash_a=hydra_ingress_iq_checksum_step(hash_a,a[n]);
        hash_b=hydra_ingress_iq_checksum_step(hash_b,b[n]);
    }
    CHECK(negative_i && positive_i && negative_q && positive_q);
    CHECK(hash_a!=hash_b);
    uint32_t altered=HYDRA_INGRESS_IQ_CHECKSUM_SEED;
    for(unsigned n=0;n<HYDRA_INGRESS_IQ_WORDS_PER_PACKET;n++)
        altered=hydra_ingress_iq_checksum_step(altered,a[n]^(n==17?1u:0u));
    CHECK(altered!=hash_a);
    uint32_t rfi_pos=0,rfi_neg=0,rfq_pos=0,rfq_neg=0,rf_hash=HYDRA_INGRESS_IQ_CHECKSUM_SEED;
    for(unsigned n=0;n<1024u;n++){
        uint32_t w=hydra_ingress_rf_qpsk_word(n,n/64u,nonce);
        uint32_t wi=w&0xfffu,wq=(w>>16)&0xfffu;
        CHECK(!(w&UINT32_C(0xf000f000)));
        CHECK((wi==64u || wi==0xfc0u) && (wq==64u || wq==0xfc0u));
        rfi_pos+=(wi==64u);rfi_neg+=(wi==0xfc0u);
        rfq_pos+=(wq==64u);rfq_neg+=(wq==0xfc0u);
        rf_hash=hydra_ingress_iq_checksum_step(rf_hash,w);
    }
    CHECK(rfi_pos && rfi_neg && rfq_pos && rfq_neg && rf_hash);
    printf("64 host-defined complex IQ words signed12; packet and corruption checks verified; "
           "RF-bounded +/-64 low-gain QPSK with independent 1024-word validation\n");
    return 0;
}
