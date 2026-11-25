/* SPDX-License-Identifier: MIT
 * Offline, no-USB CPU cost benchmark for the existing 64-IQ/348-byte
 * HydraSDR B3 MediaTek HIF OUT frame. Does NOT test USB, DMA or RF.
 *
 * Build from the HydraSDR directory:
 *   cc -O2 -std=c11 -Wall -Wextra -Itests tests/bench_host_iq_frame.c -o /tmp/bench_host_iq_frame
 * Run: /tmp/bench_host_iq_frame [frames] (default 1000000).
 */
#define _POSIX_C_SOURCE 200809L
#include "ingress_iq_payload.h"
#include <errno.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define FRAME_BYTES 348
static volatile uint32_t consumed_checksum;
static uint64_t ns_now(void)
{
    struct timespec t;
    if (clock_gettime(CLOCK_MONOTONIC, &t)) { perror("clock_gettime"); exit(1); }
    return (uint64_t)t.tv_sec * UINT64_C(1000000000) + (uint64_t)t.tv_nsec;
}
static inline void put16(uint8_t *p, uint16_t v)
{ p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
static inline void put32(uint8_t *p, uint32_t v)
{ for(unsigned n=0;n<4;n++) p[n]=(uint8_t)(v>>(8*n)); }

int main(int argc, char **argv)
{
    uint64_t count=UINT64_C(1000000);
    if(argc>2) { fprintf(stderr,"usage: %s [frames]\n",argv[0]);return 2; }
    if(argc==2) {
        char *end;
        errno=0;
        unsigned long long parsed=strtoull(argv[1],&end,10);
        if(errno || end==argv[1] || *end || !parsed || parsed>UINT64_C(100000000))return 2;
        count=(uint64_t)parsed;
    }
    uint8_t frame[FRAME_BYTES];
    uint32_t nonce=UINT32_C(0x945a56b1);
    uint32_t digest_sink=0;
    const uint64_t t0=ns_now();
    for(uint64_t packet=0;packet<count;packet++) {
        memset(frame,0,sizeof(frame));
        put32(frame,340);
        put32(frame+4,UINT32_C(0x41000000)|340);
        put32(frame+8,UINT32_C(0x80010000));
        put16(frame+36,308);
        put16(frame+38,UINT16_C(0x8000));
        frame[40]=0;frame[41]=0xa0;frame[42]=3;frame[43]=1;
        put32(frame+68,UINT32_C(0x48595131));
        put32(frame+72,nonce);
        put32(frame+76,(uint32_t)packet);
        put32(frame+80,(uint32_t)(packet>>32));
        uint32_t digest=HYDRA_INGRESS_IQ_CHECKSUM_SEED;
        for(unsigned i=0;i<64;i++) {
            uint32_t word=hydra_ingress_iq_word(packet*64u+i,packet,nonce);
            digest=hydra_ingress_iq_checksum_step(digest,word);
            put32(frame+84+4*i,word);
        }
        put32(frame+84+64*4,digest);
        /* Check the actual generated payload so the compiler cannot delete
         * the frame construction merely because there is no USB transfer. */
        for(unsigned k=0;k<sizeof(frame);k++)
            digest_sink=(digest_sink^frame[k])*UINT32_C(16777619);
        digest_sink ^= digest;
    }
    const uint64_t dt=ns_now()-t0;
    consumed_checksum=digest_sink;
    printf("HOST_ONLY_IQ_FRAMES frames=%" PRIu64 " words=%" PRIu64
           " elapsed_ns=%" PRIu64 " frame_rate=%.3f word_rate=%.3f"
           " required_5msps_frame_rate=78125 checksum=0x%08x NO_USB_NO_DMA_NO_RF=1\n",
           count,count*UINT64_C(64),dt,(double)count*1e9/(double)dt,
           (double)count*64e9/(double)dt,(unsigned)consumed_checksum);
    return 0;
}
