/* SPDX-License-Identifier: MIT
 * Short physical read-only eFuse ROM helper qualification for Fenvi MT7921AUN.
 * Uses same USB-owned device handle and resident research N9 image.
 * Does not invoke ROM OTP programming routine.
 */
#define _POSIX_C_SOURCE 200809L
#include "internal.h"
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>

enum { MB=0x02038360u, DIAG_REQ=112,DIAG_CMD=116,DIAG_ARG=120,
       DIAG_DONE=124,DIAG_STATUS=128,DIAG_RESULT=132,
       OTP_DATA=144,OTP_VALID=160,OTP_ROM_RC=180 };
struct context {hydrasdr_dev_t *dev; atomic_uint_fast64_t samples; int result;};
struct block {uint32_t words[4],valid,rom_rc,status;};
static void cb(const int16_t *iq,uint32_t n,uint64_t first,void *opaque){
    (void)first;struct context *c=opaque;
    if(n && iq)atomic_fetch_add(&c->samples,n);
}
static void *reader(void *opaque){
    struct context *c=opaque;
    c->result=hydrasdr_read_async(c->dev,cb,c,0,0);
    return NULL;
}
static void nap_us(long us){
    struct timespec ts={us/1000000,(us%1000000)*1000};
    nanosleep(&ts,NULL);
}
static uint64_t now_ms(void){
    struct timespec ts;clock_gettime(CLOCK_MONOTONIC,&ts);
    return (uint64_t)ts.tv_sec*1000+ts.tv_nsec/1000000;
}
static int mmio_read(struct context *c,uint32_t off,uint32_t *val){
    return c->dev->io.read32(c->dev->io.context,BS_MMIO,MB+off,val);
}
static int mmio_write(struct context *c,uint32_t off,uint32_t val){
    return c->dev->io.write32(c->dev->io.context,BS_MMIO,MB+off,val);
}
static int one_block(struct context *c,uint32_t arg,struct block *out){
    memset(out,0,sizeof(*out));
    pthread_mutex_lock(&c->dev->control_mutex);
    uint32_t seq=++c->dev->diag_sequence;
    if(!seq)seq=++c->dev->diag_sequence;
    int rc=0;
    if(mmio_write(c,DIAG_ARG,arg)||mmio_write(c,DIAG_CMD,5)||
       mmio_write(c,DIAG_REQ,seq)){rc=-1;goto done;}
    uint64_t end=now_ms()+2000;
    uint32_t completed=0;
    do {
        if(mmio_read(c,DIAG_DONE,&completed)){rc=-2;goto done;}
        if(completed==seq)break;
        nap_us(500);
    }while(now_ms()<end);
    if(completed!=seq){rc=-3;goto done;}
    if(mmio_read(c,DIAG_STATUS,&out->status)||
       mmio_read(c,DIAG_RESULT,&out->valid)||
       mmio_read(c,OTP_ROM_RC,&out->rom_rc)){rc=-4;goto done;}
    for(unsigned i=0;i<4;i++){
        if(mmio_read(c,OTP_DATA+i*4,&out->words[i])){rc=-5;goto done;}
    }
    if(mmio_read(c,DIAG_DONE,&completed) || completed!=seq){rc=-6;goto done;}
    if(out->status!=0 || out->rom_rc!=0)rc=-7;
done:
    pthread_mutex_unlock(&c->dev->control_mutex);
    return rc;
}
int main(int argc,char **argv){
    if(argc!=2 || strcmp(argv[1],"rx1"))return 2;
    hydrasdr_dev_t *dev=NULL;
    int ret=hydrasdr_open(&dev,0);
    if(ret){printf("OPEN_RC=%d\n",ret);return 3;}
    ret=hydrasdr_set_sample_rate(dev,10000000);
    if(!ret)ret=hydrasdr_set_rx_mode(dev,HYDRASDR_RX_SINGLE0);
    if(!ret)ret=hydrasdr_set_center_freq(dev,2437000000ULL);
    if(ret){printf("CONFIG_RC=%d\n",ret);hydrasdr_close(dev);return 4;}
    struct context c={.dev=dev,.result=-1};
    atomic_init(&c.samples,0);
    pthread_t worker;
    if(pthread_create(&worker,NULL,reader,&c)){
        hydrasdr_close(dev);return 5;
    }
    uint32_t abi=0,caps=0,gaps_before=0,gaps_after=0;
    int abi_rc=HYDRASDR_ERROR_BUSY;
    for(int j=0;j<40;j++){
        abi_rc=hydrasdr_custom_diag_query(dev,HYDRASDR_DIAG_FW_ABI,0,&abi);
        if(!abi_rc)break;
        nap_us(100000);
    }
    int caps_rc=abi_rc?abi_rc:
        hydrasdr_custom_diag_query(dev,HYDRASDR_DIAG_FW_CAPS,0,&caps);
    int gaps_rc=abi_rc?abi_rc:
        hydrasdr_custom_diag_query(dev,HYDRASDR_DIAG_FW_STAT,1,&gaps_before);
    struct block measurements[3][2]={0};
    int requests[3][2]={{0}};
    int same[3]={0};
    unsigned supported=0,valid=0;
    if(!abi_rc && abi==HYDRASDR_DIAG_ABI_MAGIC && !caps_rc &&
       caps==HYDRASDR_DIAG_CAP_STATS && !gaps_rc){
        for(unsigned bank=0;bank<3;bank++){
            uint32_t arg=bank<<16;
            requests[bank][0]=one_block(&c,arg,&measurements[bank][0]);
            if(!requests[bank][0])requests[bank][1]=one_block(&c,arg,&measurements[bank][1]);
            same[bank]=!requests[bank][0]&&!requests[bank][1]&&
               !memcmp(&measurements[bank][0],&measurements[bank][1],sizeof(struct block));
            if(same[bank])supported++;
            if(same[bank] && measurements[bank][0].valid==1)valid++;
            printf("ROM_BANK_PROBE BANK=%u ARG=0x%08x RC1=%d RC2=%d VALID=%u ROM_RC=%u STATUS=%u REPEAT=%d\n",
                   bank,arg,requests[bank][0],requests[bank][1],
                   measurements[bank][0].valid,measurements[bank][0].rom_rc,
                   measurements[bank][0].status,same[bank]);
        }
    }
    int post_rc=hydrasdr_custom_diag_query(dev,HYDRASDR_DIAG_FW_STAT,1,&gaps_after);
    nap_us(300000);
    int cancel=hydrasdr_cancel_async(dev);
    pthread_join(worker,NULL);
    uint64_t samples=atomic_load(&c.samples);
    int close_rc=hydrasdr_close(dev);
    printf("THREE_ROM_BANKS MODE=rx1 RATE=10000000 ABI_RC=%d CAPS=%08x BANKS_RESPONDED=%u VALID_FIRST_BLOCKS=%u "
           "GAPS_BEFORE=%u GAPS_AFTER=%u POST_RC=%d SAMPLES=%llu "
           "STREAM_RC=%d CANCEL_RC=%d CLOSE_RC=%d\n",
           abi_rc,caps,supported,valid,gaps_before,gaps_after,post_rc,
           (unsigned long long)samples,c.result,cancel,close_rc);
    if(abi_rc || supported!=3 || post_rc || c.result || cancel || close_rc || samples<10000)return 6;
    return 0;
}
