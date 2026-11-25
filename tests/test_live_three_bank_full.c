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
static int one_block(struct context *c,uint32_t offset,struct block *out){
    memset(out,0,sizeof(*out));
    pthread_mutex_lock(&c->dev->control_mutex);
    uint32_t seq=++c->dev->diag_sequence;
    if(!seq)seq=++c->dev->diag_sequence;
    int rc=0;
    if(mmio_write(c,DIAG_ARG,offset)||mmio_write(c,DIAG_CMD,5)||
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
#define N_EFUSE_BLOCKS 192u
#define PER_BANK_BLOCKS 64u
#define BANK_COUNT 3u
#define EFUSE_BYTES (N_EFUSE_BLOCKS*16u)
static int save_private(const uint8_t *data,const uint8_t *status){
    const char *base="/workspace/tmp/fenvi-efuse-rom-three-banks-20261105.bin";
    const char *sidecar="/workspace/tmp/fenvi-efuse-rom-three-banks-20261105.status.bin";
    if(access(base,F_OK)==0 || access(sidecar,F_OK)==0)return -1;
    int fd=open(base,O_CREAT|O_EXCL|O_WRONLY|O_CLOEXEC,0600);
    if(fd<0)return -1;
    ssize_t n=write(fd,data,EFUSE_BYTES);
    int ok=(n==EFUSE_BYTES && fsync(fd)==0);
    if(close(fd))ok=0;
    if(!ok){unlink(base);return -1;}
    fd=open(sidecar,O_CREAT|O_EXCL|O_WRONLY|O_CLOEXEC,0600);
    if(fd<0){unlink(base);return -1;}
    n=write(fd,status,N_EFUSE_BLOCKS*4u);
    ok=(n==N_EFUSE_BLOCKS*4u && fsync(fd)==0);
    if(close(fd))ok=0;
    if(!ok){unlink(base);unlink(sidecar);return -1;}
    return 0;
}
static void flatten(const struct block *record,uint8_t *bytes,uint8_t *status){
    for(unsigned i=0;i<4;i++){
        uint32_t value=record->words[i];
        bytes[i*4]=(uint8_t)value;
        bytes[i*4+1]=(uint8_t)(value>>8);
        bytes[i*4+2]=(uint8_t)(value>>16);
        bytes[i*4+3]=(uint8_t)(value>>24);
    }
    uint32_t v=record->valid;
    for(unsigned i=0;i<4;i++)status[i]=(uint8_t)(v>>(i*8));
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
    uint8_t data[2][EFUSE_BYTES]={0};
    uint8_t statuses[2][N_EFUSE_BLOCKS*4]={0};
    unsigned valid_count[2][BANK_COUNT]={{0}},invalid_count[2][BANK_COUNT]={{0}};
    int query_rc=0,attempted=0,readback_match=0,prior_bank0_match=0;
    uint64_t time_ms=0;
    if(abi_rc==0 && abi==HYDRASDR_DIAG_ABI_MAGIC &&
       caps_rc==0 && caps==HYDRASDR_DIAG_CAP_STATS && gaps_rc==0){
        uint64_t t=now_ms();
        for(unsigned pass=0;pass<2 && query_rc==0;pass++){
            for(unsigned bank=0;bank<BANK_COUNT && query_rc==0;bank++){
                for(unsigned i=0;i<PER_BANK_BLOCKS;i++){
                    struct block b={0};
                    uint32_t arg=(bank<<16)|(i*16u);
                    int ret=one_block(&c,arg,&b);
                    attempted++;
                    if(ret){query_rc=ret;printf("READ_ERROR PASS=%u BANK=%u OFFSET=0x%04x RC=%d ROM_STATUS=%u ROM_RC=%u\n",
                                       pass,bank,i*16u,ret,b.status,b.rom_rc);break;}
                    size_t idx=(bank*PER_BANK_BLOCKS+i);
                    flatten(&b,data[pass]+idx*16u,statuses[pass]+idx*4u);
                    if(b.valid==1)valid_count[pass][bank]++;
                    else if(b.valid==0)invalid_count[pass][bank]++;
                    else{query_rc=-10;break;}
                }
                printf("BANK_PROGRESS PASS=%u BANK=%u VALID=%u INVALID=%u\n",
                      pass+1,bank,valid_count[pass][bank],invalid_count[pass][bank]);
            }
        }
        time_ms=now_ms()-t;
        if(!query_rc){
            readback_match=!memcmp(data[0],data[1],EFUSE_BYTES) &&
                           !memcmp(statuses[0],statuses[1],sizeof(statuses[0]));
            int prior=open("/workspace/tmp/fenvi-factory-efuse-bank0-20261105.bin",O_RDONLY|O_CLOEXEC);
            if(prior>=0){
                uint8_t old[1024];
                prior_bank0_match=(read(prior,old,sizeof(old))==(ssize_t)sizeof(old) &&
                                   !memcmp(old,data[0],sizeof(old)));
                close(prior);
            }
        }
    }else query_rc=-11;
    int post_rc=hydrasdr_custom_diag_query(dev,HYDRASDR_DIAG_FW_STAT,1,&gaps_after);
    nap_us(200000);
    int cancel=hydrasdr_cancel_async(dev);
    pthread_join(worker,NULL);
    uint64_t samples=atomic_load(&c.samples);
    int close_rc=hydrasdr_close(dev);
    int saved=0;
    if(!query_rc && readback_match && prior_bank0_match && valid_count[0][0]>0)
        saved=save_private(data[0],statuses[0])==0;
    printf("THREE_BANK_EFUSE_FULL MODE=rx1 RATE=10000000 ABI_RC=%d CAPS=%08x "
           "QUERY_RC=%d ATTEMPTED=%d VALID_B0=%u VALID_B1=%u VALID_B2=%u "
           "REPEAT_EQUAL=%d BANK0_PRIOR_EQUAL=%d TIME_MS=%llu "
           "GAPS_BEFORE=%u GAPS_AFTER=%u POST_RC=%d "
           "SAMPLES=%llu STREAM_RC=%d CANCEL_RC=%d CLOSE_RC=%d SAVED_PRIVATELY=%d\n",
           abi_rc,caps,query_rc,attempted,valid_count[0][0],valid_count[0][1],valid_count[0][2],
           readback_match,prior_bank0_match,(unsigned long long)time_ms,
           gaps_before,gaps_after,post_rc,(unsigned long long)samples,
           c.result,cancel,close_rc,saved);
    if(abi_rc || query_rc || !readback_match || !prior_bank0_match ||
       !saved || post_rc || c.result || cancel || close_rc || samples<10000)return 6;
    return 0;
}
