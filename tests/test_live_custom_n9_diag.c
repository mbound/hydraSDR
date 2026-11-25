/* SPDX-License-Identifier: MIT
 * Physical Fenvi firmware-diagnostic smoke test, no IQ files and no OTP writes.
 */
#define _POSIX_C_SOURCE 200809L
#include "hydrasdr.h"
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
struct state{
    hydrasdr_dev_t *dev;
    atomic_uint_fast64_t samples;
    int reader_result;
};
static void receive_cb(const int16_t *iq,uint32_t count,uint64_t first,void *ctx){
    (void)first;
    struct state *s=ctx;
    if(count && iq) atomic_fetch_add(&s->samples,count);
}
static void *run_stream(void *ctx){
    struct state *s=ctx;
    s->reader_result=hydrasdr_read_async(s->dev,receive_cb,s,0,0);
    return NULL;
}
static void msleep(long ms){
    struct timespec t={ms/1000,(ms%1000)*1000000};
    nanosleep(&t,NULL);
}
int main(int argc,char **argv){
    if(argc!=2 || (strcmp(argv[1],"rx1") && strcmp(argv[1],"rx2")))return 2;
    int dual=!strcmp(argv[1],"rx2");
    hydrasdr_dev_t *dev=NULL;
    int rc=hydrasdr_open(&dev,0);
    if(rc){fprintf(stderr,"open=%d %s\n",rc,hydrasdr_strerror(rc));return 3;}
    if((rc=hydrasdr_set_sample_rate(dev,40000000)) ||
       (rc=hydrasdr_set_rx_mode(dev,dual?HYDRASDR_RX_DUAL:HYDRASDR_RX_SINGLE0)) ||
       (rc=hydrasdr_set_center_freq(dev,2437000000ULL))){
        fprintf(stderr,"configure=%d %s\n",rc,hydrasdr_strerror(rc));
        hydrasdr_close(dev);return 4;
    }
    struct state s={.dev=dev,.reader_result=-500};
    atomic_init(&s.samples,0);
    pthread_t reader;
    if(pthread_create(&reader,NULL,run_stream,&s)){hydrasdr_close(dev);return 5;}
    uint32_t abi=0,caps=0,packets=0,gaps=0;
    int abi_rc=HYDRASDR_ERROR_BUSY;
    for(int attempts=0;attempts<40;attempts++){
        abi_rc=hydrasdr_custom_diag_query(dev,HYDRASDR_DIAG_FW_ABI,0,&abi);
        if(!abi_rc)break;
        msleep(100);
    }
    int caps_rc=abi_rc?abi_rc:hydrasdr_custom_diag_query(dev,HYDRASDR_DIAG_FW_CAPS,0,&caps);
    int pkt_rc=abi_rc?abi_rc:hydrasdr_custom_diag_query(dev,HYDRASDR_DIAG_FW_STAT,0,&packets);
    int gap_rc=abi_rc?abi_rc:hydrasdr_custom_diag_query(dev,HYDRASDR_DIAG_FW_STAT,1,&gaps);
    int efuse_rc=abi_rc?abi_rc:hydrasdr_custom_diag_query(dev,HYDRASDR_DIAG_FW_EFUSE,0,&packets);
    msleep(1000);
    int cancel=hydrasdr_cancel_async(dev);
    pthread_join(reader,NULL);
    uint64_t observed=atomic_load(&s.samples);
    int close=hydrasdr_close(dev);
    printf("MODE=%s ABI_RC=%d ABI=%08x CAPS_RC=%d CAPS=%08x STATS_RC=%d GAP_RC=%d "
           "GAPS=%u EFUSE_RC=%d SAMPLES=%llu STREAM_RC=%d CANCEL_RC=%d CLOSE_RC=%d\n",
           argv[1],abi_rc,abi,caps_rc,caps,pkt_rc,gap_rc,gaps,efuse_rc,
           (unsigned long long)observed,s.reader_result,cancel,close);
    if(abi_rc || abi!=HYDRASDR_DIAG_ABI_MAGIC || caps_rc ||
       !(caps&HYDRASDR_DIAG_CAP_STATS) || (caps&HYDRASDR_DIAG_CAP_EFUSE) ||
       pkt_rc || gap_rc || efuse_rc!=HYDRASDR_ERROR_UNSUPPORTED ||
       observed<100000)return 6;
    return 0;
}
