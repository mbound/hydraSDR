/* SPDX-License-Identifier: MIT
 * Exact-target N9 ROM GPT register audit, READ ONLY and NO RF.
 * This manually gated test must never modify GPT clock, compare, IRQ masks,
 * PHY TX GO, transmitter power or eFuse. There is one USB device owner.
 */
#define _POSIX_C_SOURCE 200809L
#include "internal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

struct rx_context { hydrasdr_dev_t *dev; atomic_uint_fast64_t samples; int stream_rc; };
static void on_rx(const int16_t *iq,uint32_t count,uint64_t off,void *ctx)
{
    (void)off;
    struct rx_context *c=ctx;
    if(iq && count)atomic_fetch_add(&c->samples,count);
}
static void *rx_thread(void *ctx)
{
    struct rx_context *c=ctx;
    c->stream_rc=hydrasdr_read_async(c->dev,on_rx,c,0,0);
    return NULL;
}
static void nap_ms(long millis)
{
    struct timespec tv={.tv_sec=millis/1000,.tv_nsec=(millis%1000)*1000000L};
    (void)nanosleep(&tv,NULL);
}
static const struct gpt_reg {
    const char *name;
    uint32_t addr;
} source_backed_gpt[] = {
    {"GPT_IRQ_STATUS",0x89020000u},
    {"GPT_IRQ_ENABLE",0x89020004u},
    {"GPT0_CTRL",0x89020010u},
    {"GPT0_COMPARE",0x89020014u},
    {"GPT1_CTRL",0x89020020u},
    {"GPT1_COMPARE",0x89020024u},
    {"GPT2_CTRL",0x89020030u},
    {"GPT2_COUNT",0x89020034u},
    {"GPT0_CURRENT",0x89020040u},
    {"GPT1_CURRENT",0x89020044u},
    {"GPT4_CTRL",0x89020060u},
    {"GPT4_COUNT",0x89020068u},
    {"N9_IRQ_PENDING_ACK",0x8901010cu},
    {"N9_IRQ_ENABLE_SET",0x89010110u},
    {"N9_IRQ_ENABLE_STATE",0x89010114u}
};
int main(void)
{
    const char *opt=getenv("HYDRA_TEST_GPT_READONLY");
    if(!opt || strcmp(opt,"1"))return 2;
    struct hydrasdr_device_info info;
    if(hydrasdr_get_device_info(0,&info) || info.vid!=0x0e8d || info.pid!=0x7961)return 3;
    hydrasdr_dev_t *dev=NULL;
    int rc=hydrasdr_open(&dev,0);
    if(rc)return 4;
    if(!rc)rc=hydrasdr_set_sample_rate(dev,40000000u);
    if(!rc)rc=hydrasdr_set_center_freq(dev,UINT64_C(2437000000));
    if(!rc)rc=hydrasdr_set_rx_mode(dev,HYDRASDR_RX_DUAL);
    struct rx_context c={.dev=dev,.stream_rc=-1};
    atomic_init(&c.samples,0);
    pthread_t tid;
    bool running=false;
    if(!rc){
        if(pthread_create(&tid,NULL,rx_thread,&c))rc=HYDRASDR_ERROR_IO;
        else running=true;
    }
    uint32_t snaps[2][sizeof(source_backed_gpt)/sizeof(source_backed_gpt[0])]={{0}};
    uint32_t tx_go=0,tx_path=0;
    if(running)nap_ms(250);
    for(unsigned k=0;k<2 && !rc;k++){
        pthread_mutex_lock(&dev->control_mutex);
        if(k==0){
            rc=bs_read32(dev,BS_MMIO,0x8308fc00u,&tx_go);
            if(!rc)rc=bs_read32(dev,BS_MMIO,0x8308fc20u,&tx_path);
            if(!rc && ((tx_go&0x40000000u) || tx_path))rc=HYDRASDR_ERROR_STATE;
        }
        for(size_t i=0;i<sizeof(source_backed_gpt)/sizeof(source_backed_gpt[0]) && !rc;i++)
            rc=bs_read32(dev,BS_MMIO,source_backed_gpt[i].addr,&snaps[k][i]);
        pthread_mutex_unlock(&dev->control_mutex);
        if(k==0)nap_ms(250);
    }
    /* No assumptions about GPT0/1 availability. Log before assigning them
     * to a future IRQ-driven sample pacer. GPT4 already sources the native
     * 1us counter; only verify that its observation counter advanced. */
    uint32_t timer_delta=snaps[1][11]-snaps[0][11];
    if(!rc && (!timer_delta || timer_delta>UINT32_C(10000000)))rc=HYDRASDR_ERROR_STATE;
    for(size_t i=0;i<sizeof(source_backed_gpt)/sizeof(source_backed_gpt[0]);i++){
        printf("GPT_READONLY reg=%s addr=%08x first=%08x second=%08x delta=%u\n",
               source_backed_gpt[i].name,source_backed_gpt[i].addr,
               snaps[0][i],snaps[1][i],snaps[1][i]-snaps[0][i]);
    }
    if(running){
        int stop=hydrasdr_cancel_async(dev);
        pthread_join(tid,NULL);
        if(!rc)rc=stop;
        if(!rc)rc=c.stream_rc;
    }
    uint64_t words=atomic_load(&c.samples);
    int close_rc=hydrasdr_close(dev);
    if(!rc)rc=close_rc;
    if(!rc && words<100000)rc=HYDRASDR_ERROR_STATE;
    printf("GPT_READONLY_RESULT rc=%d mode=dual_40MSps timer4_delta_us=%u "
           "rx_samples=%llu stream_rc=%d close_rc=%d tx_go=%08x tx_path=%08x\n",
           rc,timer_delta,(unsigned long long)words,c.stream_rc,close_rc,tx_go,tx_path);
    return rc?1:0;
}
