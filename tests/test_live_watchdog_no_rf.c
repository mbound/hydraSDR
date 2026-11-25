/* SPDX-License-Identifier: MIT
 * Explicit hardware-only test of the N9 TX watchdog ARM/DISARM handshake.
 * NEVER writes any TX PHY, PA, TSSI, IQ or tone-generator register.
 * Not registered with CTest; use only after reviewing device state.
 */
#define _POSIX_C_SOURCE 200809L
#include "internal.h"
#include "hydra_tx_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

struct context { hydrasdr_dev_t *dev; atomic_uint_fast64_t words; int stream_rc; };
static void samples_cb(const int16_t *iq,uint32_t n,uint64_t offset,void *arg)
{
    (void)offset;
    struct context *c=arg;
    if(n && iq)atomic_fetch_add(&c->words,n);
}
static void *receiver(void *arg)
{
    struct context *c=arg;
    c->stream_rc=hydrasdr_read_async(c->dev,samples_cb,c,0,0);
    return NULL;
}
static void sleep_ms(int ms)
{
    struct timespec ts={ms/1000,(ms%1000)*1000000L};
    nanosleep(&ts,NULL);
}
int main(int argc,char **argv)
{
    bool expire=argc==2 && !strcmp(argv[1],"expire-no-tx");
    if(argc>2 || (argc==2 && !expire))return 2;
    if(!getenv("HYDRA_TEST_NO_RF_ARM") ||
       strcmp(getenv("HYDRA_TEST_NO_RF_ARM"),"1"))return 2;
    struct hydrasdr_device_info info;
    if(hydrasdr_get_device_info(0,&info) || info.vid!=0x0e8d || info.pid!=0x7961)return 3;
    hydrasdr_dev_t *dev=NULL;
    int rc=hydrasdr_open(&dev,0);
    if(rc)return 4;
    rc=hydrasdr_set_sample_rate(dev,10000000);
    if(!rc)rc=hydrasdr_set_center_freq(dev,UINT64_C(2437000000));
    if(!rc)rc=hydrasdr_set_rx_mode(dev,HYDRASDR_RX_SINGLE0);
    struct context c={.dev=dev,.stream_rc=-1};
    atomic_init(&c.words,0);
    pthread_t thread;bool started=false;
    if(!rc) {
        if(pthread_create(&thread,NULL,receiver,&c))rc=HYDRASDR_ERROR_IO;
        else started=true;
    }
    uint32_t state=UINT32_MAX,exp=UINT32_MAX;
    if(started) {
        int ready=HYDRASDR_ERROR_BUSY;
        for(int k=0;k<100;k++) {
            ready=hydra_tx_watchdog_status(dev,&state,&exp);
            if(!ready)break;
            if(ready!=HYDRASDR_ERROR_BUSY)break;
            sleep_ms(20);
        }
        if(!rc)rc=ready;
    }
    uint32_t val=0,after=UINT32_MAX;
    uint32_t go_before=0,path_before=0,go_after=0,path_after=0;
    if(started && !rc) {
        pthread_mutex_lock(&dev->control_mutex);
        rc=bs_read32(dev,BS_MMIO,0x8308fc00u,&go_before);
        if(!rc)rc=bs_read32(dev,BS_MMIO,0x8308fc20u,&path_before);
        if(!rc && (go_before&0x40000000u || path_before))rc=HYDRASDR_ERROR_STATE;
        /* Never set TX GO. In expiration mode, verify autonomous timeout
         * after releasing the mutex and waiting for the N9 device clock. */
        if(!rc)rc=mt_diag_transact_locked(dev,5,expire?50000u:500000u,&val);
        if(!rc)rc=mt_diag_transact_locked(dev,7,0,&val);
        if(!rc && val!=1)rc=HYDRASDR_ERROR_STATE;
        if(!rc && !expire)rc=mt_diag_transact_locked(dev,6,0,&val);
        if(!rc && !expire)rc=mt_diag_transact_locked(dev,7,0,&after);
        pthread_mutex_unlock(&dev->control_mutex);
        if(expire && !rc) {
            sleep_ms(140);
            uint32_t expiry_count=0;
            rc=hydra_tx_watchdog_status(dev,&after,&expiry_count);
            if(!rc && (after!=2 || expiry_count!=1))rc=HYDRASDR_ERROR_STATE;
            exp=expiry_count;
        }
        if(!rc) {
            pthread_mutex_lock(&dev->control_mutex);
            rc=bs_read32(dev,BS_MMIO,0x8308fc00u,&go_after);
            if(!rc)rc=bs_read32(dev,BS_MMIO,0x8308fc20u,&path_after);
            pthread_mutex_unlock(&dev->control_mutex);
        }
    }
    if(!rc && ((!expire && (after!=0 || path_before!=path_after)) ||
       (expire && (after!=2 || path_after!=0)) ||
       go_before!=go_after))rc=HYDRASDR_ERROR_STATE;
    if(started) {
        sleep_ms(80);
        int cancel=hydrasdr_cancel_async(dev);
        pthread_join(thread,NULL);
        if(!rc)rc=cancel;
        if(!rc)rc=c.stream_rc;
    }
    uint64_t delivered=atomic_load(&c.words);
    int close_rc=hydrasdr_close(dev);
    if(!rc)rc=close_rc;
    if(!rc && delivered<100000)rc=HYDRASDR_ERROR_STATE;
    printf("NO_RF_TX_WATCHDOG mode=%s rc=%d initial=%u exp=%u final=%u go=%08x/%08x path=%08x/%08x samples=%llu stream=%d close=%d\n",
           expire?"expiry":"arm_disarm",
           rc,state,exp,after,go_before,go_after,path_before,path_after,
           (unsigned long long)delivered,c.stream_rc,close_rc);
    return rc?1:0;
}
