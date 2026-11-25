/* SPDX-License-Identifier: MIT
 * Bounded native RX measurement. No TX, QA firmware or OTP access.
 * Fixed configuration for the entire run; observations bracket USB transfers,
 * not individual samples. Output only after capture, never inside callback.
 */
#define _POSIX_C_SOURCE 200809L
#include "hydra_measurement.h"
#include "hydra_tx_control.h"
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <math.h>
#include <time.h>
#define WINDOW 8192u
struct row {uint64_t first; struct hydra_iq_result result[2];};
struct context {
    hydrasdr_dev_t *dev; unsigned channels;
    uint64_t target; atomic_uint_fast64_t delivered;
    atomic_bool done; struct hydra_iq_accumulator acc;
    int16_t *capture;
    struct row *rows; size_t used,capacity; int error,stream_rc;
};
static void flush(struct context *c) {
    if(!c->acc.words) return;
    if(c->used==c->capacity) {c->error=HYDRASDR_ERROR_OVERRUN;return;}
    struct row *r=&c->rows[c->used++];r->first=c->acc.next_word-c->acc.words;
    for(unsigned l=0;l<c->channels;l++) {
        int e=hydra_iq_result(&c->acc,l,&r->result[l]);
        if(e)c->error=e;
    }
    hydra_iq_init(&c->acc,c->channels,c->acc.next_word);
}
static void callback(const int16_t *iq,uint32_t n,uint64_t first,void *ptr) {
    struct context *c=ptr; uint64_t delivered=atomic_load(&c->delivered);
    /* Cancellation can drain another already-queued callback. */
    if(delivered>=c->target || c->error)return;
    if(first!=delivered) c->error=HYDRASDR_ERROR_PROTOCOL;
    if(!c->error) {
        uint64_t take=n;
        if(take>c->target-delivered)take=c->target-delivered;
        memcpy(c->capture+2*(size_t)delivered,iq,2*(size_t)take*sizeof(*iq));
        delivered+=take;
    }
    atomic_store(&c->delivered,delivered);
    if(c->error || delivered>=c->target)hydrasdr_cancel_async(c->dev);
}
static void *reader(void *ptr) {
    struct context *c=ptr;
    c->stream_rc=hydrasdr_read_async(c->dev,callback,c,0,0);
    atomic_store(&c->done,1);return NULL;
}
static int number(const char *s,uint64_t *v) {
    char *end;errno=0;if(!s[0] || s[0]=='-' || s[0]=='+')return -1;
    unsigned long long x=strtoull(s,&end,10);
    if(errno || *end)return -1;
    *v=x;return 0;
}
static void pause_ms(long ms) {
    struct timespec t={ms/1000,(ms%1000)*1000000L};
    while(nanosleep(&t,&t) && errno==EINTR){}
}
static void scalar(double x) {if(isfinite(x))printf("%.9g",x);else printf("null");}
static void observation(const struct hydra_frontend_observation *o,uint64_t before,uint64_t after) {
    printf("{\"type\":\"frontend\",\"host_begin_ns\":%llu,\"host_end_ns\":%llu,\"delivered_before\":%llu,\"delivered_after\":%llu,\"sample_alignment\":\"unknown_usb_queue\",\"phase\":%u,\"manual_requested\":%d,\"requested_gain\":[%u,%u,%u],\"mmio\":[",
        (unsigned long long)o->host_begin_ns,(unsigned long long)o->host_end_ns,
        (unsigned long long)before,(unsigned long long)after,o->registers.phase,
        o->manual_requested,o->requested_gain.lna,o->requested_gain.lpf,o->requested_gain.digital);
    for(unsigned i=0;i<HYDRASDR_RF_CAL_MMIO_COUNT;i++)printf("%s%u",i?",":"",o->registers.mmio[i]);
    printf("],\"requested_gain_rx1\":[%u,%u,%u],\"independent_gain\":%d,\"adie\":[",o->requested_gain_pair[1].lna,o->requested_gain_pair[1].lpf,o->requested_gain_pair[1].digital,o->independent_gain_requested);
    for(unsigned port=0;port<2;port++) {
        printf("%s[",port?",":"");
        for(unsigned i=0;i<HYDRASDR_RF_CAL_ADIE_COUNT;i++)printf("%s%u",i?",":"",o->registers.adie[port][i]);
        printf("]");
    }
    printf("]}\n");
}
int main(int argc,char **argv) {
    uint64_t arg[10]={0};
    if(argc!=8 && argc!=11) {fprintf(stderr,"Usage: %s channels rate_hz center_hz samples_per_lane lna lpf digital [rx1_lna rx1_lpf rx1_digital]\n",argv[0]);return 2;}
    for(unsigned i=0;i<(unsigned)argc-1;i++)if(number(argv[i+1],&arg[i]))return 2;
    if((arg[0]!=1 && arg[0]!=2) || !arg[3] || arg[3]>20000000 ||
       arg[4]>5 || arg[5]>8 || arg[6]>12 ||
       (arg[1]!=10000000 && arg[1]!=20000000 && arg[1]!=40000000 && arg[1]!=80000000) ||
       (arg[0]==2 && arg[1]!=40000000))return 2;
    if(argc==11 && (arg[0]!=2 || arg[7]>5 || arg[8]>8 || arg[9]>12))return 2;
    struct hydrasdr_device_info info;
    if(hydrasdr_get_device_info(0,&info) || info.vid!=0x0e8d || info.pid!=0x7961)return 3;
    struct context c={.channels=(unsigned)arg[0],.target=arg[0]*arg[3]};
    c.capacity=(size_t)((c.target+WINDOW-1)/WINDOW);
    c.rows=calloc(c.capacity,sizeof(*c.rows));if(!c.rows)return 3;
    c.capture=calloc((size_t)c.target*2,sizeof(*c.capture));
    if(!c.capture){free(c.rows);return 3;}
    /* Fault pages in before GO, so allocation does not stall the RX consumer. */
    for(size_t k=0;k<(size_t)c.target*2;k+=2048)((volatile int16_t*)c.capture)[k]=0;
    const char *snapshot=getenv("HYDRA_MEASURE_SNAPSHOT");
    int live=snapshot && !strcmp(snapshot,"live");
    if(snapshot && !live && strcmp(snapshot,"pre-go")){free(c.capture);free(c.rows);return 2;}
    if(setenv("HYDRA_RF_CAL_SNAPSHOT","1",1)){free(c.capture);free(c.rows);return 3;}
    atomic_init(&c.delivered,0);atomic_init(&c.done,0);hydra_iq_init(&c.acc,c.channels,0);
    struct hydrasdr_gain gain={(uint8_t)arg[4],(uint8_t)arg[5],(uint8_t)arg[6]};
    int rc=hydrasdr_open(&c.dev,0);
    if(!rc)rc=hydrasdr_set_sample_rate(c.dev,(uint32_t)arg[1]);
    if(!rc)rc=hydrasdr_set_rx_mode(c.dev,c.channels==2?HYDRASDR_RX_DUAL:HYDRASDR_RX_SINGLE0);
    if(!rc)rc=hydrasdr_set_center_freq(c.dev,arg[2]);
    if(!rc)rc=hydrasdr_set_tuner_gain_mode(c.dev,1);
    if(!rc)rc=hydrasdr_set_gain_codes(c.dev,&gain);
    if(!rc && argc==11) {
        struct hydrasdr_gain pair[2]={gain,{(uint8_t)arg[7],(uint8_t)arg[8],(uint8_t)arg[9]}};
        rc=hydra_set_rx_gain_pair(c.dev,pair);
    }
    struct hydra_frontend_observation obs[3];uint64_t before[3],after[3];unsigned observations=0;
    int observation_error=0;
    /* Optional passive N9 watchdog ABI check; never arms or emits TX. */
    const char *wdenv=getenv("HYDRA_TX_WATCHDOG_PROBE");
    int wd_requested=wdenv && !strcmp(wdenv,"1");
    uint32_t wd_state=UINT32_MAX,wd_expirations=UINT32_MAX;
    int wd_rc=HYDRASDR_ERROR_BUSY,wd_attempted=0;
    pthread_t thread;int started=0;
    if(!rc) {if(pthread_create(&thread,NULL,reader,&c))rc=HYDRASDR_ERROR_IO;else started=1;}
    if(started) {
        unsigned ticks=0;
        while(!atomic_load(&c.done) && ticks++<400) {
            if(wd_requested && !wd_attempted && atomic_load(&c.delivered)>0) {
                wd_rc=hydra_tx_watchdog_status(c.dev,&wd_state,&wd_expirations);
                if(wd_rc!=HYDRASDR_ERROR_BUSY)wd_attempted=1;
                if(wd_attempted && (wd_rc || wd_state || wd_expirations)) {
                    observation_error=wd_rc?wd_rc:HYDRASDR_ERROR_STATE;
                    hydrasdr_cancel_async(c.dev);break;
                }
            }
            if(live && observations<3 && atomic_load(&c.delivered)>0) {
                before[observations]=atomic_load(&c.delivered);
                int e=hydra_observe_frontend(c.dev,&obs[observations]);
                after[observations]=atomic_load(&c.delivered);
                if(!e)observations++;
                else if(e!=HYDRASDR_ERROR_BUSY){observation_error=e;hydrasdr_cancel_async(c.dev);break;}
            }
            pause_ms(20);
        }
        if(!atomic_load(&c.done) && !observation_error) {rc=HYDRASDR_ERROR_TIMEOUT;hydrasdr_cancel_async(c.dev);}
        pthread_join(thread,NULL);
    }
    if(started && !live) {
        memset(&obs[0],0,sizeof(obs[0]));
        observation_error=hydrasdr_get_rf_cal_snapshot(c.dev,&obs[0].registers);
        if(!observation_error) {
            int manual;
            hydra_get_rx_gain_pair(c.dev,obs[0].requested_gain_pair,&manual);
            obs[0].requested_gain=obs[0].requested_gain_pair[0];
            obs[0].manual_requested=manual;obs[0].independent_gain_requested=argc==11;
            before[0]=0;after[0]=0;observations=1;
        }
    }
    /* Finite capture is resident first; DSP never stalls the USB callback. */
    uint64_t got=atomic_load(&c.delivered);
    for(uint64_t k=0;k<got && !c.error;) {
        uint32_t take=(uint32_t)((got-k)<WINDOW?(got-k):WINDOW);
        c.error=hydra_iq_update(&c.acc,c.capture+2*(size_t)k,take,k);
        if(!c.error)flush(&c);
        k+=take;
    }
    struct hydrasdr_stream_stats st={0};
    int stats_rc=c.dev?hydrasdr_get_stream_stats(c.dev,&st):HYDRASDR_ERROR_STATE;
    int close_rc=c.dev?hydrasdr_close(c.dev):0;
    if(wd_requested && !wd_attempted && !observation_error)
        observation_error=wd_rc?wd_rc:HYDRASDR_ERROR_STATE;
    int valid=!rc && !c.error && !c.stream_rc && !close_rc && !stats_rc &&
        !st.reset_error && !st.dma_errors && !observation_error && observations &&
        atomic_load(&c.delivered)==c.target;
    printf("{\"type\":\"context\",\"schema\":1,\"channels\":%u,\"sample_rate_per_lane\":%llu,\"frequency_hz\":%llu,\"scale\":\"complex_magnitude_8192_is_0dBFS\",\"reference_bandwidth\":\"whole_captured_discrete_time_band\",\"rf_enbw_hz\":null,\"rcpi\":null,\"rf_dbm\":null,\"sinr_db\":null}\n",c.channels,(unsigned long long)arg[1],(unsigned long long)arg[2]);
    if(wd_requested)
        printf("{\"type\":\"watchdog_readonly\",\"probe_rc\":%d,\"state\":%u,\"expirations\":%u,\"attempted\":%s}\n",wd_rc,wd_state,wd_expirations,wd_attempted?"true":"false");
    for(unsigned i=0;i<observations;i++)observation(&obs[i],before[i],after[i]);
    for(size_t k=0;k<c.used;k++)for(unsigned l=0;l<c.channels;l++) {
        struct hydra_iq_result *r=&c.rows[k].result[l];
        printf("{\"type\":\"iq_window\",\"first_multiplexed_word\":%llu,\"lane\":%u,\"samples\":%llu,\"rail_hits\":%llu,\"mean_i\":%.9g,\"mean_q\":%.9g,\"power_dbfs\":",(unsigned long long)c.rows[k].first,l,(unsigned long long)r->count,(unsigned long long)r->clipped,r->mean_i,r->mean_q);
        scalar(r->power_dbfs);printf(",\"ac_dbfs\":");scalar(r->ac_dbfs);printf(",\"peak_dbfs\":");scalar(r->peak_dbfs);puts("}");
    }
    printf("{\"type\":\"completion\",\"valid_capture\":%s,\"setup_rc\":%d,\"measure_rc\":%d,\"stream_rc\":%d,\"observation_rc\":%d,\"close_rc\":%d,\"stats_rc\":%d,\"reset_rc\":%d,\"dma_errors\":%u,\"words\":%llu,\"observations\":%u}\n",valid?"true":"false",rc,c.error,c.stream_rc,observation_error,close_rc,stats_rc,st.reset_error,st.dma_errors,(unsigned long long)atomic_load(&c.delivered),observations);
    free(c.capture);free(c.rows);return valid?0:1;
}
