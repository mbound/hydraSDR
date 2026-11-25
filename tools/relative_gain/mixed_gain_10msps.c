/* Relative RX gain qualification, Fenvi MT7961 0e8d:7961.
 * RX only, volatile register gain controls, no RF transmission/eFuse writes.
 * The statistics are raw dBFS, never antenna-input dBm.
 */
#define _POSIX_C_SOURCE 200809L
#include "hydrasdr.h"
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <string.h>

#define BINS 320
#define STAGES 9
#define CHUNK 256
static const struct hydrasdr_gain sweep[STAGES]={
{3,4,0}, {1,8,0}, {3,4,0}, {3,0,8}, {3,4,0}, {2,6,4}, {3,4,0}, {5,0,0}, {3,4,0}
};
struct stat {
    uint64_t bins[BINS], blocks, samples, clip, max_component;
};
struct monitor{
    hydrasdr_dev_t *dev;
    pthread_t thread;
    atomic_int active;
    atomic_uint_fast64_t total;
    int stream_rc;
    struct stat data[STAGES];
};
static void sleep_ms(unsigned ms){
    struct timespec t={ms/1000,(long)(ms%1000)*1000000L};
    nanosleep(&t,NULL);
}
static void received(const int16_t*iq,uint32_t count,uint64_t first,void*ptr){
    (void)first; struct monitor *m=ptr;
    atomic_fetch_add_explicit(&m->total,count,memory_order_relaxed);
    int stage=atomic_load_explicit(&m->active,memory_order_relaxed);
    if(stage<0 || stage>=STAGES) return;
    struct stat *st=&m->data[stage];
    for(unsigned i=0;i<count;){
        uint32_t n=count-i;
        if(n>CHUNK)n=CHUNK;
        uint64_t sum=0,clipped=0,maximum=0;
        for(unsigned j=0;j<n;j++){
           int32_t x=iq[2*(i+j)],y=iq[2*(i+j)+1];
           uint32_t ax=(uint32_t)(x<0?-x:x);
           uint32_t ay=(uint32_t)(y<0?-y:y);
           if(ax>=8180 || ay>=8180) clipped++;
           if(ax>maximum)maximum=ax;
           if(ay>maximum)maximum=ay;
           sum+=(uint64_t)(x*x)+(uint64_t)(y*y);
        }
        double p=(double)sum/(2.0*8192.0*8192.0*n);
        double db=10.0*log10(p+1e-24);
        int bin=(int)floor((db+130)*2);
        if(bin<0) bin=0;
        if(bin>=BINS)bin=BINS-1;
        st->bins[bin]++; st->blocks++;st->samples+=n;st->clip+=clipped;
        if(maximum>st->max_component)st->max_component=maximum;
        i+=n;
    }
}
static void*reader(void*ptr){
    struct monitor*m=ptr;
    m->stream_rc=hydrasdr_read_async(m->dev,received,m,0,0);
    return NULL;
}
static double quantile(struct stat*s,double q) {
    if(!s->blocks)return -999.0;
    uint64_t target=(uint64_t)ceil(q*s->blocks);
    uint64_t acc=0;
    for(int i=0;i<BINS;i++){
       acc+=s->bins[i];if(acc>=target) return -130.0+i*0.5;
    }
    return 29.5;
}
int main(void){
    struct hydrasdr_device_info info={0};
    int id=hydrasdr_get_device_info(0,&info);
    if(id || info.vid!=0x0e8d || info.pid!=0x7961){
      fprintf(stderr,"require exact MT7961 0e8d:7961 USB target, no test performed\n");return 2;
    }
    struct monitor m={.active=-1,.stream_rc=-999};atomic_init(&m.active,-1);
    atomic_init(&m.total,0);
    int rc=hydrasdr_open(&m.dev,0);
    if(rc){fprintf(stderr,"OPEN=%d\n",rc);return 3;}
    if(!(rc=hydrasdr_set_sample_rate(m.dev,10000000)))
        rc=hydrasdr_set_center_freq(m.dev,2437000000ULL);
    if(!rc)rc=hydrasdr_set_rx_mode(m.dev,HYDRASDR_RX_SINGLE0);
    if(!rc)rc=hydrasdr_set_tuner_gain_mode(m.dev,1);
    if(!rc)rc=hydrasdr_set_gain_codes(m.dev,&sweep[0]);
    if(rc){fprintf(stderr,"STAGING=%d\n",rc);hydrasdr_close(m.dev);return 4;}
    if(pthread_create(&m.thread,NULL,reader,&m)){hydrasdr_close(m.dev);return 5;}
    /* Wait until firmware is running and the controls are scheduled. */
    for(unsigned s=0;s<STAGES;s++){
        atomic_store(&m.active,-1);
        int gain_rc=HYDRASDR_ERROR_BUSY;
        for(unsigned attempt=0;attempt<35;attempt++){
            gain_rc=hydrasdr_set_gain_codes(m.dev,&sweep[s]);
            if(gain_rc!=HYDRASDR_ERROR_BUSY)break;
            sleep_ms(100);
        }
        if(gain_rc){
            fprintf(stderr,"SET_GAIN stage=%u code=%u/%u/%u rc=%d\n",s,
                     sweep[s].lna,sweep[s].lpf,sweep[s].digital,gain_rc);
            rc=gain_rc;break;
        }
        sleep_ms(150); /* Allow analog settling and queued USB samples to drain. */
        atomic_store(&m.active,(int)s);
        sleep_ms(900);
        atomic_store(&m.active,-1);
        fprintf(stderr,"STAGE_COMPLETE %u code=%u/%u/%u samples=%llu\n",s,
                sweep[s].lna,sweep[s].lpf,sweep[s].digital,
                (unsigned long long)m.data[s].samples);
    }
    int cancelled=hydrasdr_cancel_async(m.dev);
    pthread_join(m.thread,NULL);
    struct hydrasdr_stream_stats st={0};
    int stats_rc=hydrasdr_get_stream_stats(m.dev,&st);
    int closed=hydrasdr_close(m.dev);
    printf("HEADER mode=rx1 rate=10000000 center_hz=2437000000 reference=raw_dBFS no_absolute_dBm\n");
    for(unsigned i=0;i<STAGES;i++) {
        const struct stat *p=&m.data[i];
        printf("GAIN_STAGE %u LNA=%u LPF=%u DIG=%u BLOCKS=%llu SAMPLES=%llu P50=%.2f P95=%.2f P99=%.2f P999=%.2f P9999=%.2f CLIPPED=%llu MAX_ABS=%llu\n",
          i,sweep[i].lna,sweep[i].lpf,sweep[i].digital,
          (unsigned long long)p->blocks,(unsigned long long)p->samples,
          quantile(&m.data[i],.5),quantile(&m.data[i],.95),
          quantile(&m.data[i],.99),quantile(&m.data[i],.999),
          quantile(&m.data[i],.9999),
          (unsigned long long)p->clip,(unsigned long long)p->max_component);
    }
    printf("SUMMARY rc=%d stream=%d cancel=%d close=%d stats=%d gaps_unknown reset=%d dma=%u total_samples=%llu\n",
           rc,m.stream_rc,cancelled,closed,stats_rc,st.reset_error,st.dma_errors,
           (unsigned long long)atomic_load(&m.total));
    return (rc || m.stream_rc || cancelled || closed || stats_rc || st.reset_error ||
            st.dma_errors)?6:0;
}
