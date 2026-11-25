#define _POSIX_C_SOURCE 200809L
#include "internal.h"
#include "chips/mt7921.h"
#include <stdio.h>
#include <time.h>
static void cb(const int16_t*a,uint32_t n,uint64_t o,void*x){(void)a;(void)n;(void)o;(void)x;}
static void *thr(void *p){int *rc=((void**)p)[1];*rc=hydrasdr_read_async(((void**)p)[0],cb,NULL,0,0);return NULL;}
int main(void){hydrasdr_dev_t*d=NULL;int rc=hydrasdr_open(&d,0),src=0;if(rc)return 2;rc=hydrasdr_set_sample_rate(d,40000000);if(!rc)rc=hydrasdr_set_center_freq(d,2437000000ULL);if(!rc)rc=hydrasdr_set_rx_mode(d,HYDRASDR_RX_DUAL);pthread_t t;void*args[2]={d,&src};if(!rc&&pthread_create(&t,NULL,thr,args))rc=-1;struct timespec ts={0,250000000};nanosleep(&ts,NULL);const uint32_t regs[]={0x204,0x21c,0x240,0x160,0xc68,0xc6c,0xca8,0xcac,0xc0c,0xd78,0xd7c};pthread_mutex_lock(&d->control_mutex);for(unsigned p=0;p<2&&!rc;p++){for(unsigned i=0;i<sizeof(regs)/sizeof(regs[0]);i++){uint32_t v=0;rc=mt_rf_read(d,p,regs[i],&v);if(!rc)printf("RF_SYNTH_READ port=%u reg=%03x value=%08x\n",p,regs[i],v);}}pthread_mutex_unlock(&d->control_mutex);int c=hydrasdr_cancel_async(d);pthread_join(t,NULL);struct hydrasdr_stream_stats st={0};int sr=hydrasdr_get_stream_stats(d,&st),cl=hydrasdr_close(d);printf("RF_SYNTH_READ_RESULT rc=%d stream=%d cancel=%d stats=%d close=%d dma=%u reset=%d\n",rc,src,c,sr,cl,st.dma_errors,st.reset_error);return rc||src||c||sr||cl?1:0;}
