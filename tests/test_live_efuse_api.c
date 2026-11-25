/* SPDX-License-Identifier: MIT
 * Physical MT7921AUN read-only eFuse API + native coherent 2RX smoke test.
 * No private OTP bytes printed. No stock FW, USB replug or OTP programming.
 */
#define _POSIX_C_SOURCE 200809L
#include "hydrasdr.h"
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

struct state{hydrasdr_dev_t *dev;atomic_uint_fast64_t samples;int reader_rc;};
static void consume(const int16_t *iq,uint32_t count,uint64_t first,void *ptr){
    (void)first;struct state *s=ptr;
    if(iq&&count)atomic_fetch_add(&s->samples,count);
}
static void *capture(void *ptr){
    struct state *s=ptr;
    s->reader_rc=hydrasdr_read_async(s->dev,consume,s,0,0);
    return NULL;
}
static void sleep_ms(long milliseconds){
    struct timespec t={milliseconds/1000,(milliseconds%1000)*1000000};
    nanosleep(&t,NULL);
}
static int compare_private(unsigned bank,unsigned offset,
                           const struct hydrasdr_efuse_block *b){
    FILE *f=fopen("/workspace/tmp/fenvi-efuse-rom-three-banks-20261105.bin","rb");
    if(!f)return -1;
    if(fseek(f,(long)(bank*1024+offset),SEEK_SET)){fclose(f);return -1;}
    uint8_t expected[16];
    size_t n=fread(expected,1,sizeof(expected),f);
    fclose(f);
    return n==16 && !memcmp(b->data,expected,16);
}
int main(int argc,char **argv){
    if(argc!=2 || strcmp(argv[1],"rx2"))return 2;
    hydrasdr_dev_t *dev=NULL;
    int rc=hydrasdr_open(&dev,0);
    if(rc){fprintf(stderr,"OPEN_RC=%d\n",rc);return 3;}
    rc=hydrasdr_set_sample_rate(dev,40000000);
    if(!rc)rc=hydrasdr_set_rx_mode(dev,HYDRASDR_RX_DUAL);
    if(!rc)rc=hydrasdr_set_center_freq(dev,2437000000ULL);
    if(rc){fprintf(stderr,"CONFIG_RC=%d\n",rc);hydrasdr_close(dev);return 4;}
    struct state s={.dev=dev,.reader_rc=-999};atomic_init(&s.samples,0);
    pthread_t worker;
    if(pthread_create(&worker,NULL,capture,&s)){hydrasdr_close(dev);return 5;}
    uint32_t abi=0,caps=0,gap0=0,gap1=0;
    int a=HYDRASDR_ERROR_BUSY;
    for(unsigned i=0;i<30;i++){
        a=hydrasdr_custom_diag_query(dev,HYDRASDR_DIAG_FW_ABI,0,&abi);
        if(!a)break;
        sleep_ms(100);
    }
    int c=a?a:hydrasdr_custom_diag_query(dev,HYDRASDR_DIAG_FW_CAPS,0,&caps);
    int g=a?a:hydrasdr_custom_diag_query(dev,HYDRASDR_DIAG_FW_STAT,1,&gap0);
    struct hydrasdr_efuse_block b0={0},b2={0},b1={0};
    int r0=-99,r2=-99,r1=-99,compare0=0,compare2=0,compare1=0;
    if(!a && !c && !g && caps==(HYDRASDR_DIAG_CAP_STATS|HYDRASDR_DIAG_CAP_EFUSE)) {
        r0=hydrasdr_custom_efuse_read(dev,0,0,&b0);
        r2=hydrasdr_custom_efuse_read(dev,2,0x140,&b2);
        r1=hydrasdr_custom_efuse_read(dev,1,0,&b1);
        if(!r0)compare0=compare_private(0,0,&b0);
        if(!r2)compare2=compare_private(2,0x140,&b2);
        if(!r1)compare1=compare_private(1,0,&b1);
    }
    int g1=hydrasdr_custom_diag_query(dev,HYDRASDR_DIAG_FW_STAT,1,&gap1);
    sleep_ms(450);
    int cancel=hydrasdr_cancel_async(dev);
    pthread_join(worker,NULL);
    uint64_t samples=atomic_load(&s.samples);
    int close=hydrasdr_close(dev);
    printf("EFUSE_API_2RX MODE=2RX RATE=40000000 ABI_RC=%d ABI=%08x CAPS_RC=%d CAPS=%08x "
           "BANK0_RC=%d VALID0=%u MATCH0=%d BANK2_RC=%d VALID2=%u MATCH2=%d "
           "BANK1_RC=%d VALID1=%u MATCH1=%d "
           "GAPS_BEFORE=%u GAPS_AFTER=%u GAPS_RC=%d "
           "SAMPLES=%llu STREAM_RC=%d CANCEL_RC=%d CLOSE_RC=%d\n",
           a,abi,c,caps,r0,b0.raw_validity,compare0,r2,b2.raw_validity,compare2,
           r1,b1.raw_validity,compare1,gap0,gap1,g1,
           (unsigned long long)samples,s.reader_rc,cancel,close);
    if(a||c||g||g1||r0||r2||r1||!compare0||!compare2||!compare1||
       b0.raw_validity!=1||b2.raw_validity!=1||b1.raw_validity!=0||
       s.reader_rc||cancel||close||samples<100000) return 6;
    return 0;
}
