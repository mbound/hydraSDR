/* SPDX-License-Identifier: MIT
 * Experimental Fenvi MT7961 dual-RX finite recorder. The HydraSDR
 * backend must emit alternating complex samples RX0(t), RX1(t)....
 * Native CS16, shared LO. RX0 and RX1 individually contiguous in output.
 * No eFuse, flash, kernel module or physical RF modification.
 */
#define _POSIX_C_SOURCE 200809L
#include "hydrasdr.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

static int write_le(FILE *f, const int16_t *buf, uint32_t raw_words, unsigned lane)
{
    unsigned char out[8192];
    for (uint32_t n=lane; n<raw_words; ) {
        size_t used=0;
        while (n<raw_words && used+4<=sizeof(out)) {
            const uint16_t i=(uint16_t)buf[n*2], q=(uint16_t)buf[n*2+1];
            out[used++]=i&255;out[used++]=i>>8;
            out[used++]=q&255;out[used++]=q>>8;
            n+=2;
        }
        if (fwrite(out,1,used,f)!=used) return -1;
    }
    return 0;
}

static int parse_u64(const char*s,uint64_t *v){char *e;errno=0;unsigned long long z=strtoull(s,&e,10);if(errno||e==s||*e||!z)return -1;*v=z;return 0;}
int main(int argc,char **argv)
{
    if(argc!=6){fprintf(stderr,"Usage: %s center_hz rate_per_rx (40000000) complex_samples_per_rx rx0.cs16 rx1.cs16\n",argv[0]);return 2;}
    uint64_t hz,rate,perrx;
    if(parse_u64(argv[1],&hz)||parse_u64(argv[2],&rate)||parse_u64(argv[3],&perrx)||
       rate!=40000000||perrx%1024||perrx>4000000||!strcmp(argv[4],argv[5]))return 2;
    /* This experimental build's known-good LO recipe is qualified at 2.4GHz. */
    if (hz<2156000000ULL||hz>2610000000ULL){fprintf(stderr,"Dual LO tuning qualified only on 2.4GHz test band; 5GHz independent bank0 LO not yet configured\n");return 2;}
    const uint32_t rawwords=(uint32_t)(2*perrx);
    int16_t *buf=malloc((size_t)rawwords*2*sizeof(int16_t));
    if(!buf)return 2;
    hydrasdr_dev_t *dev=NULL;
    int status=hydrasdr_open(&dev,0), result=1;
    uint32_t received=0;
    struct hydrasdr_stream_stats st={0};
    if(status<0)goto cleanup;
    status=hydrasdr_set_center_freq(dev,hz);
    if(status<0)goto cleanup;
    status=hydrasdr_set_sample_rate(dev,(uint32_t)rate);
    if(status<0)goto cleanup;
    status=hydrasdr_set_rx_mode(dev,HYDRASDR_RX_DUAL);
    if(status<0)goto cleanup;
    if(hydrasdr_get_rx_channel_count(dev)!=2){status=HYDRASDR_ERROR_STATE;goto cleanup;}
    status=hydrasdr_read_sync(dev,buf,rawwords,&received);
    hydrasdr_get_stream_stats(dev,&st);
    fprintf(stderr,"dual-RX read: status=%s words=%u/%u fwstate=%u dma=%u reset=%s\n",
      hydrasdr_strerror(status),received,rawwords,st.firmware_state,st.dma_errors,hydrasdr_strerror(st.reset_error));
    if(status<0||received!=rawwords||st.firmware_state!=2||st.dma_errors||st.reset_error){if(!status)status=HYDRASDR_ERROR_STATE;goto cleanup;}
    for(uint32_t n=0;n<rawwords;n++) {
       int16_t i=buf[2*n],q=buf[2*n+1];
       if(i<-8192||i>8191||q<-8192||q>8191){fprintf(stderr,"invalid I/Q at raw word %u\n",n);status=HYDRASDR_ERROR_PROTOCOL;goto cleanup;}
    }
    FILE *a=fopen(argv[4],"wbx");
    if(!a){perror("rx0");status=HYDRASDR_ERROR_IO;goto cleanup;}
    FILE *b=fopen(argv[5],"wbx");
    if(!b){perror("rx1");fclose(a);status=HYDRASDR_ERROR_IO;goto cleanup;}
    int wr=write_le(a,buf,rawwords,0)|write_le(b,buf,rawwords,1);
    int ca=fclose(a), cb=fclose(b);
    if(ca||cb||wr){fprintf(stderr,"file I/O error\n");status=HYDRASDR_ERROR_IO;goto cleanup;}
    printf("PASS dual RX: %llu complex samples per channel, %llu Msps nominal per channel; CS16 files written\n",(unsigned long long)perrx,(unsigned long long)(rate/1000000));
    result=0;
cleanup:
    if(status<0)fprintf(stderr,"FAIL %s\n",hydrasdr_strerror(status));
    if(dev){int c=hydrasdr_close(dev);if(c<0){fprintf(stderr,"close failed: %s\n",hydrasdr_strerror(c));result=1;}}
    free(buf);
    return result;
}
