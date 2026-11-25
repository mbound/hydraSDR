/* Read-only Fenvi MT7961 resident ROM eFuse sanity, no OTP writes/TX.
 * Historical reference image must be supplied from protected custody.
 * Results do NOT expose identity or calibration bytes.
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
#include <errno.h>
struct ctx {hydrasdr_dev_t *dev; atomic_ulong samples; int reader_rc;};
static void cb(const int16_t*iq,uint32_t n,uint64_t start,void* ptr) {
    (void)iq;(void)start;
    struct ctx*c=ptr;atomic_fetch_add(&c->samples,n);
}
static void*stream(void*ptr) {
    struct ctx*c=ptr;
    c->reader_rc=hydrasdr_read_async(c->dev,cb,c,0,0);
    return NULL;
}
static void pause_ms(long ms) {
    struct timespec t={.tv_sec=ms/1000,.tv_nsec=(ms%1000)*1000000L};
    nanosleep(&t,NULL);
}
static int compare(const struct hydrasdr_efuse_block *b, FILE *f) {
    uint8_t expected[16]={0};
    long addr=(long)b->bank*1024+b->offset;
    if(fseek(f,addr,SEEK_SET)<0) return -2;
    if(fread(expected,1,16,f)!=16) return -3;
    return memcmp(expected,b->data,16)==0?1:0;
}
int main(int argc,char**argv) {
    if(argc!=3)return 2;
    struct hydrasdr_device_info info={0};
    if(hydrasdr_get_device_info(0,&info)!=0 ||
       info.vid!=0x0e8d || info.pid!=0x7961) {
        fprintf(stderr,"Wrong device: expected exact Fenvi MT7961 identity; no probing performed\n");
        return 3;
    }
    FILE*f=fopen(argv[1],"rb");
    if(!f){fprintf(stderr,"PRIVATE_REFERENCE_OPEN_FAIL errno=%d\n",errno);return 3;}
    if(fseek(f,0,SEEK_END) || ftell(f)!=3072){fclose(f);return 3;}
    rewind(f);
    hydrasdr_dev_t*dev=NULL;
    int rc=hydrasdr_open(&dev,0);
    if(rc){fprintf(stderr,"OPEN_RC=%d\n",rc);fclose(f);return 3;}
    rc=hydrasdr_set_rx_mode(dev,HYDRASDR_RX_SINGLE0);
    if(!rc)rc=hydrasdr_set_sample_rate(dev,10000000);
    if(!rc)rc=hydrasdr_set_center_freq(dev,2437000000ULL);
    if(rc){fprintf(stderr,"CONFIG_RC=%d\n",rc);hydrasdr_close(dev);fclose(f);return 4;}
    struct ctx c={.dev=dev,.reader_rc=-999};
    atomic_init(&c.samples,0);
    pthread_t pt;
    if(pthread_create(&pt,NULL,stream,&c)){hydrasdr_close(dev);fclose(f);return 5;}
    uint32_t abi=0,caps=0;
    int ar=HYDRASDR_ERROR_BUSY;
    for(int i=0;i<35;i++){
        ar=hydrasdr_custom_diag_query(dev,HYDRASDR_DIAG_FW_ABI,0,&abi);
        if(!ar) break;
        pause_ms(80);
    }
    int cr=ar?ar:hydrasdr_custom_diag_query(dev,HYDRASDR_DIAG_FW_CAPS,0,&caps);
    struct hydrasdr_rf_cal_snapshot rf={0};
    int sr=hydrasdr_get_rf_cal_snapshot(dev,&rf);
    uint32_t ff=sr?0:hydrasdr_rf_cal_flags(&rf);

    struct hydrasdr_efuse_block b0={0},b2={0},b1={0};
    int r0=-99,r2=-99,r1=-99,m0=-99,m2=-99,m1=-99;
    if(!ar && !cr && caps==(HYDRASDR_DIAG_CAP_STATS|HYDRASDR_DIAG_CAP_EFUSE)) {
      r0=hydrasdr_custom_efuse_read(dev,0,0,&b0);
      r2=hydrasdr_custom_efuse_read(dev,2,0x140,&b2);
      r1=hydrasdr_custom_efuse_read(dev,1,0,&b1);
      if(!r0)m0=compare(&b0,f);
      if(!r2)m2=compare(&b2,f);
      if(!r1)m1=compare(&b1,f);
    }
    unsigned valid[3]={0}, complete=0, mismatched=0, status_errors=0;
    FILE*vs=fopen(argv[2],"rb");
    uint32_t oldstatus[192]={0};
    int oldstatus_ok=vs && fread(oldstatus,sizeof(uint32_t),192,vs)==192;
    if(vs)fclose(vs);
    if(m0==1 && m1==1 && m2==1 && oldstatus_ok) {
      for(unsigned pass=0;pass<2 && !mismatched && !status_errors;pass++) {
        for(unsigned bank=0;bank<3 && !mismatched && !status_errors;bank++) {
          for(unsigned block=0;block<64;block++) {
            struct hydrasdr_efuse_block b={0};
            int rr=hydrasdr_custom_efuse_read(dev,bank,block*16,&b);
            if(rr){status_errors++;break;}
            int cc=compare(&b,f);
            if(cc!=1){mismatched++;break;}
            if(b.raw_validity!=oldstatus[bank*64+block]){status_errors++;break;}
            if(pass==0)valid[bank]+=b.raw_validity;
            complete++;
          }
        }
      }
    }
    /* Optional source-qualified register-only audit: no additional USB I/O.
     * Read the immutable cached *pre-GO* snapshot, not live/AGC telemetry.
     * No private OTP/eFuse/MAC bytes or per-device identifiers are printed.
     */
    const char*verbose=getenv("HYDRA_CAL_VERBOSE_REGS");
    if(!sr && verbose && !strcmp(verbose,"1")) {
       for(unsigned i=0;i<HYDRASDR_RF_CAL_MMIO_COUNT;i++){
           const struct hydrasdr_rf_diag_reg*name=hydrasdr_rf_cal_mmio_info(i);
           printf("HYDRA_RF_MMIO %s %08x %08x\n",name->name,name->address,rf.mmio[i]);
       }
       for(unsigned port=0;port<2;port++)
           for(unsigned i=0;i<HYDRASDR_RF_CAL_ADIE_COUNT;i++){
               const struct hydrasdr_rf_diag_reg*name=hydrasdr_rf_cal_adie_info(i);
               printf("HYDRA_RF_ADIE %u %s %03x %08x\n",port,name->name,
                      name->address,rf.adie[port][i]);
           }
    }
    uint32_t gaps_before=0,gaps_after=0;
    int gap_before_rc=hydrasdr_custom_diag_query(dev,HYDRASDR_DIAG_FW_STAT,1,&gaps_before);
    pause_ms(90);
    int gap_after_rc=hydrasdr_custom_diag_query(dev,HYDRASDR_DIAG_FW_STAT,1,&gaps_after);
    int stop=hydrasdr_cancel_async(dev);
    pthread_join(pt,NULL);
    int cleanup=hydrasdr_close(dev);
    fclose(f);
    printf("CAL_READONLY_VERIFY ABI_RC=%d ABI_OK=%d CAPS_RC=%d CAPS=%08x B0_RC=%d B0_VALID=%u B0_EQUAL=%d B2_RC=%d B2_VALID=%u B2_EQUAL=%d B1_RC=%d B1_VALID=%u B1_EQUAL=%d SAMPLES=%lu STREAM_RC=%d STOP_RC=%d CLOSE_RC=%d FULL_QUERIES=%u VALID=[%u,%u,%u] MISMATCH=%u STATUS_ERRORS=%u RF_SNAPSHOT_RC=%d RF_SCHEMA=%u RF_PHASE=%u RF_FLAGS=%02x RF_RATE=%u GAP_RC=[%d,%d] GAPS=[%u,%u]\n",
       ar,abi==HYDRASDR_DIAG_ABI_MAGIC,cr,caps,r0,b0.raw_validity,m0,r2,b2.raw_validity,m2,r1,b1.raw_validity,m1,atomic_load(&c.samples),c.reader_rc,stop,cleanup,
       complete,valid[0],valid[1],valid[2],mismatched,status_errors,
       sr,rf.schema_version,rf.phase,ff,rf.sample_rate,gap_before_rc,gap_after_rc,gaps_before,gaps_after);
    return (ar==0 && abi==HYDRASDR_DIAG_ABI_MAGIC && cr==0 && caps==3 &&
            r0==0 && r2==0 && r1==0 && m0==1 && m2==1 && m1==1 &&
            b0.raw_validity==1 && b2.raw_validity==1 && b1.raw_validity==0 &&
            c.reader_rc==0 && stop==0 && cleanup==0 &&
            complete==384 && valid[0]==9 && valid[1]==0 && valid[2]==8 &&
            mismatched==0 && status_errors==0 &&
            gap_before_rc==0 && gap_after_rc==0 &&
            gaps_before==0 && gaps_after==0 &&
            sr==0 && rf.schema_version==HYDRASDR_RF_CAL_SNAPSHOT_VERSION &&
            rf.phase==HYDRASDR_RF_CAL_PHASE_PRE_GO &&
            rf.sample_rate==10000000 && rf.frequency_hz==2437000000ULL)?0:6;
}
