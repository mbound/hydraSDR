/* SPDX-License-Identifier: MIT
 * Hardware-independent test for live N9 mailbox protocol and failure gating.
 */
#include "internal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do {if(!(x)){fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#x);exit(1);}}while(0)
enum {MB=0x02038360u};
struct fake {uint32_t words[40];int replies,old_firmware,writes,efuse_enabled;uint64_t clock;};
static uint32_t word(uint32_t a) {CHECK(a>=MB && a<MB+160 && (a-MB)%4==0);return (a-MB)/4;}
static int fake_read(void *ctx,enum bs_space space,uint32_t addr,uint32_t *val){
    struct fake *f=ctx;CHECK(space==BS_MMIO);
    *val=f->words[word(addr)];return 0;
}
static int fake_write(void *ctx,enum bs_space space,uint32_t addr,uint32_t val){
    struct fake *f=ctx;CHECK(space==BS_MMIO);
    f->words[word(addr)]=val;f->writes++;
    if(addr==MB+112 && !f->old_firmware){
        uint32_t cmd=f->words[29],arg=f->words[30];
        uint32_t status=0,result=0;
        switch(cmd){
        case 1:result=HYDRASDR_DIAG_ABI_MAGIC;break;
        case 2:
            if(arg==0)result=772;
            else if(arg==1)result=3;
            else if(arg==2)result=4096;
            else status=3;
            break;
        case 3:
            if(!f->efuse_enabled){status=2;break;}
            if(arg!=((2u<<16)|0x140u)){status=3;break;}
            result=1;
            f->words[36]=0x44332211;
            f->words[37]=0x88776655;
            f->words[38]=0xccbbaa99;
            f->words[39]=0x00ffeedd;
            break;
        case 4:result=HYDRASDR_DIAG_CAP_STATS|
                          (f->efuse_enabled?HYDRASDR_DIAG_CAP_EFUSE:0u);break;
        default:status=2;break;
        }
        f->words[33]=result;
        f->words[32]=status;
        f->words[31]=val;f->replies++;
    }
    return 0;
}
int bs_read32(hydrasdr_dev_t *dev,enum bs_space space,uint32_t address,uint32_t *val){
    return dev->io.read32(dev->io.context,space,address,val);
}
int bs_write32(hydrasdr_dev_t *dev,enum bs_space space,uint32_t address,uint32_t val){
    return dev->io.write32(dev->io.context,space,address,val);
}
uint64_t bs_now_ns(void) {
    static uint64_t tick=0;tick+=UINT64_C(1000000);return tick;
}
void bs_sleep_us(unsigned delay){(void)delay;}
int main(void){
    struct fake f={0};
    hydrasdr_dev_t dev={0};
    CHECK(pthread_mutex_init(&dev.control_mutex,NULL)==0);
    atomic_init(&dev.run_state,BS_RUNNING);
    dev.control_ready=true;
    dev.io.read32=fake_read;
    dev.io.write32=fake_write;
    dev.io.context=&f;
    uint32_t val=0;
    CHECK(hydrasdr_custom_diag_query(&dev,1,0,&val)==0);
    CHECK(val==HYDRASDR_DIAG_ABI_MAGIC && dev.diag_abi_verified);
    CHECK(hydrasdr_custom_diag_query(&dev,4,0,&val)==0);
    CHECK(val==HYDRASDR_DIAG_CAP_STATS);
    CHECK(hydrasdr_custom_diag_query(&dev,2,0,&val)==0 && val==772);
    CHECK(hydrasdr_custom_diag_query(&dev,2,1,&val)==0 && val==3);
    CHECK(hydrasdr_custom_diag_query(&dev,2,3,&val)==HYDRASDR_ERROR_INVALID);
    int before=f.replies;
    CHECK(hydrasdr_custom_diag_query(&dev,3,0,&val)==HYDRASDR_ERROR_UNSUPPORTED);
    CHECK(f.replies==before+1); /* only capabilities, no OTP query transmitted */
    struct hydrasdr_efuse_block otp={0};
    int total=f.replies;
    CHECK(hydrasdr_custom_efuse_read(&dev,2,0x140,&otp)==HYDRASDR_ERROR_UNSUPPORTED);
    CHECK(f.replies==total+1); /* capability only when not supported */
    CHECK(hydrasdr_custom_efuse_read(&dev,3,0,&otp)==HYDRASDR_ERROR_INVALID);
    CHECK(hydrasdr_custom_efuse_read(&dev,0,0x400,&otp)==HYDRASDR_ERROR_INVALID);
    CHECK(hydrasdr_custom_efuse_read(&dev,1,0x131,&otp)==HYDRASDR_ERROR_INVALID);
    f.efuse_enabled=1;
    CHECK(hydrasdr_custom_efuse_read(&dev,2,0x140,&otp)==0);
    CHECK(otp.bank==2 && otp.offset==0x140 && otp.raw_validity==1);
    const uint8_t expected[16]={0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,
                                 0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff,0x00};
    CHECK(!memcmp(otp.data,expected,sizeof(expected)));
    CHECK(hydrasdr_custom_diag_query(&dev,99,0,&val)==HYDRASDR_ERROR_INVALID);
    atomic_store(&dev.run_state,BS_IDLE);
    CHECK(hydrasdr_custom_diag_query(&dev,4,0,&val)==HYDRASDR_ERROR_BUSY);
    atomic_store(&dev.run_state,BS_RUNNING);
    dev.diag_abi_verified=false;
    f.old_firmware=1;
    CHECK(hydrasdr_custom_diag_query(&dev,1,0,&val)==HYDRASDR_ERROR_TIMEOUT);
    CHECK(!dev.diag_abi_verified);
    pthread_mutex_destroy(&dev.control_mutex);
    puts("custom research firmware mailbox: ABI/stats/eFuse 3-bank read and gating PASS");
    return 0;
}
