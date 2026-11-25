/* SPDX-License-Identifier: MIT
 * MT7961 original-ROM N9 AXI DMA channel register READ-ONLY occupancy probe.
 * NO DMA configuration, NO DMA reset, NO TX/RF enable, NO eFuse writes.
 * Manual opt-in; not CTest. Source-qualified 4 channels base 0x80010000
 * stride 0x100 and registers present in exact-ROM decompilation.
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
static void sleep_ms(long m)
{
    const struct timespec t={.tv_sec=m/1000,.tv_nsec=(m%1000)*1000000L};
    (void)nanosleep(&t,NULL);
}
static const struct reg {
    const char *name;uint32_t off;
} regs[]={
    {"INT_FLAG",0x00u}, {"INT_EN",0x04u}, {"START",0x08u},
    {"CON",0x18u}, {"SRC_candidate",0x1cu},
    {"DST_candidate",0x20u}, {"LEN_candidate",0x24u},
    {"JUMP_candidate",0x2cu}
};
int main(void)
{
    const char *gate=getenv("HYDRA_TEST_AXI_DMA_READONLY");
    if(!gate || strcmp(gate,"1"))return 2;
    struct hydrasdr_device_info info;
    if(hydrasdr_get_device_info(0,&info)||info.vid!=0x0e8d||info.pid!=0x7961)return 3;
    hydrasdr_dev_t *dev=NULL;
    int rc=hydrasdr_open(&dev,0);
    if(rc)return 4;
    if(!rc)rc=hydrasdr_set_sample_rate(dev,40000000u);
    if(!rc)rc=hydrasdr_set_center_freq(dev,UINT64_C(2437000000));
    if(!rc)rc=hydrasdr_set_rx_mode(dev,HYDRASDR_RX_DUAL);
    struct rx_context ctx={.dev=dev,.stream_rc=-1};
    atomic_init(&ctx.samples,0);
    pthread_t tid;
    bool streaming=false;
    if(!rc){
        if(pthread_create(&tid,NULL,rx_thread,&ctx))rc=HYDRASDR_ERROR_IO;
        else streaming=true;
    }
    if(streaming)sleep_ms(140);
    uint32_t values[2][4][sizeof(regs)/sizeof(regs[0])];
    memset(values,0,sizeof(values));
    uint32_t tx_go[2]={0,0},tx_path[2]={0,0};
    /* The immutable N9 firmware startup has ALWAYS saved the actual CPU GP
     * register at mailbox+20 (see iq_stream.S _start). ROM disassembly
     * at 0x81708e performs lwi.gp a2,[+-0x624] BEFORE validating the DMA
     * configuration. This optional two-snapshot probe DOES NOT call it.
     * Its pointer read is allowed ONLY within the known volatile MCU SRAM
     * and under both-chain TX-off gates. No direct GP CPU manipulation. */
    const bool survey_gp=(getenv("HYDRA_TEST_ROM_GP_READONLY")!=NULL &&
        !strcmp(getenv("HYDRA_TEST_ROM_GP_READONLY"),"1"));
    const char *phy_env=getenv("HYDRA_TEST_ROM_PHY_CALLBACKS_READONLY");
    const bool survey_phy=phy_env && !strcmp(phy_env,"1");
    enum { PHY_CB_FIRST=0x117a0, PHY_CB_LAST=0x118b4,
           PHY_CB_COUNT=(PHY_CB_LAST-PHY_CB_FIRST)/4+1 };
    uint32_t phy_callbacks[2][PHY_CB_COUNT]={{0}};
    if(phy_env && (!survey_phy || !survey_gp))rc=HYDRASDR_ERROR_STATE;
    uint32_t gp[2]={0,0},gp_slot_value[2]={0,0},gp_slot_addr[2]={0,0};
    uint32_t gp_magic[2]={0,0};
    /* Original ROM + verified N9 live GP source: nearby GP slots are other
     * DMA status/IRQ function pointers, four IRQ channel callback pointers
     * and three allocator bitmaps. Exact source ROM FUN_ram_00816742 and
     * FUN_ram_0081722e. Read-only; NEVER modify or claim these slots. */
    static const struct gp_slot {
        const char *name;int32_t offset;
    } extra_gp[]={
        {"DMA_STATUS_CALLBACK",-0x62c},{"DMA_MISC_CALLBACK",-0x628},
        {"DMA_IRQ_CALLBACK",-0x620},
        {"DMA_CH0_OWNER_CALLBACK",0x47b8},{"DMA_CH1_OWNER_CALLBACK",0x47bc},
        {"DMA_CH2_OWNER_CALLBACK",0x47c0},{"DMA_CH3_OWNER_CALLBACK",0x47c4},
        {"DMA_ALLOCATOR_TYPE2",0x48b0},{"DMA_ALLOCATOR_TYPE1",0x48b4},
        {"DMA_ALLOCATOR_TYPE0",0x48b8}
    };
    uint32_t owner_values[2][sizeof(extra_gp)/sizeof(extra_gp[0])];
    memset(owner_values,0,sizeof(owner_values));
    for(unsigned snap=0;snap<2 && !rc;snap++){
        pthread_mutex_lock(&dev->control_mutex);
        /* Refuse any reading if transmitter is active. No writes. */
        for(unsigned chain=0;chain<2 && !rc;chain++){
            uint32_t base=UINT32_C(0x8308fc00)+chain*UINT32_C(0x10000);
            rc=bs_read32(dev,BS_MMIO,base,&tx_go[chain]);
            if(!rc)rc=bs_read32(dev,BS_MMIO,base+0x20u,&tx_path[chain]);
            if(!rc && ((tx_go[chain]&UINT32_C(0x40000000))||tx_path[chain]))
                rc=HYDRASDR_ERROR_STATE;
        }
        if(survey_gp && !rc){
            rc=bs_read32(dev,BS_MMIO,UINT32_C(0x02038360),&gp_magic[snap]);
            if(!rc && gp_magic[snap]!=UINT32_C(0x41495242))rc=HYDRASDR_ERROR_STATE;
            if(!rc)rc=bs_read32(dev,BS_MMIO,UINT32_C(0x02038360)+20u,&gp[snap]);
            /* Conservative volatile-SRAM data-only constraint. If GP lies
             * outside verified N9 system SRAM, stop. Never blind-read MMIO
             * or inferred target registers through a computed pointer. */
            if(!rc && (gp[snap]<UINT32_C(0x02000624) ||
                       gp[snap]>=UINT32_C(0x02080000) ||
                       (gp[snap]&3u)!=0))rc=HYDRASDR_ERROR_STATE;
            if(!rc){
                gp_slot_addr[snap]=gp[snap]-UINT32_C(0x624);
                rc=bs_read32(dev,BS_MMIO,gp_slot_addr[snap],&gp_slot_value[snap]);
            }
            for(size_t i=0;i<sizeof(extra_gp)/sizeof(extra_gp[0]) && !rc;i++){
                int64_t address=(int64_t)gp[snap]+extra_gp[i].offset;
                if(address<INT64_C(0x02000000) ||
                   address>=INT64_C(0x02080000) ||
                   (address&3)!=0)rc=HYDRASDR_ERROR_STATE;
                else rc=bs_read32(dev,BS_MMIO,(uint32_t)address,&owner_values[snap][i]);
            }
        }
        if(survey_phy && !rc){
            for(unsigned i=0;i<PHY_CB_COUNT && !rc;i++){
                uint64_t addr=(uint64_t)gp[snap]+PHY_CB_FIRST+4u*i;
                if(addr<UINT64_C(0x02000000) || addr>=UINT64_C(0x02080000) || (addr&3u))
                    rc=HYDRASDR_ERROR_STATE;
                else rc=bs_read32(dev,BS_MMIO,(uint32_t)addr,&phy_callbacks[snap][i]);
            }
        }
        for(unsigned ch=0;ch<4 && !rc;ch++){
            for(size_t i=0;i<sizeof(regs)/sizeof(regs[0]) && !rc;i++){
                uint32_t addr=UINT32_C(0x80010000)+ch*UINT32_C(0x100)+regs[i].off;
                rc=bs_read32(dev,BS_MMIO,addr,&values[snap][ch][i]);
            }
        }
        pthread_mutex_unlock(&dev->control_mutex);
        if(snap==0 && !rc)sleep_ms(150);
    }
    if(survey_gp){
        /* These raw pointers are sensitive and MUST remain in ignored
         * private on-worker diagnostic logs. Never commit this stdout. */
        printf("ROM_GP_READONLY magic0=%08x magic1=%08x gp0=%08x gp1=%08x "
               "slot0=%08x slot1=%08x callback0=%08x callback1=%08x "
               "gp_stable=%u callback_stable=%u rc=%d dma_mmio_writes=0\n",
               gp_magic[0],gp_magic[1],gp[0],gp[1],gp_slot_addr[0],
               gp_slot_addr[1],gp_slot_value[0],gp_slot_value[1],
               gp[0]==gp[1],gp_slot_value[0]==gp_slot_value[1],rc);
        if(!rc && (gp[0]!=gp[1] || gp_slot_value[0]!=gp_slot_value[1]))
            rc=HYDRASDR_ERROR_STATE;
        for(size_t i=0;i<sizeof(extra_gp)/sizeof(extra_gp[0]);i++){
            printf("ROM_GP_EXTRA_READONLY slot=%s first=%08x second=%08x "
                   "stable=%u rf_on=0 dma_mmio_writes=0\n",
                   extra_gp[i].name,owner_values[0][i],owner_values[1][i],
                   owner_values[0][i]==owner_values[1][i]);
            if(!rc && owner_values[0][i]!=owner_values[1][i])
                rc=HYDRASDR_ERROR_STATE;
        }
    }
    if(survey_phy){
        for(unsigned i=0;i<PHY_CB_COUNT;i++){
            printf("ROM_PHY_CALLBACK offset=0x%05x first=%08x second=%08x stable=%u\n",
                   PHY_CB_FIRST+4u*i,phy_callbacks[0][i],phy_callbacks[1][i],
                   phy_callbacks[0][i]==phy_callbacks[1][i]);
            if(!rc && phy_callbacks[0][i]!=phy_callbacks[1][i])rc=HYDRASDR_ERROR_STATE;
        }
    }
    for(unsigned ch=0;ch<4;ch++){
        for(size_t i=0;i<sizeof(regs)/sizeof(regs[0]);i++){
            /* Do not publish SRC/DST candidate raw values (may identify
             * private buffers); only print same-vs-different and bit status.
             * Private raw observations stay on this authorized worker.
             */
            uint32_t x=values[0][ch][i],y=values[1][ch][i];
            if(i==4 || i==5 || i==7){
                printf("AXI_DMA_READONLY ch=%u reg=%s offset=+0x%02x same=%u zero_first=%u zero_second=%u\n",
                       ch,regs[i].name,regs[i].off,x==y,x==0,y==0);
            }else{
                printf("AXI_DMA_READONLY ch=%u reg=%s offset=+0x%02x first=%08x second=%08x same=%u\n",
                       ch,regs[i].name,regs[i].off,x,y,x==y);
            }
        }
    }
    if(streaming){
        int stop=hydrasdr_cancel_async(dev);
        pthread_join(tid,NULL);
        if(!rc)rc=stop;
        if(!rc)rc=ctx.stream_rc;
    }
    uint64_t words=atomic_load(&ctx.samples);
    int close_rc=hydrasdr_close(dev);
    if(!rc)rc=close_rc;
    if(!rc && words<100000)rc=HYDRASDR_ERROR_STATE;
    printf("AXI_DMA_READONLY_RESULT rc=%d mode=2RX40 tx_chains_go_and_path_off=%d dma_mmio_writes=0 "
           "rx_samples=%llu stream_rc=%d close_rc=%d tx0_go=%08x tx0_path=%08x "
           "tx1_go=%08x tx1_path=%08x\n",
           rc,!tx_go[0]&&!tx_path[0]&&!tx_go[1]&&!tx_path[1],
           (unsigned long long)words,ctx.stream_rc,close_rc,
           tx_go[0],tx_path[0],tx_go[1],tx_path[1]);
    return rc?1:0;
}
