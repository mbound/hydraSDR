#define main hydra_original_dma_scratch_main
#include "test_live_axi_dma_scratch.c"
#undef main

#define DMA2 UINT32_C(0x80010200)
#define DMA3 UINT32_C(0x80010300)
#define PHY0 UINT32_C(0x8308fc08)
#define PHY1 UINT32_C(0x8309fc08)
#define SRC0 UINT32_C(0x0203e000)
#define SRC1 UINT32_C(0x0203f000)
#define DUAL_WORDS 1024u

static uint32_t stream1_word(unsigned i){
    uint32_t x=UINT32_C(0x6d4a7951) ^ (i*UINT32_C(0x9e3779b9));
    x^=x<<13;x^=x>>17;x^=x<<5;
    uint32_t iv=(x&1u)?32u:0xfe0u, qv=(x&2u)?32u:0xfe0u;
    return iv|(qv<<16);
}
static int stop_dma(hydrasdr_dev_t *d,uint32_t base){
    uint32_t a=1; int rc=read32(d,base+0x08,&a);
    if(!rc && (a&1u)){rc=write32(d,base+0x10,1);if(!rc)rc=write32(d,base+0x14,1);}
    for(unsigned i=0;i<64&&!rc&&(a&1u);i++)rc=read32(d,base+0x08,&a);
    return rc?rc:((a&1u)?HYDRASDR_ERROR_STATE:0);
}
int main(void){
    const char *g=getenv("HYDRA_TEST_DUAL_PHY_DMA"); if(!g||strcmp(g,"1"))return 2;
    struct hydrasdr_device_info info={0}; if(hydrasdr_get_device_info(0,&info)||info.vid!=0x0e8d||info.pid!=0x7961)return 3;
    hydrasdr_dev_t *dev=NULL; int rc=hydrasdr_open(&dev,0); if(rc)return 4;
    struct rx_context ctx={.dev=dev}; atomic_init(&ctx.samples,0); atomic_init(&ctx.phase,3); pthread_t tid; bool streaming=false;
    uint32_t src_saved[2*DUAL_WORDS], latch_saved[2]={0}; bool have_src=false,have_latch=false;
    const uint32_t bases[2]={DMA2,DMA3}, srcs[2]={SRC0,SRC1}, dsts[2]={PHY0,PHY1};
    const uint32_t roff[6]={0x18,0x1c,0x20,0x24,0x28,0x2c}; uint32_t dma_saved[2][6]={{0}};bool have_dma=false;
    unsigned successful=0; uint32_t elapsed[7]={0};
    if(!rc)rc=hydrasdr_set_sample_rate(dev,40000000u);
    if(!rc)rc=hydrasdr_set_center_freq(dev,UINT64_C(2437000000));
    if(!rc)rc=hydrasdr_set_rx_mode(dev,HYDRASDR_RX_DUAL);
    if(!rc){if(pthread_create(&tid,NULL,rx_thread,&ctx))rc=HYDRASDR_ERROR_IO;else streaming=true;} if(streaming)sleep_ms(150);
    pthread_mutex_lock(&dev->control_mutex);
    if(!rc)rc=validate_boot_and_dma_ownership(dev);
    uint32_t caps=0; if(!rc)rc=mt_diag_transact_locked(dev,HYDRASDR_DIAG_FW_CAPS,0,&caps); if(!rc&&!(caps&HYDRASDR_DIAG_CAP_DUAL_PHY_DMA))rc=HYDRASDR_ERROR_UNSUPPORTED;
    /* Same ownership gate used by the validated 4-KiB source tests. */
    uint32_t mode=~0u,fill=~0u,arm=~0u,en=~0u,prod=~0u,cons=~0u,fill2=~0u,prod2=~0u,cons2=~0u;
    if(!rc)rc=read32(dev,MB+292,&mode);
    if(!rc)rc=read32(dev,MB+356,&fill);
    if(!rc)rc=read32(dev,MB+444,&arm);
    if(!rc)rc=read32(dev,MB+240,&en);
    if(!rc)rc=read32(dev,MB+348,&prod);
    if(!rc)rc=read32(dev,MB+352,&cons);
    if(!rc&&(mode||arm||en))rc=HYDRASDR_ERROR_STATE;
    if(!rc)sleep_ms(20);
    if(!rc)rc=read32(dev,MB+356,&fill2);
    if(!rc)rc=read32(dev,MB+348,&prod2);
    if(!rc)rc=read32(dev,MB+352,&cons2);
    if(!rc&&(fill!=fill2||prod!=prod2||cons!=cons2))rc=HYDRASDR_ERROR_STATE;
    for(unsigned i=0;i<2*DUAL_WORDS&&!rc;i++)rc=read32(dev,SRC0+4u*i,&src_saved[i]);
    if(!rc)have_src=true;
    for(unsigned i=0;i<2&&!rc;i++)rc=read32(dev,dsts[i],&latch_saved[i]);
    if(!rc)have_latch=true;
    for(unsigned c=0;c<2&&!rc;c++)for(unsigned i=0;i<6&&!rc;i++)rc=read32(dev,bases[c]+roff[i],&dma_saved[c][i]);
    if(!rc)have_dma=true;
    for(unsigned i=0;i<DUAL_WORDS&&!rc;i++){rc=write32(dev,SRC0+4u*i,long_phy_word(i));if(!rc)rc=write32(dev,SRC1+4u*i,stream1_word(i));}
    for(unsigned c=0;c<2&&!rc;c++){rc=write32(dev,bases[c]+0x1c,srcs[c]);if(!rc)rc=write32(dev,bases[c]+0x20,dsts[c]);if(!rc)rc=write32(dev,bases[c]+0x24,4096);if(!rc)rc=write32(dev,bases[c]+0x28,0);if(!rc)rc=write32(dev,bases[c]+0x2c,0);if(!rc)rc=write32(dev,bases[c]+0x18,DMA_CON_4BYTE|DMA_CON_FIXED_DEST);}
    for(unsigned trial=0;trial<1&&!rc;trial++){
        for(unsigned c=0;c<2&&!rc;c++)rc=write32(dev,bases[c]+0x00,0);
        uint32_t result=0; if(!rc)rc=mt_diag_transact_locked(dev,9u,0,&result);
        bool overlap=!!(result&UINT32_C(0x80000000)); uint32_t us=result&UINT32_C(0x7fffffff); if(!rc&&(!overlap||!us))rc=HYDRASDR_ERROR_STATE;
        uint32_t got0=0,got1=0;if(!rc)rc=read32(dev,PHY0,&got0);if(!rc)rc=read32(dev,PHY1,&got1);
        const uint32_t exp0=long_phy_word(DUAL_WORDS-1), exp1=stream1_word(DUAL_WORDS-1);if(!rc&&(got0!=exp0||got1!=exp1||got0==got1))rc=HYDRASDR_ERROR_STATE;
        printf("DUAL_PHY_DMA trial=%u overlap=%u elapsed_us=%u ch0_last=%08x ch1_last=%08x distinct=%u rc=%d\n",trial,overlap,us,got0,got1,got0!=got1,rc);
        if(!rc){elapsed[successful++]=us;}
    }
    /* Stop before restoration even though successful START should self-clear. */
    for(unsigned c=0;c<2;c++){int x=stop_dma(dev,bases[c]);if(!rc)rc=x;}
    if(have_latch)for(unsigned c=0;c<2;c++){int x=write32(dev,dsts[c],latch_saved[c]);if(!rc)rc=x;}
    if(have_dma)for(unsigned c=0;c<2;c++)for(unsigned i=0;i<6;i++){int x=write32(dev,bases[c]+roff[i],dma_saved[c][i]);if(!rc)rc=x;}
    if(have_src)for(unsigned i=0;i<2*DUAL_WORDS;i++){int x=write32(dev,SRC0+4u*i,src_saved[i]);if(!rc)rc=x;}
    if(have_latch)for(unsigned c=0;c<2&&!rc;c++){uint32_t v=0;rc=read32(dev,dsts[c],&v);if(!rc&&v!=latch_saved[c])rc=HYDRASDR_ERROR_STATE;}
    pthread_mutex_unlock(&dev->control_mutex);
    if(streaming){int x=hydrasdr_cancel_async(dev);pthread_join(tid,NULL);if(!rc)rc=x;if(!rc)rc=ctx.stream_rc;}
    struct hydrasdr_stream_stats st={0};int sr=hydrasdr_get_stream_stats(dev,&st),cl=hydrasdr_close(dev);if(!rc)rc=sr;if(!rc)rc=cl;
    if(successful){for(unsigned i=1;i<successful;i++){uint32_t v=elapsed[i],j=i;while(j&&elapsed[j-1]>v){elapsed[j]=elapsed[j-1];j--;}elapsed[j]=v;}uint32_t med=elapsed[successful/2];printf("DUAL_PHY_DMA_RESULT trials=%u median_us=%u aggregate_Mword_s=%.6f per_stream_floor_Mword_s=%.6f rx_samples=%llu dma_errors=%u reset_error=%d final_rc=%d\n",successful,med,2048.0/(double)med,1024.0/(double)med,(unsigned long long)atomic_load(&ctx.samples),st.dma_errors,st.reset_error,rc);}
    return rc?1:0;
}
