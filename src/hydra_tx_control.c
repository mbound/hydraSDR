/* SPDX-License-Identifier: MIT
 * Experimental host-timed TX generator. Register choreography reconstructed
 * from exact-family 2026 WM/ROM and prior Fenvi tone-burst evidence.
 * No calibration/eFuse/OTP writes; no MT7996 RFINTF +0x250 transplant.
 */
#include "internal.h"
#include "chips/mt7921.h"
#include "hydra_tx_control.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

struct tx_reg { const char *name; uint32_t addr; };
static const struct tx_reg regs[HYDRA_TX_REG_COUNT] = {
    {"tssi0",0x8300c000},{"tssi1",0x8301c000},
    {"mux0",0x830ad4d0},{"mux1",0x830ad4d8},
    {"txpwr0",0x8300c508},{"txpwr1",0x8301c508},
    {"tone0",0x8308fc00},{"tone1",0x8309fc00},
    {"iq0",0x8308fc08},{"iq1",0x8309fc08},
    {"cfg0",0x8308fc0c},{"cfg1",0x8309fc0c},
    {"path0",0x8300d008},{"path1",0x8301d008},
    {"clk00",0x83080004},{"clk01",0x83080008},
    {"clk10",0x83090004},{"clk11",0x83090008},
    {"tonepath0",0x8308fc20},{"tonepath1",0x8309fc20}
};
enum {TX_WD_ARM=5,TX_WD_DISARM=6,TX_WD_STATUS=7,TX_WD_TIMEOUT_US=1000000};
enum {TSSI0,TSSI1,MUX0,MUX1,PWR0,PWR1,TONE0,TONE1,IQ0,IQ1,
      CFG0,CFG1,PATH0,PATH1,CLK00,CLK01,CLK10,CLK11,TONEPATH0,TONEPATH1};
const char *hydra_tx_reg_name(unsigned i) {
    return i<HYDRA_TX_REG_COUNT?regs[i].name:NULL;
}
static int snapshot(hydrasdr_dev_t *dev,uint32_t values[HYDRA_TX_REG_COUNT]) {
    uint32_t tmp[HYDRA_TX_REG_COUNT];
    for(unsigned i=0;i<HYDRA_TX_REG_COUNT;i++)
        BS_TRY(bs_read32(dev,BS_MMIO,regs[i].addr,&tmp[i]));
    memcpy(values,tmp,sizeof(tmp));return 0;
}
static int live(hydrasdr_dev_t *dev) {
    if(!dev || !dev->backend || dev->backend->chipset!=HYDRASDR_CHIP_MT7921)
        return HYDRASDR_ERROR_UNSUPPORTED;
    return atomic_load(&dev->run_state)==BS_RUNNING && dev->control_ready
           ? 0 : HYDRASDR_ERROR_BUSY;
}
int hydra_tx_read_raw(hydrasdr_dev_t *dev,uint32_t values[HYDRA_TX_REG_COUNT]) {
    if(!dev || !values)return HYDRASDR_ERROR_INVALID;
    pthread_mutex_lock(&dev->control_mutex);
    int rc=live(dev);
    if(!rc)rc=snapshot(dev,values);
    pthread_mutex_unlock(&dev->control_mutex);return rc;
}
/* Exact physical Feb-2024 Fenvi N9 code recovered from live RAM:
 * 0x009677de uses ZEB33 on input gain -> uint8, masks existing TX PHY
 * 0x8300/1c508 with 0xff00f000, ORs gain into [7:0] and sets bit31.
 * 0x009677f6 chooses bank(s). Source 2026 helper 0x96951c agrees.
 * Earlier [23:16] "raw power" mapping in Hydra was INCORRECT.
 * C700 path 0x00967828 exact-build semantics remain incomplete. * Independent MT7972 RF 0x768 analog gain is read in RX snapshots.
 */
static const struct tx_reg frontend_regs[HYDRA_TX_FE_MMIO_COUNT]={
    {"tssi_ctl0",0x8300c000},{"tssi_ctl1",0x8301c000},
    {"txpwr0",0x8300c508},{"txpwr1",0x8301c508},
    {"tx_c700_0",0x8300c700},{"tx_c700_1",0x8301c700},
    {"tx_path0",0x8300d008},{"tx_path1",0x8301d008},
    {"tone_go0",0x8308fc00},{"tone_go1",0x8309fc00},
    {"tone_en0",0x8308fc20},{"tone_en1",0x8309fc20},
    {"tone_iq0",0x8308fc08},{"tone_iq1",0x8309fc08},
    {"tx_mux0",0x830ad4d0},{"tx_mux1",0x830ad4d8}
};
const char *hydra_tx_fe_reg_name(unsigned index) {
    return index<HYDRA_TX_FE_MMIO_COUNT?frontend_regs[index].name:NULL;
}
int hydra_tx_fe_observe(hydrasdr_dev_t *dev,struct hydra_tx_fe_status *out) {
    if(!dev || !out)return HYDRASDR_ERROR_INVALID;
    struct hydra_tx_fe_status tmp={0};
    pthread_mutex_lock(&dev->control_mutex);
    int rc=live(dev);
    if(rc)goto done;
    tmp.host_begin_ns=bs_now_ns();
    for(unsigned i=0;i<HYDRA_TX_FE_MMIO_COUNT;i++) {
        rc=bs_read32(dev,BS_MMIO,frontend_regs[i].addr,&tmp.mmio[i]);
        if(rc)goto done;
    }
    for(unsigned i=0;i<2;i++) {
        rc=mt_rf_read(dev,i,0x768,&tmp.adie_gain_raw[i]);
        if(rc)goto done;
    }
    tmp.host_end_ns=bs_now_ns();
    if(!tmp.host_begin_ns || !tmp.host_end_ns || tmp.host_end_ns<tmp.host_begin_ns)
        rc=HYDRASDR_ERROR_STATE;
    else if(atomic_load(&dev->run_state)!=BS_RUNNING || !dev->control_ready)
        rc=HYDRASDR_ERROR_BUSY;
    else *out=tmp;
done:
    pthread_mutex_unlock(&dev->control_mutex);
    return rc;
}
/* Exact-chip RX gain register map; no user-triggered override or RF write. */
static const struct tx_reg rx_fe_regs[HYDRA_RX_FE_MMIO_COUNT]={
    {"phy_lna_lpf0",0x830000c4u}, {"digital_gain0",0x83000108u},
    {"gain_force0",0x83000150u}, {"rx_coarse0",0x830000d0u},
    {"phy_lna_lpf1",0x830100c4u}, {"digital_gain1",0x83010108u},
    {"gain_force1",0x83010150u}, {"rx_coarse1",0x830100d0u}
};
const char *hydra_rx_fe_reg_name(unsigned index){
    return index<HYDRA_RX_FE_MMIO_COUNT?rx_fe_regs[index].name:NULL;
}
int hydra_rx_fe_observe(hydrasdr_dev_t *dev,struct hydra_rx_fe_status *out){
    if(!dev || !out)return HYDRASDR_ERROR_INVALID;
    struct hydra_rx_fe_status tmp={0};
    pthread_mutex_lock(&dev->control_mutex);
    int rc=live(dev);
    if(rc)goto done;
    tmp.host_begin_ns=bs_now_ns();
    for(unsigned i=0;i<HYDRA_RX_FE_MMIO_COUNT;i++){
        rc=bs_read32(dev,BS_MMIO,rx_fe_regs[i].addr,&tmp.phy_mmio[i]);
        if(rc)goto done;
    }
    for(unsigned i=0;i<2;i++){
        rc=mt_rf_read(dev,i,0x768,&tmp.adie_gain_raw[i]);
        if(rc)goto done;
    }
    tmp.host_end_ns=bs_now_ns();
    if(tmp.host_end_ns<tmp.host_begin_ns || !tmp.host_begin_ns)
        rc=HYDRASDR_ERROR_STATE;
    else if(atomic_load(&dev->run_state)!=BS_RUNNING || !dev->control_ready)
        rc=HYDRASDR_ERROR_BUSY;
    else *out=tmp;
done:
    pthread_mutex_unlock(&dev->control_mutex);
    return rc;
}
int hydra_tx_watchdog_status(hydrasdr_dev_t *dev,uint32_t *state,
                              uint32_t *expirations)
{
    if(!dev || !state || !expirations)return HYDRASDR_ERROR_INVALID;
    pthread_mutex_lock(&dev->control_mutex);
    int rc=live(dev);uint32_t result=0,exp=0;
    if(rc)goto done;
    rc=mt_diag_transact_locked(dev,HYDRASDR_DIAG_FW_ABI,0,&result);
    if(rc)goto done;
    if(result!=HYDRASDR_DIAG_ABI_MAGIC){rc=HYDRASDR_ERROR_PROTOCOL;goto done;}
    rc=mt_diag_transact_locked(dev,HYDRASDR_DIAG_FW_CAPS,0,&result);
    if(rc)goto done;
    if(!(result&HYDRASDR_DIAG_CAP_TX_WATCHDOG)) {
        rc=HYDRASDR_ERROR_UNSUPPORTED;goto done;
    }
    rc=mt_diag_transact_locked(dev,TX_WD_STATUS,0,&result);
    if(rc)goto done;
    if(result>2){rc=HYDRASDR_ERROR_PROTOCOL;goto done;}
    rc=bs_read32(dev,BS_MMIO,0x02038360u+196u,&exp);
    if(rc)goto done;
    *state=result;*expirations=exp;
done:
    pthread_mutex_unlock(&dev->control_mutex);
    return rc;
}
/* Readback every non-strobe write. If ignored, fail before generator GO.
 * Hold the existing control mutex throughout every burst and restoration.
 */
static int write_checked(hydrasdr_dev_t*dev,unsigned index,uint32_t value) {
    uint32_t actual=0;
    int rc=bs_write32(dev,BS_MMIO,regs[index].addr,value);
    if(!rc)rc=bs_read32(dev,BS_MMIO,regs[index].addr,&actual);
    /* Physical Fenvi MT7961 readback (2026-11-23): writing CFG0 0x10122
     * immediately reads 0x00120. Bit16 and bit1 are hardware self-clearing;
     * the original exact-chip ROM routine at 0x0083F074 writes 0x10122
     * without requiring these bits to stay latched. Verify all OTHER bits.
     * This exemption is intentionally LIMITED to this one observed register.
     */
    const uint32_t stable_mask=(index==CFG0 || index==CFG1)?~UINT32_C(0x00010002):UINT32_MAX;
    if(!rc && (actual&stable_mask)!=(value&stable_mask))
        rc=HYDRASDR_ERROR_STATE;
    if(rc && getenv("HYDRA_TX_DEBUG") && !strcmp(getenv("HYDRA_TX_DEBUG"),"1"))
        fprintf(stderr,"HYDRA_TX_CHECK_FAIL name=%s addr=0x%08x requested=0x%08x "
                       "readback=0x%08x rc=%d\n",
                regs[index].name,regs[index].addr,value,actual,rc);
    return rc;
}
/* Verified from exact physical Feb-2024 N9 WM function 0x0093e27a.
 * C114 is read as a raw low16 word and corrected in steps of 16 by an
 * application-specific call parameter. We do NOT label it dBm/TSSI here.
 * Read operations only; synchronized to host-control stages not RF samples. */
static int read_candidate_c114(hydrasdr_dev_t *dev,uint32_t words[2]) {
    BS_TRY(bs_read32(dev,BS_MMIO,UINT32_C(0x8300c114),&words[0]));
    return bs_read32(dev,BS_MMIO,UINT32_C(0x8301c114),&words[1]);
}
static int write_iq_bank(hydrasdr_dev_t*dev,unsigned bank,int16_t i,int16_t q) {
    uint32_t iq=((uint32_t)(uint16_t)i & 0xfffu)|
                (((uint32_t)(uint16_t)q & 0xfffu)<<16);
    return write_checked(dev,bank?IQ1:IQ0,iq);
}
static int write_iq(hydrasdr_dev_t*dev,const struct hydra_tx_iq_step*s) {
    return write_iq_bank(dev,0,s->i,s->q);
}
static int start(hydrasdr_dev_t*dev,uint8_t power,const struct hydra_tx_iq_step *s) {
    /* Same order as exact WM tone routine, but keep chain 1 GO and tonepath
     * OFF: no evidence yet warrants emitting on both physical chains. */
    BS_TRY(write_checked(dev,TSSI0,0x00274027));
    BS_TRY(write_checked(dev,TSSI1,0x00274027));
    BS_TRY(write_checked(dev,MUX0,0x111f1f01));
    BS_TRY(write_checked(dev,MUX1,0x101f1f00));
    /* Source: Feb-2024 WM 0x009677de: (old & 0xff00f000) |
     * (uint8_t)gain | 0x80000000. Tone starts with both overrides
     * enabled (0x80008000), so apply the original vendor helper's
     * low-byte field while preserving bit15, NOT the old Hydra [23:16].
     * Gain code is not mapped to physical dBm and cannot be called PA gain. */
    const uint32_t tone_power=UINT32_C(0x80008000);
    const uint32_t vendor_gain_word=(tone_power&UINT32_C(0xff00f000)) |
                                    (uint32_t)power | UINT32_C(0x80000000);
    BS_TRY(write_checked(dev,PWR0,vendor_gain_word));
    BS_TRY(write_checked(dev,PWR1,0x80008000u));
    BS_TRY(write_checked(dev,TONE0,0x80830000));
    BS_TRY(write_iq(dev,s));
    BS_TRY(write_checked(dev,CFG0,0x00010122));
    BS_TRY(write_checked(dev,PATH0,0x1dd0));
    BS_TRY(write_checked(dev,PATH1,0x1d90));
    BS_TRY(write_checked(dev,CLK00,0x00ffffff));
    BS_TRY(write_checked(dev,CLK01,0x00ffffff));
    BS_TRY(write_checked(dev,CLK10,0x00ffffff));
    BS_TRY(write_checked(dev,CLK11,0x00ffffff));
    BS_TRY(write_checked(dev,TONEPATH0,1));
    /* The LOAD pulse has a transient self-clearing side effect and must not
     * be retried; the exact base register is read back before GO. */
    BS_TRY(bs_write32(dev,BS_MMIO,regs[CFG0].addr,0x00010123));
    BS_TRY(bs_write32(dev,BS_MMIO,regs[CFG0].addr,0x00010122));
    return write_checked(dev,TONE0,0xc0830000u);
}
/* Disable GO first, even after a partial failed transaction. Attempt every
 * restoration word and preserve the earliest failure; a subsequent WFSYS
 * reset is required on any failure and never interpreted as verified RF off.
 */
/* Candidate 2T2R mode reconstructed from the exact ROM path bitmap.
 * ROM 0x83f0a6..0x83f0d6 uses bitmap values 1/2/4/8 and 0xf in the high
 * byte of D4D0, then the common tail programs both PHYDFE banks. 0x3 is the
 * minimal composition for paths 0+1; it is a hypothesis until RF-qualified.
 * The rest follows the already validated chain0 mode symmetrically. */
static int start_dual(hydrasdr_dev_t*dev,uint8_t power,
                      const struct hydra_tx_dual_iq_seed *seed) {
    const uint32_t tone_power=UINT32_C(0x80008000);
    const uint32_t vendor_gain_word=(tone_power&UINT32_C(0xff00f000)) |
                                    (uint32_t)power | UINT32_C(0x80000000);
    BS_TRY(write_checked(dev,TSSI0,0x00274027));
    BS_TRY(write_checked(dev,TSSI1,0x00274027));
    BS_TRY(write_checked(dev,MUX0,0x131f1f01));
    BS_TRY(write_checked(dev,MUX1,0x101f1f00));
    BS_TRY(write_checked(dev,PWR0,vendor_gain_word));
    BS_TRY(write_checked(dev,PWR1,vendor_gain_word));
    BS_TRY(write_checked(dev,TONE0,0x80830000));
    BS_TRY(write_checked(dev,TONE1,0x80830000));
    BS_TRY(write_iq_bank(dev,0,seed->i0,seed->q0));
    BS_TRY(write_iq_bank(dev,1,seed->i1,seed->q1));
    BS_TRY(write_checked(dev,CFG0,0x00010122));
    BS_TRY(write_checked(dev,CFG1,0x00010122));
    BS_TRY(write_checked(dev,PATH0,0x1dd0));
    BS_TRY(write_checked(dev,PATH1,0x1dd0));
    BS_TRY(write_checked(dev,CLK00,0x00ffffff));
    BS_TRY(write_checked(dev,CLK01,0x00ffffff));
    BS_TRY(write_checked(dev,CLK10,0x00ffffff));
    BS_TRY(write_checked(dev,CLK11,0x00ffffff));
    BS_TRY(write_checked(dev,TONEPATH0,1));
    BS_TRY(write_checked(dev,TONEPATH1,1));
    /* Exact ROM pulses CTL3 bit0 in both banks. Strobes are never retried. */
    BS_TRY(bs_write32(dev,BS_MMIO,regs[CFG0].addr,0x00010123));
    BS_TRY(bs_write32(dev,BS_MMIO,regs[CFG1].addr,0x00010123));
    BS_TRY(bs_write32(dev,BS_MMIO,regs[CFG0].addr,0x00010122));
    BS_TRY(bs_write32(dev,BS_MMIO,regs[CFG1].addr,0x00010122));
    BS_TRY(write_checked(dev,TONE0,0xc0830000u));
    return write_checked(dev,TONE1,0xc0830000u);
}
static int restore(hydrasdr_dev_t*dev,const uint32_t before[HYDRA_TX_REG_COUNT]) {
    int first=0,rc;
    /* Both banks must be stopped before restoring any frontend state.
     * The preflight admits only idle generators; preserve first error. */
    for(unsigned i=0;i<2;i++){
        uint32_t tone=0;
        rc=bs_read32(dev,BS_MMIO,regs[TONE0+i].addr,&tone);
        if(rc<0){if(!first)first=rc;}
        else {
            rc=bs_write32(dev,BS_MMIO,regs[TONE0+i].addr,tone&~(1u<<30));
            if(rc<0&&!first)first=rc;
        }
    }
    for(unsigned i=0;i<2;i++){
        rc=bs_write32(dev,BS_MMIO,regs[TONEPATH0+i].addr,0);
        if(rc<0&&!first)first=rc;
    }
    /* Every register is then restored, including original control words. */
    static const unsigned order[]={TONEPATH1,TONEPATH0,TONE1,TONE0,IQ1,IQ0,
         CFG1,CFG0,CLK11,CLK10,CLK01,CLK00,PATH1,PATH0,PWR1,PWR0,
         MUX1,MUX0,TSSI1,TSSI0};
    for(unsigned j=0;j<HYDRA_TX_REG_COUNT;j++) {
        unsigned k=order[j];
        rc=write_checked(dev,k,before[k]);
        if(rc<0 && !first)first=rc;
    }
    return first;
}
static void failure(hydrasdr_dev_t*dev,int rc) {
    dev->control_ready=false;
    dev->control_error=rc;
    dev->control_error_stage=HYDRASDR_STAGE_RF;
    unsigned state=BS_RUNNING;
    atomic_compare_exchange_strong(&dev->run_state,&state,BS_CONTROL_FAILED);
    dev->needs_usb_reset=true;
}
static int hydra_tx_run_bounded_iq_internal(hydrasdr_dev_t*dev,
                            const struct hydra_tx_burst_config*cfg,
                            const struct hydra_tx_iq_step*steps,unsigned count,
                            struct hydra_tx_burst_receipt*out,
                            int (*on_tx_started)(hydrasdr_dev_t*,void*),void *on_tx_ctx)
{
    if(!dev || !cfg || !steps || !out || !count || count>64 || cfg->chain!=0)
        return HYDRASDR_ERROR_INVALID;
    uint64_t total=0;
    for(unsigned i=0;i<count;i++) {
        if(steps[i].i < -2048 || steps[i].i>2047 ||
           steps[i].q < -2048 || steps[i].q>2047 ||
           steps[i].dwell_us<100 || steps[i].dwell_us>20000)
            return HYDRASDR_ERROR_INVALID;
        total+=steps[i].dwell_us;
    }
    if(total>100000)return HYDRASDR_ERROR_INVALID;
    const char *opt=getenv("HYDRA_TX_EXPERIMENTAL");
    if(!opt || strcmp(opt,"1"))return HYDRASDR_ERROR_UNSUPPORTED;
    if(cfg->raw_power_code) {
        opt=getenv("HYDRA_TX_ALLOW_UNCALIBRATED_POWER");
        if(!opt || strcmp(opt,"1"))return HYDRASDR_ERROR_UNSUPPORTED;
    }
    int rc=0;struct hydra_tx_burst_receipt receipt={0};
    pthread_mutex_lock(&dev->control_mutex);
    rc=live(dev);
    if(rc)goto done;
    if(atomic_load(&dev->frequency)!=UINT64_C(2437000000)) {
        rc=HYDRASDR_ERROR_UNSUPPORTED;goto done;
    }
    rc=snapshot(dev,receipt.before);
    if(rc)goto done;
    rc=read_candidate_c114(dev,receipt.candidate_c114[0]);
    if(rc)goto done;
    if(getenv("HYDRA_TX_DEBUG") && !strcmp(getenv("HYDRA_TX_DEBUG"),"1")){
        for(unsigned i=0;i<HYDRA_TX_REG_COUNT;i++)
            fprintf(stderr,"HYDRA_TX_BASELINE %s=0x%08x\n",
                    regs[i].name,receipt.before[i]);
    }
    /* Fail closed on any active generator, existing override or preexisting
     * forced chain TX path. Do not intrude on another TX owner. */
    if((receipt.before[TONE0]&0xc0000000u) ||
       (receipt.before[TONE1]&0xc0000000u) ||
       receipt.before[TONEPATH0] || receipt.before[TONEPATH1] ||
       (receipt.before[PWR0]&0x80008000u) ||
       (receipt.before[PWR1]&0x80008000u) ||
       receipt.before[PATH0]!=0x1d10u || receipt.before[PATH1]!=0x1d10u ||
       receipt.before[TSSI0]!=0xc0c80806u || receipt.before[TSSI1]!=0xc0c80806u ||
       receipt.before[MUX0]!=0x01010101u || receipt.before[MUX1]!=0x01010101u) {
        rc=HYDRASDR_ERROR_STATE;goto done;
    }
    /* Mandatory N9-side watchdog capability. Unlike a host deadline it
     * continues checking while the host process/USB control side is stalled.
     * Exact firmware capability is verified BEFORE any TX register is touched.
     * An unacknowledged ARM may still have taken effect; attempt DISARM and
     * force a complete acquisition reset if the protocol is ambiguous. */
    uint32_t result=0;
    rc=mt_diag_transact_locked(dev,HYDRASDR_DIAG_FW_ABI,0,&result);
    if(rc)goto done;
    if(result!=HYDRASDR_DIAG_ABI_MAGIC){rc=HYDRASDR_ERROR_PROTOCOL;goto done;}
    rc=mt_diag_transact_locked(dev,HYDRASDR_DIAG_FW_CAPS,0,&result);
    if(rc)goto done;
    if(!(result&HYDRASDR_DIAG_CAP_TX_WATCHDOG)){
        rc=HYDRASDR_ERROR_UNSUPPORTED;goto done;
    }
    /* Manual bounded DMA mode has a shorter fail-safe. Unmodified host
     * step sequencer retains its existing one-second watchdog. */
    const uint32_t watchdog_us=on_tx_started?50000u:TX_WD_TIMEOUT_US;
    rc=mt_diag_transact_locked(dev,TX_WD_ARM,watchdog_us,&result);
    if(rc) {
        uint32_t ignored=0;
        /* No emission yet, but an unacknowledged ARM must not linger. */
        (void)mt_diag_transact_locked(dev,TX_WD_DISARM,0,&ignored);
        failure(dev,rc);
        goto done;
    }
    /* Starting with a valid baseline makes rollback mandatory even on the
     * first failed MMIO transaction: it may have taken effect before EIO. */
    uint64_t armed_host_ns=bs_now_ns();
    rc=start(dev,cfg->raw_power_code,&steps[0]);
    if(getenv("HYDRA_TX_DEBUG") && !strcmp(getenv("HYDRA_TX_DEBUG"),"1"))
        fprintf(stderr,"HYDRA_TX_START_RESULT rc=%d host_elapsed_us=%llu\n",rc,
                (unsigned long long)((bs_now_ns()-armed_host_ns)/1000));
    if(!rc)rc=read_candidate_c114(dev,receipt.candidate_c114[1]);
    /* Hook runs after RF GO under the SAME sole USB/control owner;
     * must finish and quiesce DMA before the existing TX rollback. */
    if(!rc && on_tx_started)rc=on_tx_started(dev,on_tx_ctx);
    if(!rc) {
        receipt.host_start_ns=bs_now_ns();
        for(unsigned i=0;i<count;i++) {
            if(atomic_load(&dev->run_state)!=BS_RUNNING) {
                rc=HYDRASDR_ERROR_BUSY;break;
            }
            if(i)rc=write_iq(dev,&steps[i]);
            if(rc)break;
            bs_sleep_us(steps[i].dwell_us);
            receipt.completed_steps++;
        }
    }
    int restore_rc=restore(dev,receipt.before);
    receipt.host_stop_ns=bs_now_ns();
    /* Query BEFORE disarming so that an N9 timeout is not silently masked by
     * later successful host restoration. Disarm accepts only actual RF-off. */
    uint32_t wd_state=0;
    int status_rc=mt_diag_transact_locked(dev,TX_WD_STATUS,0,&wd_state);
    int disarm_rc=mt_diag_transact_locked(dev,TX_WD_DISARM,0,&result);
    int read_rc=snapshot(dev,receipt.after);
    int feedback_rc=read_rc?read_rc:read_candidate_c114(
            dev,receipt.candidate_c114[2]);
    if(getenv("HYDRA_TX_DEBUG") && !strcmp(getenv("HYDRA_TX_DEBUG"),"1"))
        fprintf(stderr,"HYDRA_TX_TEARDOWN start_rc=%d steps=%u restore_rc=%d "
                       "watchdog_status_rc=%d watchdog_state=%u "
                       "watchdog_disarm_rc=%d read_rc=%d equal=%d\n",
                rc,receipt.completed_steps,restore_rc,status_rc,wd_state,
                disarm_rc,read_rc,
                read_rc?0:!memcmp(receipt.before,receipt.after,sizeof(receipt.before)));
    if(!rc)rc=restore_rc;
    if(!rc)rc=status_rc;
    if(!rc && wd_state!=1)rc=HYDRASDR_ERROR_STATE;
    if(!rc)rc=disarm_rc;
    if(!rc)rc=read_rc;
    if(!rc)rc=feedback_rc;
    if(!rc && memcmp(receipt.before,receipt.after,sizeof(receipt.before)))
        rc=HYDRASDR_ERROR_STATE;
    /* Ordinary cancellation is not a control failure if the RF state was
     * restored exactly; the RX owner is already handling shutdown. */
    if(rc && !(rc==HYDRASDR_ERROR_BUSY && !restore_rc && !read_rc &&
              !memcmp(receipt.before,receipt.after,sizeof(receipt.before))))
        failure(dev,rc);
    if(!rc)*out=receipt;
done:
    pthread_mutex_unlock(&dev->control_mutex);
    return rc;
}
int hydra_tx_run_bounded_iq(hydrasdr_dev_t *dev,
        const struct hydra_tx_burst_config *cfg,
        const struct hydra_tx_iq_step *steps,unsigned count,
        struct hydra_tx_burst_receipt *receipt)
{
    return hydra_tx_run_bounded_iq_internal(dev,cfg,steps,count,receipt,NULL,NULL);
}
int hydra_tx_run_bounded_iq_with_hook(hydrasdr_dev_t *dev,
        const struct hydra_tx_burst_config *cfg,
        const struct hydra_tx_iq_step *steps,unsigned count,
        struct hydra_tx_burst_receipt *receipt,
        int (*on_tx_started)(hydrasdr_dev_t*,void*),void *ctx)
{
    /* Restricted single-owner short RF experiment only. */
    if(!on_tx_started || !ctx || !cfg || cfg->chain!=0 || cfg->raw_power_code ||
       count!=1 || !steps || steps[0].dwell_us<1000u || steps[0].dwell_us>2500u ||
       !getenv("HYDRA_TEST_AXI_DMA_RF_BURST") ||
       strcmp(getenv("HYDRA_TEST_AXI_DMA_RF_BURST"),"1") ||
       !getenv("HYDRA_RF_SHIELDED") || strcmp(getenv("HYDRA_RF_SHIELDED"),"1") ||
       !getenv("HYDRA_TX_EXPERIMENTAL") || strcmp(getenv("HYDRA_TX_EXPERIMENTAL"),"1") ||
       !getenv("HYDRA_TX_ALLOW_UNCALIBRATED_POWER") ||
       strcmp(getenv("HYDRA_TX_ALLOW_UNCALIBRATED_POWER"),"1"))
       return HYDRASDR_ERROR_UNSUPPORTED;
    return hydra_tx_run_bounded_iq_internal(dev,cfg,steps,count,receipt,on_tx_started,ctx);
}

int hydra_tx_run_bounded_dual_iq_with_hook(hydrasdr_dev_t *dev,
        uint8_t raw_power_code,const struct hydra_tx_dual_iq_seed *seed,
        struct hydra_tx_burst_receipt *out,
        int (*on_tx_started)(hydrasdr_dev_t*,void*),void *ctx)
{
    if(!dev || !seed || !out || !on_tx_started || !ctx || raw_power_code ||
       seed->i0<-2048 || seed->i0>2047 || seed->q0<-2048 || seed->q0>2047 ||
       seed->i1<-2048 || seed->i1>2047 || seed->q1<-2048 || seed->q1>2047 ||
       seed->dwell_us<1000u || seed->dwell_us>2500u)
        return HYDRASDR_ERROR_INVALID;
    const char *g=getenv("HYDRA_TX_DUAL_EXPERIMENTAL");
    if(!g || strcmp(g,"1") ||
       !getenv("HYDRA_TX_EXPERIMENTAL") || strcmp(getenv("HYDRA_TX_EXPERIMENTAL"),"1") ||
       !getenv("HYDRA_RF_SHIELDED") || strcmp(getenv("HYDRA_RF_SHIELDED"),"1") ||
       !getenv("HYDRA_TX_ALLOW_UNCALIBRATED_POWER") || strcmp(getenv("HYDRA_TX_ALLOW_UNCALIBRATED_POWER"),"1") ||
       !getenv("HYDRA_TEST_DUAL_TX_RF_BURST") || strcmp(getenv("HYDRA_TEST_DUAL_TX_RF_BURST"),"1"))
        return HYDRASDR_ERROR_UNSUPPORTED;
    int rc=0; struct hydra_tx_burst_receipt receipt={0};
    pthread_mutex_lock(&dev->control_mutex);
    rc=live(dev); if(rc)goto done;
    if(atomic_load(&dev->frequency)!=UINT64_C(2437000000)) {rc=HYDRASDR_ERROR_UNSUPPORTED;goto done;}
    rc=snapshot(dev,receipt.before); if(rc)goto done;
    if((receipt.before[TONE0]&0xc0000000u) || (receipt.before[TONE1]&0xc0000000u) ||
       receipt.before[TONEPATH0] || receipt.before[TONEPATH1] ||
       (receipt.before[PWR0]&0x80008000u) || (receipt.before[PWR1]&0x80008000u) ||
       receipt.before[PATH0]!=0x1d10u || receipt.before[PATH1]!=0x1d10u ||
       receipt.before[TSSI0]!=0xc0c80806u || receipt.before[TSSI1]!=0xc0c80806u ||
       receipt.before[MUX0]!=0x01010101u || receipt.before[MUX1]!=0x01010101u) {
        rc=HYDRASDR_ERROR_STATE;goto done;
    }
    uint32_t result=0;
    rc=mt_diag_transact_locked(dev,HYDRASDR_DIAG_FW_ABI,0,&result); if(rc)goto done;
    if(result!=HYDRASDR_DIAG_ABI_MAGIC){rc=HYDRASDR_ERROR_PROTOCOL;goto done;}
    rc=mt_diag_transact_locked(dev,HYDRASDR_DIAG_FW_CAPS,0,&result); if(rc)goto done;
    if((result&(HYDRASDR_DIAG_CAP_TX_WATCHDOG|HYDRASDR_DIAG_CAP_TX_WATCHDOG_DUAL_BANK)) !=
       (HYDRASDR_DIAG_CAP_TX_WATCHDOG|HYDRASDR_DIAG_CAP_TX_WATCHDOG_DUAL_BANK)) {
        rc=HYDRASDR_ERROR_UNSUPPORTED;goto done;
    }
    rc=mt_diag_transact_locked(dev,TX_WD_ARM,50000u,&result);
    if(rc){uint32_t ignored=0;(void)mt_diag_transact_locked(dev,TX_WD_DISARM,0,&ignored);failure(dev,rc);goto done;}
    rc=start_dual(dev,raw_power_code,seed);
    if(!rc)rc=on_tx_started(dev,ctx);
    if(!rc){receipt.host_start_ns=bs_now_ns();bs_sleep_us(seed->dwell_us);receipt.completed_steps=1;}
    int restore_rc=restore(dev,receipt.before);receipt.host_stop_ns=bs_now_ns();
    uint32_t wd_state=0;
    int status_rc=mt_diag_transact_locked(dev,TX_WD_STATUS,0,&wd_state);
    int disarm_rc=mt_diag_transact_locked(dev,TX_WD_DISARM,0,&result);
    int read_rc=snapshot(dev,receipt.after);
    if(!rc)rc=restore_rc;
    if(!rc)rc=status_rc;
    if(!rc&&wd_state!=1)rc=HYDRASDR_ERROR_STATE;
    if(!rc)rc=disarm_rc;
    if(!rc)rc=read_rc;
    if(!rc && memcmp(receipt.before,receipt.after,sizeof(receipt.before)))rc=HYDRASDR_ERROR_STATE;
    if(rc && !(rc==HYDRASDR_ERROR_BUSY && !restore_rc && !read_rc &&
              !memcmp(receipt.before,receipt.after,sizeof(receipt.before))))failure(dev,rc);
    if(!rc)*out=receipt;
done:
    pthread_mutex_unlock(&dev->control_mutex);
    return rc;
}

/* Host-to-N9 staged cyclic playback is a FIRST STEP in IQ delivery. The
 * source remains the physically proven PHY IQ setpoint register, not an
 * undocumented DAC DMA interface. All SRAM/MAILBOX addresses are exclusively
 * reserved by the current resident N9 image, not by the original stock WM. */
enum {STAGED_MB=0x02038360u,STAGED_RING=0x0203c000u,
      STAGED_ON=STAGED_MB+200u,STAGED_LEN=STAGED_MB+204u,
      STAGED_PERIOD=STAGED_MB+208u,STAGED_INDEX=STAGED_MB+212u,
      STAGED_COMPLETED=STAGED_MB+216u,STAGED_LAST=STAGED_MB+220u,
      STAGED_MAXDELTA=STAGED_MB+224u,STAGED_OVERRUN=STAGED_MB+228u,
      STAGED_FIRST_EMIT=STAGED_MB+232u,STAGED_LAST_EMIT=STAGED_MB+236u};
static int write_staged_word(hydrasdr_dev_t *dev,uint32_t addr,uint32_t value){
    uint32_t observed=0;
    int rc=bs_write32(dev,BS_MMIO,addr,value);
    if(!rc)rc=bs_read32(dev,BS_MMIO,addr,&observed);
    if(!rc && observed!=value)rc=HYDRASDR_ERROR_STATE;
    return rc;
}
static int staged_preflight(hydrasdr_dev_t *dev,
                            const uint32_t state[HYDRA_TX_REG_COUNT]){
    if((state[TONE0]&0xc0000000u) || (state[TONE1]&0xc0000000u) ||
       state[TONEPATH0] || state[TONEPATH1] ||
       (state[PWR0]&0x80008000u) || (state[PWR1]&0x80008000u) ||
       state[PATH0]!=0x1d10u || state[PATH1]!=0x1d10u ||
       state[TSSI0]!=0xc0c80806u || state[TSSI1]!=0xc0c80806u ||
       state[MUX0]!=0x01010101u || state[MUX1]!=0x01010101u)
        return HYDRASDR_ERROR_STATE;
    uint32_t playback=UINT32_MAX;
    BS_TRY(bs_read32(dev,BS_MMIO,STAGED_ON,&playback));
    if(playback) return HYDRASDR_ERROR_STATE;
    return 0;
}
const char *hydra_tx_active_reg_name(unsigned index) {
    if(index<HYDRA_TX_FE_MMIO_COUNT)return frontend_regs[index].name;
    index-=HYDRA_TX_FE_MMIO_COUNT;
    if(index<HYDRA_RX_FE_MMIO_COUNT)return rx_fe_regs[index].name;
    return index==8?"candidate_c114_0":index==9?"candidate_c114_1":NULL;
}
static int active_snapshot_read(hydrasdr_dev_t *dev,uint32_t base,
                               struct hydra_tx_active_snapshot *out) {
    /* Host STOP plus the subsequent diagnostic ACK fences the N9 service.
     * Still reject absent, torn or stale output rather than assuming it. */
    struct hydra_tx_active_snapshot tmp={0};
    uint32_t end_seq=0;
    BS_TRY(bs_read32(dev,BS_MMIO,base,&tmp.sequence));
    if(!tmp.sequence || (tmp.sequence&1u))return HYDRASDR_ERROR_PROTOCOL;
    BS_TRY(bs_read32(dev,BS_MMIO,base+4,&tmp.begin_us));
    BS_TRY(bs_read32(dev,BS_MMIO,base+8,&tmp.end_us));
    BS_TRY(bs_read32(dev,BS_MMIO,base+12,&tmp.emission));
    for(unsigned j=0;j<HYDRA_TX_ACTIVE_REG_COUNT;j++)
        BS_TRY(bs_read32(dev,BS_MMIO,base+16+4*j,&tmp.words[j]));
    BS_TRY(bs_read32(dev,BS_MMIO,base,&end_seq));
    if(end_seq!=tmp.sequence || tmp.end_us-tmp.begin_us>1000u ||
       !tmp.emission || !(tmp.words[8]&UINT32_C(0x40000000)))
        return HYDRASDR_ERROR_PROTOCOL;
    *out=tmp;return 0;
}
int hydra_tx_run_bounded_staged_iq(hydrasdr_dev_t *dev,
                            const struct hydra_tx_burst_config *cfg,
                            const struct hydra_tx_iq_sample *samples,
                            unsigned sample_count,unsigned period_us,
                            unsigned duration_us,
                            struct hydra_tx_staged_iq_receipt *out,
                            void (*on_tx_phase)(void *ctx,int active),
                            void *phase_ctx)
{
    if(!dev || !cfg || !samples || !out || cfg->chain!=0 ||
       sample_count==0 || sample_count>HYDRA_TX_STAGED_IQ_MAX_SAMPLES ||
       period_us<2 || period_us>1000 || duration_us<1000 ||
       duration_us>80000 || duration_us/period_us<sample_count)
        return HYDRASDR_ERROR_INVALID;
    for(unsigned j=0;j<sample_count;j++)
        if(samples[j].i< -2048 || samples[j].i>2047 ||
           samples[j].q< -2048 || samples[j].q>2047)
            return HYDRASDR_ERROR_INVALID;
    const char *opt=getenv("HYDRA_TX_IQ_STAGED");
    if(!opt || strcmp(opt,"1"))return HYDRASDR_ERROR_UNSUPPORTED;
    opt=getenv("HYDRA_RF_SHIELDED");
    if(!opt || strcmp(opt,"1"))return HYDRASDR_ERROR_UNSUPPORTED;
    opt=getenv("HYDRA_TX_EXPERIMENTAL");
    if(!opt || strcmp(opt,"1"))return HYDRASDR_ERROR_UNSUPPORTED;
    if(cfg->raw_power_code){
        opt=getenv("HYDRA_TX_ALLOW_UNCALIBRATED_POWER");
        if(!opt || strcmp(opt,"1"))return HYDRASDR_ERROR_UNSUPPORTED;
    }
    struct hydra_tx_staged_iq_receipt report={0};
    struct hydra_tx_burst_receipt *tx=&report.tx;
    report.sample_count=sample_count;
    report.period_us=period_us;
    report.duration_us=duration_us;
    uint32_t status=UINT32_MAX,wd=UINT32_MAX,result=0;
    int rc=0;
    pthread_mutex_lock(&dev->control_mutex);
    rc=live(dev);
    if(rc)goto finish;
    if(atomic_load(&dev->frequency)!=UINT64_C(2437000000)) {
        rc=HYDRASDR_ERROR_UNSUPPORTED;goto finish;
    }
    rc=snapshot(dev,tx->before);
    if(rc)goto finish;
    rc=staged_preflight(dev,tx->before);
    if(rc)goto finish;
    rc=mt_diag_transact_locked(dev,HYDRASDR_DIAG_FW_ABI,0,&result);
    if(rc)goto finish;
    if(result!=HYDRASDR_DIAG_ABI_MAGIC){rc=HYDRASDR_ERROR_PROTOCOL;goto finish;}
    rc=mt_diag_transact_locked(dev,HYDRASDR_DIAG_FW_CAPS,0,&result);
    if(rc)goto finish;
    if((result & (HYDRASDR_DIAG_CAP_TX_WATCHDOG | HYDRASDR_DIAG_CAP_STAGED_IQ | HYDRASDR_DIAG_CAP_TX_ACTIVE_SNAPSHOT)) !=
       (HYDRASDR_DIAG_CAP_TX_WATCHDOG | HYDRASDR_DIAG_CAP_STAGED_IQ | HYDRASDR_DIAG_CAP_TX_ACTIVE_SNAPSHOT)){
        rc=HYDRASDR_ERROR_UNSUPPORTED;goto finish;
    }
    rc=mt_diag_transact_locked(dev,TX_WD_STATUS,0,&wd);
    if(rc)goto finish;
    if(wd){rc=HYDRASDR_ERROR_STATE;goto finish;}
    /* With the generator OFF and watchdog DISARMED, prepare an immutable
     * cyclic table in dedicated N9 SRAM. Every word has exact readback. */
    for(unsigned j=0;j<sample_count;j++){
        uint32_t word=((uint32_t)(uint16_t)samples[j].i&0xfffu) |
            (((uint32_t)(uint16_t)samples[j].q&0xfffu)<<16);
        rc=write_staged_word(dev,STAGED_RING+j*4u,word);
        if(rc){failure(dev,rc);goto finish;}
    }
    static const uint32_t reset_fields[]={STAGED_LEN,STAGED_PERIOD,
                   STAGED_INDEX,STAGED_COMPLETED,STAGED_LAST,
                   STAGED_MAXDELTA,STAGED_OVERRUN,
                   STAGED_FIRST_EMIT,STAGED_LAST_EMIT};
    const uint32_t reset_values[]={sample_count,period_us,0,0,0,0,0,0,0};
    for(unsigned j=0;j<sizeof(reset_fields)/sizeof(reset_fields[0]);j++){
        rc=write_staged_word(dev,reset_fields[j],reset_values[j]);
        if(rc){failure(dev,rc);goto finish;}
    }
    rc=write_staged_word(dev,0x0203c400u,0);
    if(!rc)rc=write_staged_word(dev,0x0203c480u,0);
    if(!rc)rc=read_candidate_c114(dev,tx->candidate_c114[0]);
    if(rc){failure(dev,rc);goto finish;}
    /* Same acknowledged device-side 1s watchdog as host-timed TX. The
     * playback service will not touch the PHY unless it sees ARMED=1. */
    rc=mt_diag_transact_locked(dev,TX_WD_ARM,TX_WD_TIMEOUT_US,&result);
    if(rc){
        uint32_t ignored=0;
        (void)mt_diag_transact_locked(dev,TX_WD_DISARM,0,&ignored);
        failure(dev,rc);goto finish;
    }
    const struct hydra_tx_iq_step first={samples[0].i,samples[0].q,100};
    rc=start(dev,cfg->raw_power_code,&first);
    if(!rc)rc=read_candidate_c114(dev,tx->candidate_c114[1]);
    if(!rc){
        uint32_t tick=0;
        rc=bs_read32(dev,BS_MMIO,UINT32_C(0x89020068),&tick);
        if(!rc)rc=write_staged_word(dev,STAGED_LAST,tick);
    }
    if(!rc)rc=write_staged_word(dev,STAGED_ON,1);
    if(!rc){
        tx->host_start_ns=bs_now_ns();
        if(on_tx_phase)on_tx_phase(phase_ctx,1);
        bs_sleep_us(duration_us);
        tx->completed_steps=1; /* one bounded host session; N9 counts I/Q */
    }
    /* Always disable autonomous playback BEFORE restoring any TX PHY words.
     * A failed control write is not ignored; the watchdog remains armed and
     * subsequent restoration must still be attempted. */
    int stop_rc=write_staged_word(dev,STAGED_ON,0);
    if(on_tx_phase && tx->host_start_ns)on_tx_phase(phase_ctx,0);
    int telemetry_rc=bs_read32(dev,BS_MMIO,STAGED_COMPLETED,&report.completed_updates);
    if(!telemetry_rc)telemetry_rc=bs_read32(dev,BS_MMIO,STAGED_OVERRUN,&report.overruns);
    if(!telemetry_rc)telemetry_rc=bs_read32(dev,BS_MMIO,STAGED_MAXDELTA,&report.max_update_delta_us);
    if(!telemetry_rc)telemetry_rc=bs_read32(dev,BS_MMIO,STAGED_INDEX,&report.final_index);
    if(!telemetry_rc)telemetry_rc=bs_read32(dev,BS_MMIO,STAGED_ON,&report.iq_control_after);
    if(!telemetry_rc)telemetry_rc=bs_read32(dev,BS_MMIO,STAGED_FIRST_EMIT,&report.n9_first_emit_us);
    if(!telemetry_rc)telemetry_rc=bs_read32(dev,BS_MMIO,STAGED_LAST_EMIT,&report.n9_last_emit_us);
    report.n9_span_us=report.n9_last_emit_us-report.n9_first_emit_us;
    int restore_rc=restore(dev,tx->before);
    tx->host_stop_ns=bs_now_ns();
    int status_rc=mt_diag_transact_locked(dev,TX_WD_STATUS,0,&status);
    int disarm_rc=mt_diag_transact_locked(dev,TX_WD_DISARM,0,&result);
    int snapshot_rc=snapshot(dev,tx->after);
    int feedback_rc=snapshot_rc?snapshot_rc:read_candidate_c114(dev,tx->candidate_c114[2]);
    /* Read the buffered MMIO only AFTER RF rollback and N9 diagnostic ACK,
     * never prolong TX by performing dozens of USB control transactions. */
    int active_rc=stop_rc?stop_rc:status_rc;
    if(!active_rc)active_rc=active_snapshot_read(dev,0x0203c400u,&report.active[0]);
    if(!active_rc && report.completed_updates>=256)
        active_rc=active_snapshot_read(dev,0x0203c480u,&report.active[1]);
    if(!active_rc && (report.active[0].emission!=1 ||
       report.active[1].emission>report.completed_updates ||
       report.active[1].emission%256u))active_rc=HYDRASDR_ERROR_PROTOCOL;
    if(!rc)rc=stop_rc;
    if(!rc)rc=telemetry_rc;
    if(!rc)rc=active_rc;
    if(!rc && (report.completed_updates<2 || !report.n9_span_us ||
               report.n9_span_us>duration_us*2u ||
               report.final_index>=sample_count || report.iq_control_after))
        rc=HYDRASDR_ERROR_STATE;
    if(!rc)rc=restore_rc;
    if(!rc)rc=status_rc;
    if(!rc && status!=1)rc=HYDRASDR_ERROR_STATE;
    if(!rc)rc=disarm_rc;
    if(!rc)rc=snapshot_rc;
    if(!rc)rc=feedback_rc;
    if(!rc && memcmp(tx->before,tx->after,sizeof(tx->before)))
        rc=HYDRASDR_ERROR_STATE;
    if(rc)failure(dev,rc);
    else *out=report;
finish:
    pthread_mutex_unlock(&dev->control_mutex);
    return rc;
}

/* R2 manual USB-host-fed IQ bridge; NOT a hardware-paced 5MS/s DAC path.
 * Caller holds dev->control_mutex (same single USB owner). Prior to entry:
 *  - native dual RX is already capturing, four RX1 hardware DMA slots owned
 *    by dedicated HIF ingress, mode3 and exactly 3+ checksum-verified 64word
 *    frames staged in the separate 16slot software FIFO, RF arm=0;
 *  - no emitter, PA, generator, cyclic table or other PHY writer is active;
 *  - send_live_frames exclusively uses bulk OUT without control writes.
 * This R2 callback may return an error but must never block indefinitely.
 * Host control deadline is 20ms; N9 autonomous watchdog is 100ms in case
 * the host callback or USB gets stuck. These are finite scalar PHY setpoint
 * stores, not a proven TX sample-clock FIFO. No new PHY/AXI-DMA registers.
 */
int hydra_tx_run_bounded_usb_iq_locked(hydrasdr_dev_t *dev,
          const struct hydra_tx_burst_config *cfg,
          unsigned expected_samples,unsigned duration_us,
          int (*send_live_frames)(void *ctx),void *send_ctx,
          struct hydra_tx_usb_ingress_receipt *out,
          void (*on_tx_phase)(void *ctx,int active),void *phase_ctx)
{
    enum {MB=0x02038360u, RF_ARM=444u,RF_WRITES=448u,RF_LAST=452u,
          RF_FIRST=456u,RF_FINAL=460u,CONSUMER_STATE=300u,
          CONSUMED_LO=260u,CONSUMER_UNDERRUN=312u,FIFO_FILL=356u,
          FIFO_HIGHWATER=360u,STAGE_PACKETS=368u,ABORT_REQ=408u,
          ABORT_ACK=412u};
    if(!dev || !cfg || !out || !send_live_frames || cfg->chain!=0 ||
       cfg->raw_power_code!=0 || duration_us!=20000u ||
       expected_samples<192u || expected_samples>2048u ||
       (expected_samples%64u) ||
       !getenv("HYDRA_TEST_USB_INGRESS_RF_LIVE") ||
       strcmp(getenv("HYDRA_TEST_USB_INGRESS_RF_LIVE"),"1") ||
       !getenv("HYDRA_TX_EXPERIMENTAL") ||
       strcmp(getenv("HYDRA_TX_EXPERIMENTAL"),"1") ||
       !getenv("HYDRA_RF_SHIELDED") || strcmp(getenv("HYDRA_RF_SHIELDED"),"1") ||
       !getenv("HYDRA_TX_ALLOW_UNCALIBRATED_POWER") ||
       strcmp(getenv("HYDRA_TX_ALLOW_UNCALIBRATED_POWER"),"1"))
        return HYDRASDR_ERROR_UNSUPPORTED;
    struct hydra_tx_usb_ingress_receipt report={0};
    struct hydra_tx_burst_receipt *tx=&report.tx;
    uint32_t result=0,wd=UINT32_MAX,go=UINT32_MAX;
    bool armed=false,may_have_started=false,phase_active=false;
    int rc=live(dev);
    if(rc)return rc;
    if(atomic_load(&dev->frequency)!=UINT64_C(2437000000))
        return HYDRASDR_ERROR_UNSUPPORTED;
    rc=snapshot(dev,tx->before);
    if(rc)return rc;
    rc=staged_preflight(dev,tx->before);
    if(rc)return rc;
    rc=mt_diag_transact_locked(dev,HYDRASDR_DIAG_FW_ABI,0,&result);
    if(rc)return rc;
    if(result!=HYDRASDR_DIAG_ABI_MAGIC)return HYDRASDR_ERROR_PROTOCOL;
    rc=mt_diag_transact_locked(dev,HYDRASDR_DIAG_FW_CAPS,0,&result);
    if(rc)return rc;
    const uint32_t needed=HYDRASDR_DIAG_CAP_TX_WATCHDOG|
                         HYDRASDR_DIAG_CAP_USB_INGRESS_AUDIT|
                         HYDRASDR_DIAG_CAP_USB_INGRESS_PACED;
    if((result&needed)!=needed)return HYDRASDR_ERROR_UNSUPPORTED;
    rc=mt_diag_transact_locked(dev,TX_WD_STATUS,0,&wd);
    if(rc || wd)return rc?rc:HYDRASDR_ERROR_STATE;
    uint32_t mode=UINT32_MAX,enable=UINT32_MAX,state=UINT32_MAX;
    uint32_t fill=0,producer=0,arm=UINT32_MAX,completed=UINT32_MAX;
    rc=bs_read32(dev,BS_MMIO,MB+292u,&mode);
    if(!rc)rc=bs_read32(dev,BS_MMIO,MB+240u,&enable);
    if(!rc)rc=bs_read32(dev,BS_MMIO,MB+CONSUMER_STATE,&state);
    if(!rc)rc=bs_read32(dev,BS_MMIO,MB+FIFO_FILL,&fill);
    if(!rc)rc=bs_read32(dev,BS_MMIO,MB+STAGE_PACKETS,&producer);
    if(!rc)rc=bs_read32(dev,BS_MMIO,MB+RF_ARM,&arm);
    if(!rc)rc=bs_read32(dev,BS_MMIO,MB+CONSUMED_LO,&completed);
    if(rc)return rc;
    if(mode!=3u || enable!=1u || state!=0u || fill<3u || fill>16u ||
       producer!=fill || arm!=0u || completed!=0u)
        return HYDRASDR_ERROR_STATE;
    /* Use the exact first host HIF payload word as initial scalar PHY IQ,
     * not a synthesized replacement for sample index zero. */
    uint32_t first_word=UINT32_MAX;
    rc=bs_read32(dev,BS_MMIO,0x0203e000u,&first_word);
    if(rc)return rc;
    uint16_t i=(uint16_t)(first_word&0xfffu),q=(uint16_t)((first_word>>16)&0xfffu);
    if(i&0x800u)i|=0xf000u;
    if(q&0x800u)q|=0xf000u;
    const struct hydra_tx_iq_step initial={(int16_t)i,(int16_t)q,100};
    /* 100ms device watchdog is a fail-safe upper bound if the 20ms host
     * loop stalls. It remains shorter than the general 1s TX test. */
    rc=mt_diag_transact_locked(dev,TX_WD_ARM,100000u,&result);
    if(rc){
        uint32_t ignored=0;
        (void)mt_diag_transact_locked(dev,TX_WD_DISARM,0,&ignored);
        failure(dev,rc);
        return rc;
    }
    armed=true;may_have_started=true;
    rc=start(dev,0,&initial);
    if(!rc){
        rc=bs_read32(dev,BS_MMIO,regs[TONE0].addr,&go);
        if(!rc && !(go&UINT32_C(0x40000000)))rc=HYDRASDR_ERROR_STATE;
    }
    uint64_t host_start=0;
    if(!rc){
        rc=write_staged_word(dev,MB+RF_ARM,1u);
        if(!rc){
            host_start=bs_now_ns();
            tx->host_start_ns=host_start;
            if(on_tx_phase){on_tx_phase(phase_ctx,1);phase_active=true;}
            rc=send_live_frames(send_ctx); /* only bulk OUT, never MMIO */
        }
    }
    /* Source-qualified completion: N9 consumed and verified each fresh IQ
     * word and produced one scalar PHY write per consumed sample, while
     * the live watchdog, GO and PHY path were active. */
    if(!rc){
        const uint64_t deadline=host_start+UINT64_C(20000000);
        uint32_t samples=UINT32_MAX;
        do{
            rc=bs_read32(dev,BS_MMIO,MB+CONSUMER_STATE,&state);
            if(rc || state==3u || state==4u || state==5u)break;
            if(bs_now_ns()>=deadline){rc=HYDRASDR_ERROR_TIMEOUT;break;}
            bs_sleep_us(100);
        }while(1);
        if(!rc && state!=3u)rc=HYDRASDR_ERROR_STATE;
        if(!rc)rc=bs_read32(dev,BS_MMIO,MB+CONSUMED_LO,&samples);
        if(!rc && samples!=expected_samples)rc=HYDRASDR_ERROR_STATE;
        report.consumed_words=samples;
        report.consumer_state_before_abort=state;
        if(!rc)rc=bs_read32(dev,BS_MMIO,MB+RF_WRITES,&report.rf_setpoint_writes);
        if(!rc)rc=bs_read32(dev,BS_MMIO,MB+RF_LAST,&report.rf_last_iq);
        if(!rc)rc=bs_read32(dev,BS_MMIO,MB+RF_FIRST,&report.rf_first_us);
        if(!rc)rc=bs_read32(dev,BS_MMIO,MB+RF_FINAL,&report.rf_last_us);
        if(!rc)rc=bs_read32(dev,BS_MMIO,MB+CONSUMER_UNDERRUN,&report.consumer_underruns);
        if(!rc)rc=bs_read32(dev,BS_MMIO,MB+FIFO_HIGHWATER,&report.fifo_highwater);
        if(!rc)rc=bs_read32(dev,BS_MMIO,MB+FIFO_FILL,&report.fifo_fill_after);
        if(!rc)rc=bs_read32(dev,BS_MMIO,MB+STAGE_PACKETS,&report.producer_packets);
        if(!rc)rc=bs_read32(dev,BS_MMIO,MB+252u,&report.consumed_packets);
        if(!rc && (report.rf_setpoint_writes!=expected_samples ||
                   report.consumer_underruns || report.consumed_packets!=expected_samples/64u ||
                   report.producer_packets!=expected_samples/64u ||
                   report.fifo_fill_after || report.fifo_highwater>16u))
            rc=HYDRASDR_ERROR_STATE;
        if(!rc)tx->completed_steps=1;
    }
    /* MANDATORY writer quiescence and RF GO cut-off BEFORE restoring PHY IQ.
     * Do not stop the writer merely by clearing its ARM: ABORT service calls
     * tx_shutdown when ARM=1, then ACKs after it will never write again. */
    int abort_rc=0,restore_rc=0,status_rc=0,disarm_rc=0,read_rc=0;
    uint32_t ack=UINT32_MAX;
    if(may_have_started){
        abort_rc=bs_write32(dev,BS_MMIO,MB+ABORT_REQ,1u);
        if(!abort_rc)abort_rc=bs_poll32(dev,BS_MMIO,MB+ABORT_ACK,
                                      UINT32_MAX,1u,50u);
        if(!abort_rc)abort_rc=bs_read32(dev,BS_MMIO,MB+ABORT_ACK,&ack);
        report.abort_ack=ack;
        /* If N9 cannot ACK, force TX GO OFF via known host restoration
         * anyway; mark USB reset and reject the run, never claim safety. */
        if(phase_active && on_tx_phase){on_tx_phase(phase_ctx,0);phase_active=false;}
        restore_rc=restore(dev,tx->before);
        tx->host_stop_ns=bs_now_ns();
    }
    if(armed){
        status_rc=mt_diag_transact_locked(dev,TX_WD_STATUS,0,&wd);
        report.watchdog_state_before_disarm=wd;
        disarm_rc=mt_diag_transact_locked(dev,TX_WD_DISARM,0,&result);
        read_rc=snapshot(dev,tx->after);
    }
    if(!rc)rc=abort_rc;
    if(!rc && ack!=1u)rc=HYDRASDR_ERROR_STATE;
    if(!rc)rc=restore_rc;
    if(!rc)rc=status_rc;
    if(!rc && wd!=1u)rc=HYDRASDR_ERROR_STATE;
    if(!rc)rc=disarm_rc;
    if(!rc)rc=read_rc;
    if(!rc && memcmp(tx->before,tx->after,sizeof(tx->before)))
        rc=HYDRASDR_ERROR_STATE;
    if(rc){failure(dev,rc);return rc;}
    *out=report;
    return 0;
}
