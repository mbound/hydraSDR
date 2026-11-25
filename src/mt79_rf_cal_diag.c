/* SPDX-License-Identifier: MIT
 * hydraSDR / Birria-compatible read-only RF front-end calibration diagnostics.
 * Capture runs during ordinary RX setup, before GO and without a second USB
 * owner. No eFuse writes, stock-WM RF_AT commands, or RF transmission.
 *
 * A-die port numbers name physical SPI ports; no verified antenna assignment.
 * All RX gain, TSSI and PA words remain RAW registers, not calibrated units.
 */
#include "internal.h"
#include "chips/mt7921.h"
#include <string.h>
#include <time.h>
#include "hydra_measurement.h"

static const struct hydrasdr_rf_diag_reg mmio_regs[] = {
    {"rx0_phy_lna_lpf", 0x830000c4},
    {"rx1_phy_lna_lpf", 0x830100c4},
    {"rx0_phy_dgain", 0x83000108},
    {"rx1_phy_dgain", 0x83010108},
    {"rx0_phy_override", 0x83000150},
    {"rx1_phy_override", 0x83010150},
    {"rx0_coarse", 0x830000d0},
    {"rx1_coarse", 0x830100d0},
    {"rx0_filter", 0x83081000},
    {"rx1_filter", 0x83091000},
    {"rx0_tssi", 0x8300c000},
    {"rx1_tssi", 0x8301c000},
    {"rx0_txpwr", 0x8300c508},
    {"rx1_txpwr", 0x8301c508},
    {"rx0_tone", 0x8308fc00},
    {"rx1_tone", 0x8309fc00},
    {"rx0_tone_iq", 0x8308fc08},
    {"rx1_tone_iq", 0x8309fc08},
    {"rx0_txpath", 0x8300d008},
    {"rx1_txpath", 0x8301d008},
    {"dfe_source", 0x830ad440},
    {"dfe_mux", 0x830ad448}
};
static const struct hydrasdr_rf_diag_reg adie_regs[] = {
    {"gain_analog",0x768},
    {"analog_filter",0x160},
    {"rf_switch",0x204},
    {"rf_selector",0x21c},
    {"pll",0xc68},
    {"pll_companion",0xca8},
    {"filter_width",0xb00},
    {"filter_shape",0xb04},
    {"vco",0xc0c}
};
_Static_assert(sizeof(mmio_regs)/sizeof(*mmio_regs)==HYDRASDR_RF_CAL_MMIO_COUNT,
               "calibration RF MMIO ABI differs from source-qualified inventory");
_Static_assert(sizeof(adie_regs)/sizeof(*adie_regs)==HYDRASDR_RF_CAL_ADIE_COUNT,
               "calibration A-die ABI differs from source-qualified inventory");

const struct hydrasdr_rf_diag_reg *hydrasdr_rf_cal_mmio_info(unsigned index)
{
    return index < HYDRASDR_RF_CAL_MMIO_COUNT ? &mmio_regs[index] : NULL;
}
const struct hydrasdr_rf_diag_reg *hydrasdr_rf_cal_adie_info(unsigned index)
{
    return index < HYDRASDR_RF_CAL_ADIE_COUNT ? &adie_regs[index] : NULL;
}

/* Called only by the RF owner during boot/prepare, after ordinary analog
 * tuning, manual-gain handling and filter setup but BEFORE firmware GO.
 * Fail closed: no partially filled snapshot can reach a public caller.
 * RF-SPI reads change transient bridge command/data registers, but only
 * perform ANALOG READ transactions, never program the target A-die.
 */
int mt_rf_cal_snapshot_capture(hydrasdr_dev_t *dev)
{
    struct hydrasdr_rf_cal_snapshot snap={0};
    if(!dev || !dev->backend || dev->backend->chipset!=HYDRASDR_CHIP_MT7921)
        return HYDRASDR_ERROR_UNSUPPORTED;
    dev->rf_cal_snapshot_valid=false;
    snap.schema_version=HYDRASDR_RF_CAL_SNAPSHOT_VERSION;
    snap.frequency_hz=atomic_load(&dev->frequency);
    snap.sample_rate=dev->rate;
    snap.rx_mode=dev->rx_mode;
    snap.phase=HYDRASDR_RF_CAL_PHASE_PRE_GO;
    for(unsigned i=0;i<HYDRASDR_RF_CAL_MMIO_COUNT;i++)
        BS_TRY(bs_read32(dev,BS_MMIO,mmio_regs[i].address,&snap.mmio[i]));
    for(unsigned p=0;p<2;p++)
        for(unsigned i=0;i<HYDRASDR_RF_CAL_ADIE_COUNT;i++)
            BS_TRY(mt_rf_read(dev,p,adie_regs[i].address,&snap.adie[p][i]));
    dev->rf_cal_snapshot=snap;
    dev->rf_cal_snapshot_valid=true;
    return 0;
}

/* Cached only, no USB I/O; control mutex serializes readers with capture and
 * reset. This is not a continuous AGC monitor and cannot attribute a change
 * to a specific IQ sample or waveform interval.
 */
int hydrasdr_get_rf_cal_snapshot(hydrasdr_dev_t *dev,
                                  struct hydrasdr_rf_cal_snapshot *out)
{
    if(!dev || !out) return HYDRASDR_ERROR_INVALID;
    pthread_mutex_lock(&dev->control_mutex);
    int rc=0;
    if(!dev->backend || dev->backend->chipset!=HYDRASDR_CHIP_MT7921)
        rc=HYDRASDR_ERROR_UNSUPPORTED;
    else if(!dev->rf_cal_snapshot_valid)
        rc=HYDRASDR_ERROR_STATE;
    else
        *out=dev->rf_cal_snapshot;
    pthread_mutex_unlock(&dev->control_mutex);
    return rc;
}

/* Never convert to dB or dBm until a per-die calibrated model exists. */
uint32_t hydrasdr_rf_cal_flags(const struct hydrasdr_rf_cal_snapshot *s)
{
    if(!s || s->schema_version!=HYDRASDR_RF_CAL_SNAPSHOT_VERSION)
        return 0;
    uint32_t flags=0;
    for(unsigned bank=0;bank<2;bank++){
        if((s->mmio[bank]&7u) ||
           (s->mmio[bank+4]&(1u<<31)) ||
           (s->mmio[bank+6]&(1u<<27)))
            flags |= bank==0 ? HYDRASDR_RF_CAL_MANUAL0 : HYDRASDR_RF_CAL_MANUAL1;
        if(s->mmio[bank+12]&(1u<<31))
            flags |= bank==0 ? HYDRASDR_RF_CAL_TXPWR_OVERRIDE0
                             : HYDRASDR_RF_CAL_TXPWR_OVERRIDE1;
    }
    if (s->adie[0][0] & (1u<<31))
        flags |= HYDRASDR_RF_CAL_ADIE_GAIN_OVERRIDE_RF0;
    if (s->adie[1][0] & (1u<<31))
        flags |= HYDRASDR_RF_CAL_ADIE_GAIN_OVERRIDE_RF1;
    return flags;
}

static uint64_t monotonic_ns(void)
{
    struct timespec t;
    if(clock_gettime(CLOCK_MONOTONIC,&t)) return 0;
    return (uint64_t)t.tv_sec*1000000000u+(uint64_t)t.tv_nsec;
}
int hydra_observe_frontend(hydrasdr_dev_t *dev,struct hydra_frontend_observation *out)
{
    if(!dev || !out) return HYDRASDR_ERROR_INVALID;
    struct hydra_frontend_observation o={0};
    int rc=0;
    pthread_mutex_lock(&dev->control_mutex);
    if(!dev->backend || dev->backend->chipset!=HYDRASDR_CHIP_MT7921) {
        rc=HYDRASDR_ERROR_UNSUPPORTED; goto done;
    }
    if(atomic_load(&dev->run_state)!=BS_RUNNING || !dev->control_ready) {
        rc=HYDRASDR_ERROR_BUSY; goto done;
    }
    o.host_begin_ns=monotonic_ns();
    if(!o.host_begin_ns) {rc=HYDRASDR_ERROR_IO; goto done;}
    o.requested_gain=dev->gain; o.manual_requested=dev->manual;
    o.requested_gain_pair[0]=dev->gain;
    o.requested_gain_pair[1]=dev->mt7921.gain_pair_enabled?dev->mt7921.gain_pair[1]:dev->gain;
    o.independent_gain_requested=dev->mt7921.gain_pair_enabled;
    o.registers.schema_version=HYDRASDR_RF_CAL_SNAPSHOT_VERSION;
    o.registers.phase=HYDRA_RF_PHASE_LIVE_SEQUENTIAL;
    o.registers.frequency_hz=atomic_load(&dev->frequency);
    o.registers.sample_rate=dev->rate; o.registers.rx_mode=dev->rx_mode;
    for(unsigned i=0;i<HYDRASDR_RF_CAL_MMIO_COUNT;i++) {
        rc=bs_read32(dev,BS_MMIO,mmio_regs[i].address,&o.registers.mmio[i]);
        if(rc) goto done;
    }
    for(unsigned port=0;port<2;port++)
        for(unsigned i=0;i<HYDRASDR_RF_CAL_ADIE_COUNT;i++) {
            rc=mt_rf_read(dev,port,adie_regs[i].address,&o.registers.adie[port][i]);
            if(rc) goto done;
        }
    o.host_end_ns=monotonic_ns();
    if(!o.host_end_ns) {rc=HYDRASDR_ERROR_IO; goto done;}
    if(atomic_load(&dev->run_state)!=BS_RUNNING || !dev->control_ready) {
        rc=HYDRASDR_ERROR_BUSY; goto done;
    }
    *out=o;
done:
    pthread_mutex_unlock(&dev->control_mutex);
    return rc;
}
