/* SPDX-License-Identifier: MIT
 * Injected USB register model: prove named RX/PHY snapshot is read-only and
 * complete, and rejects invalid/partial calibration results.
 */
#include "internal.h"
#include "hydra_measurement.h"
#include "chips/mt7921.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(p) do{if(!(p)){fprintf(stderr,"%s:%d CHECK %s failed\n",__FILE__,__LINE__,#p);exit(1);}}while(0)
struct model{uint32_t control;unsigned mmio_reads,bridge_reads,bridge_writes,bad_writes;uint32_t fail_addr;};
static int read32(void *p,enum bs_space space,uint32_t address,uint32_t *value)
{
    struct model*m=p;
    if(space==BS_MMIO){
        if(address==m->fail_addr) return HYDRASDR_ERROR_IO;
        m->mmio_reads++;*value=address^0x55aa0012u;return 0;
    }
    if(space==BS_BRIDGE){
        m->bridge_reads++;
        switch(address){
            case 0x7c004000:*value=0;return 0;
            case 0x7c004010:*value=m->control;return 0;
            case 0x7c004018:*value=0x0076000u | (m->control & 0x3fff);return 0;
            default: return HYDRASDR_ERROR_INVALID;
        }
    }
    return HYDRASDR_ERROR_INVALID;
}
static int write32(void *p,enum bs_space space,uint32_t address,uint32_t value)
{
    struct model*m=p;
    if(space!=BS_BRIDGE){m->bad_writes++;return HYDRASDR_ERROR_INVALID;}
    m->bridge_writes++;
    if(address==0x7c004010){
        if(!(value & 0x1000u)){m->bad_writes++;return HYDRASDR_ERROR_INVALID;}
        m->control=value;return 0;
    }
    if(address==0x7c004014 && value==0)return 0;
    m->bad_writes++;return HYDRASDR_ERROR_INVALID;
}
int main(void)
{
    struct model m={0};
    hydrasdr_dev_t dev={.backend=&bs_mt7921_backend,
                        .rate=10000000,.rx_mode=HYDRASDR_RX_SINGLE0,
                        .control_mutex=PTHREAD_MUTEX_INITIALIZER};
    dev.io.context=&m;
    dev.io.read32=read32;
    dev.io.write32=write32;
    atomic_init(&dev.frequency,2437000000ULL);
    struct hydrasdr_rf_cal_snapshot out={0};
    CHECK(hydrasdr_get_rf_cal_snapshot(&dev,&out)==HYDRASDR_ERROR_STATE);
    CHECK(hydrasdr_get_rf_cal_snapshot(NULL,&out)==HYDRASDR_ERROR_INVALID);
    CHECK(hydrasdr_get_rf_cal_snapshot(&dev,NULL)==HYDRASDR_ERROR_INVALID);
    CHECK(hydrasdr_rf_cal_mmio_info(HYDRASDR_RF_CAL_MMIO_COUNT)==NULL);
    CHECK(hydrasdr_rf_cal_adie_info(HYDRASDR_RF_CAL_ADIE_COUNT)==NULL);
    CHECK(hydrasdr_rf_cal_mmio_info(0)->address==0x830000c4u);
    CHECK(hydrasdr_rf_cal_adie_info(0)->address==0x768u);
    CHECK(mt_rf_cal_snapshot_capture(&dev)==0);
    CHECK(m.mmio_reads==HYDRASDR_RF_CAL_MMIO_COUNT);
    CHECK(m.bridge_writes==2*HYDRASDR_RF_CAL_ADIE_COUNT*2);
    CHECK(m.bad_writes==0);
    CHECK(hydrasdr_get_rf_cal_snapshot(&dev,&out)==0);
    CHECK(out.schema_version==HYDRASDR_RF_CAL_SNAPSHOT_VERSION);
    CHECK(out.phase==HYDRASDR_RF_CAL_PHASE_PRE_GO);
    CHECK(out.sample_rate==10000000);
    CHECK(out.frequency_hz==2437000000ULL);
    CHECK(out.rx_mode==HYDRASDR_RX_SINGLE0);
    CHECK(out.mmio[0]==(0x830000c4u^0x55aa0012u));
    CHECK(out.adie[0][0] == (0x0076000u | ((1u<<12)|0x768u)));
    CHECK(out.adie[1][8] == (0x0076000u | ((1u<<13)|(1u<<12)|0xc0cu)));
    out.schema_version=0;
    CHECK(hydrasdr_rf_cal_flags(&out)==0);
    out.schema_version=HYDRASDR_RF_CAL_SNAPSHOT_VERSION;
    memset(out.mmio,0,sizeof(out.mmio));
    memset(out.adie,0,sizeof(out.adie));
    out.mmio[0]=7;
    out.mmio[13]=1u<<31;
    out.adie[1][0]=1u<<31;
    CHECK(hydrasdr_rf_cal_flags(&out)==
          (HYDRASDR_RF_CAL_MANUAL0|HYDRASDR_RF_CAL_TXPWR_OVERRIDE1|
           HYDRASDR_RF_CAL_ADIE_GAIN_OVERRIDE_RF1));
    struct hydra_frontend_observation live={0}, unchanged={0};
    CHECK(hydra_observe_frontend(&dev,&live)==HYDRASDR_ERROR_BUSY);
    atomic_store(&dev.run_state,BS_RUNNING);dev.control_ready=true;
    CHECK(hydra_observe_frontend(&dev,&live)==0);
    CHECK(live.registers.phase==HYDRA_RF_PHASE_LIVE_SEQUENTIAL);
    CHECK(live.host_end_ns>=live.host_begin_ns && live.host_begin_ns);
    CHECK(dev.rf_cal_snapshot.phase==HYDRASDR_RF_CAL_PHASE_PRE_GO);
    CHECK(m.bad_writes==0);
    m.fail_addr=0x830000c4u;
    CHECK(hydra_observe_frontend(&dev,&unchanged)==HYDRASDR_ERROR_IO);
    struct hydra_frontend_observation zero={0};CHECK(!memcmp(&unchanged,&zero,sizeof(zero)));
    CHECK(dev.rf_cal_snapshot_valid);
    dev.control_ready=false;
    CHECK(hydra_observe_frontend(&dev,&live)==HYDRASDR_ERROR_BUSY);

    CHECK(mt_rf_cal_snapshot_capture(&dev)==HYDRASDR_ERROR_IO);
    CHECK(!dev.rf_cal_snapshot_valid);
    CHECK(hydrasdr_get_rf_cal_snapshot(&dev,&out)==HYDRASDR_ERROR_STATE);
    CHECK(pthread_mutex_destroy(&dev.control_mutex)==0);
    puts("read-only, bounded pre-GO calibration snapshot mock passed");
    return 0;
}
