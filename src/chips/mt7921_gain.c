/* Fenvi dual-branch experimental gain: volatile, RF0+RF1, independent baselines.
 * This is NOT a calibrated dB gain or a validated AGC disable.
 * Both branches are intentionally given identical *codes*.
 */
#include "mt7921.h"
#include "hydra_measurement.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static const uint32_t mask[4] = {0xf7f7f707u,0x3f3f0000u,0x8070f000u,0xff800000u};
static const uint32_t base[4] = {0x830000c4u,0x83000108u,0x83000150u,0x768u};

static uint32_t addr(unsigned index) {
    unsigned bank=index/4,field=index%4;
    return field==3?base[field]:base[field]+bank*0x10000u;
}
static unsigned analog_port(unsigned index) {return index/4?0u:MT_RX_PORT;}

static int rd(hydrasdr_dev_t *d,unsigned index,uint32_t *out){
    if(index%4==3)return mt_rf_read(d,analog_port(index),addr(index),out);
    return bs_read32(d,BS_MMIO,addr(index),out);
}
static int wr(hydrasdr_dev_t *d,unsigned index,uint32_t value){
    unsigned field=index%4;uint32_t actual;
    if(field==3) {
       BS_TRY(mt_rf_update(d,analog_port(index),addr(index),mask[field],value&mask[field]));
    } else {
       BS_TRY(bs_update32(d,BS_MMIO,addr(index),mask[field],value&mask[field]));
    }
    BS_TRY(rd(d,index,&actual));
    return ((actual^value)&mask[field])?HYDRASDR_ERROR_STATE:0;
}
static int snapshot(hydrasdr_dev_t*d,uint32_t v[8]){
    memset(v,0,8*sizeof(*v));
    const unsigned n=d->rx_mode==HYDRASDR_RX_DUAL?8u:4u;
    for(unsigned i=0;i<n;i++) BS_TRY(rd(d,i,&v[i]));
    return 0;
}
static int gain_values(const struct hydrasdr_gain*g,uint32_t v[4]){
    uint32_t l=g->lna,p=g->lpf,z=g->digital;
    if(l>5||p>8||z>12)return HYDRASDR_ERROR_INVALID;
    v[0]=7u|(l<<8)|(p<<12)|(l<<16)|(p<<20)|(l<<24)|(p<<28);
    v[1]=(z<<24)|(z<<16);
    v[2]=0x80000000u|(l<<20);
    v[3]=0x80000000u|(l<<27)|(p<<23);
    return 0;
}
static int restore_to(hydrasdr_dev_t*d,const uint32_t target[8]){
    /* Restore digital/PHY first, analog last, but attempt all operations. */
    static const unsigned dual_order[8]={1,2,0,5,6,4,3,7};
    static const unsigned single_order[4]={1,2,0,3};
    const unsigned *order=d->rx_mode==HYDRASDR_RX_DUAL?dual_order:single_order;
    const unsigned n=d->rx_mode==HYDRASDR_RX_DUAL?8u:4u;
    int first=0;
    for(unsigned i=0;i<n;i++) {
        int e=wr(d,order[i],target[order[i]]);
        if(e<0&&!first)first=e;
    }
    return first;
}
static int force_gain(hydrasdr_dev_t*d,const uint32_t before[8],
                      const struct hydrasdr_gain*g,const struct hydrasdr_gain*g1){
    uint32_t v[2][4];
    BS_TRY(gain_values(g,v[0]));
    BS_TRY(gain_values(g1,v[1]));
    static const unsigned dual_order[8]={3,7,1,5,2,6,0,4};
    static const unsigned single_order[4]={3,1,2,0};
    const unsigned *order=d->rx_mode==HYDRASDR_RX_DUAL?dual_order:single_order;
    const unsigned n=d->rx_mode==HYDRASDR_RX_DUAL?8u:4u;
    for(unsigned j=0;j<n;j++){
        unsigned i=order[j];
        int e=wr(d,i,v[i/4][i%4]);
        if(e<0) {
            int rollback=restore_to(d,before);
            return rollback<0?HYDRASDR_ERROR_STATE:e;
        }
    }
    return 0;
}
int mt_apply_gain(hydrasdr_dev_t*d){
    /* Experimental fixed code profiles, both banks; disabled unless opted in. */
    const char *profile=getenv("HYDRA_GAIN_PROFILE");
    if(profile && *profile){
        if(d->mt7921.gain_pair_enabled)return HYDRASDR_ERROR_STATE;
        if(!strcmp(profile,"low"))d->gain=(struct hydrasdr_gain){1,4,0};
        else if(!strcmp(profile,"medium"))d->gain=(struct hydrasdr_gain){2,5,0};
        else if(!strcmp(profile,"high"))d->gain=(struct hydrasdr_gain){3,6,0};
        else if(!strcmp(profile,"max"))d->gain=(struct hydrasdr_gain){5,8,0};
        else return HYDRASDR_ERROR_INVALID;
        d->manual=true;
    }
    uint32_t v[8],coarse[2]={0,0};
    const unsigned banks=d->rx_mode==HYDRASDR_RX_DUAL?2u:1u;
    BS_TRY(snapshot(d,v));
    BS_TRY(bs_read32(d,BS_MMIO,0x830000d0,&coarse[0]));
    if(banks==2)BS_TRY(bs_read32(d,BS_MMIO,0x830100d0,&coarse[1]));
    for(unsigned b=0;b<banks;b++)
       if((v[4*b]&7u)||(v[4*b+2]&0x80000000u)||
          (v[4*b+3]&0x80000000u)||(coarse[b]&(1u<<27)))
            return HYDRASDR_ERROR_STATE;
    memcpy(d->mt7921.gain_baseline,v,sizeof(v));
    d->mt7921.gain_saved=true;
    if(!d->manual)return 0;
    int err=force_gain(d,v,&d->gain,
         d->mt7921.gain_pair_enabled?&d->mt7921.gain_pair[1]:&d->gain);
    if(err<0)return err;
    if(getenv("HYDRA_GAIN_DIAG")){
       for(unsigned i=0;i<4*banks;i++){
          uint32_t a=0;
          int r=rd(d,i,&a);
          fprintf(stderr,"DUALGAIN bank=%u field=%u addr=%08x value=%08x read_rc=%d\n",
                  i/4,i%4,addr(i),a,r);
       }
    }
    return 0;
}
int mt_change_gain(hydrasdr_dev_t*d,bool manual,const struct hydrasdr_gain*g){
    uint32_t before[8],valid[4];
    if(!g)return HYDRASDR_ERROR_INVALID;
    BS_TRY(gain_values(g,valid));
    if(!d->mt7921.gain_saved)return HYDRASDR_ERROR_STATE;
    BS_TRY(snapshot(d,before));
    if(!manual)return restore_to(d,d->mt7921.gain_baseline);
    return force_gain(d,before,g,g);
}
int mt_restore_gain(hydrasdr_dev_t*d){
    if(!d->mt7921.gain_saved)return 0;
    int e=restore_to(d,d->mt7921.gain_baseline);
    if(e==0)d->mt7921.gain_saved=false;
    return e;
}

int hydra_set_rx_gain_pair(hydrasdr_dev_t*d,const struct hydrasdr_gain gains[2])
{
    if(!d || !gains)return HYDRASDR_ERROR_INVALID;
    uint32_t values[4];
    BS_TRY(gain_values(&gains[0],values));BS_TRY(gain_values(&gains[1],values));
    int rc=0;pthread_mutex_lock(&d->control_mutex);
    unsigned state=atomic_load(&d->run_state);
    if(!d->backend || d->backend->chipset!=HYDRASDR_CHIP_MT7921 ||
       d->rx_mode!=HYDRASDR_RX_DUAL) {rc=HYDRASDR_ERROR_UNSUPPORTED;goto done;}
    if(state!=BS_IDLE && (state!=BS_RUNNING || !d->control_ready)) {
        rc=HYDRASDR_ERROR_BUSY;goto done;
    }
    if(!d->manual){rc=HYDRASDR_ERROR_STATE;goto done;}
    if(state==BS_RUNNING) {
        if(!d->mt7921.gain_saved){rc=HYDRASDR_ERROR_STATE;goto done;}
        uint32_t before[8];
        rc=snapshot(d,before);
        if(!rc)rc=force_gain(d,before,&gains[0],&gains[1]);
        if(rc) {
            d->control_ready=false;d->control_error=rc;
            d->control_error_stage=HYDRASDR_STAGE_GAIN;
            unsigned expected=BS_RUNNING;
            atomic_compare_exchange_strong(&d->run_state,&expected,BS_CONTROL_FAILED);
            goto done;
        }
    }
    d->gain=gains[0];memcpy(d->mt7921.gain_pair,gains,sizeof(d->mt7921.gain_pair));
    d->mt7921.gain_pair_enabled=true;
done:
    pthread_mutex_unlock(&d->control_mutex);return rc;
}
int hydra_get_rx_gain_pair(hydrasdr_dev_t*d,struct hydrasdr_gain gains[2],int *manual)
{
    if(!d || !gains || !manual)return HYDRASDR_ERROR_INVALID;
    pthread_mutex_lock(&d->control_mutex);
    gains[0]=d->gain;gains[1]=d->mt7921.gain_pair_enabled?d->mt7921.gain_pair[1]:d->gain;
    *manual=d->manual;pthread_mutex_unlock(&d->control_mutex);return 0;
}
