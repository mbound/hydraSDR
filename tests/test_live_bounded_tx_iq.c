/* SPDX-License-Identifier: MIT
 * Explicit opt-in on-device bounded TX IQ control + simultaneous RX test.
 * No external RF output or absolute-power measurement is claimed here.
 * Requires shielded/terminated RF lab setup verified by operator.
 * Not a default CTest. NEVER writes OTP, EEPROM or persistent ROM patch.
 */
#define _POSIX_C_SOURCE 200809L
#include "internal.h"
#include "hydra_tx_control.h"
#include <inttypes.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <math.h>
#include <string.h>
#include <time.h>

#define CAPTURE_WORDS 2000000u
#define CAPTURE_I16 (CAPTURE_WORDS*2u)
struct power_stats {
    uint64_t words,energy,clips;
    uint32_t peak;
};
struct experiment {
    hydrasdr_dev_t *dev;
    int16_t *capture[3];
    size_t captured[3];
    uint64_t first_offset[3];
    atomic_uint_fast64_t delivered;
    atomic_uint phase;
    struct power_stats stats[3];
    struct power_stats lane_stats[3][2];
    unsigned rx_lanes;
    int stream_rc;
};
static void sleep_ms(unsigned m){
    struct timespec t={(time_t)(m/1000u),(long)(m%1000u)*1000000L};
    nanosleep(&t,NULL);
}
static void rx_callback(const int16_t *iq,uint32_t n,uint64_t first,void *ctx){
    struct experiment *e=ctx;
    if(!iq || !n)return;
    unsigned phase=atomic_load(&e->phase);
    if(phase>2)phase=2;
    struct power_stats *s=&e->stats[phase];
    for(uint32_t j=0;j<n;j++){
        const int32_t i=iq[2u*j],q=iq[2u*j+1];
        uint32_t ai=(uint32_t)(i<0?-i:i),aq=(uint32_t)(q<0?-q:q);
        uint32_t p=ai>aq?ai:aq;
        uint64_t energy=(uint64_t)(i*i)+(uint64_t)(q*q);
        if(p>s->peak)s->peak=p;
        s->energy+=energy;
        if(p>=8100u)s->clips++;
        struct power_stats *lane=&e->lane_stats[phase][e->rx_lanes==2?((first+j)&1u):0];
        lane->words++;lane->energy+=energy;
        if(p>lane->peak)lane->peak=p;
        if(p>=8100u)lane->clips++;
    }
    s->words+=n;
    if(e->capture[phase] && e->captured[phase]<CAPTURE_WORDS){
        size_t take=(size_t)n;
        if(!e->captured[phase])e->first_offset[phase]=first;
        if(take>CAPTURE_WORDS-e->captured[phase])
            take=CAPTURE_WORDS-e->captured[phase];
        memcpy(e->capture[phase]+2u*e->captured[phase],iq,take*4u);
        e->captured[phase]+=take;
    }
    atomic_fetch_add(&e->delivered,n);
}
/* Only flips the host capture phase; never re-enters USB/control mutex. */
static void tx_iq_phase(void *ctx,int active){
    struct experiment *e=ctx;
    atomic_store(&e->phase,active?1u:2u);
}
static void *rx_worker(void *ctx){
    struct experiment *e=ctx;
    e->stream_rc=hydrasdr_read_async(e->dev,rx_callback,e,0,0);
    return NULL;
}
static int save_private_capture(const char *prefix,const struct experiment *e,
                                unsigned stage){
    if(!prefix || !*prefix)return 0;
    if(stage>2)return -1;
    const char *names[]={"pre","tx","post"};
    char path[1024];
    int n=snprintf(path,sizeof(path),"%s.%s.cs16",prefix,names[stage]);
    if(n<0 || (size_t)n>=sizeof(path))return -1;
    /* Do not leak potentially identifiable ambient IQ through shared paths. */
    int fd=open(path,O_WRONLY|O_CREAT|O_EXCL|O_NOFOLLOW,0600);
    if(fd<0)return -1;
    size_t done=0,want=e->captured[stage]*4u;
    const uint8_t *data=(const uint8_t *)e->capture[stage];
    int rc=0;
    while(done<want){
        ssize_t n=write(fd,data+done,want-done);
        if(n<=0){rc=-1;break;}
        done+=(size_t)n;
    }
    if(fsync(fd))rc=-1;
    if(close(fd))rc=-1;
    if(rc)unlink(path);
    return rc;
}
int main(int argc,char **argv){
    const bool dry_run=getenv("HYDRA_TX_DRY_RUN") &&
                       !strcmp(getenv("HYDRA_TX_DRY_RUN"),"1");
    if(argc>2 || (argc==2 && !strcmp(argv[1],"--help")) ||
       (!dry_run && (
       !getenv("HYDRA_TX_EXPERIMENTAL") ||
       strcmp(getenv("HYDRA_TX_EXPERIMENTAL"),"1") ||
       !getenv("HYDRA_RF_SHIELDED") || strcmp(getenv("HYDRA_RF_SHIELDED"),"1") ||
       !getenv("HYDRA_TX_ALLOW_UNCALIBRATED_POWER") ||
       strcmp(getenv("HYDRA_TX_ALLOW_UNCALIBRATED_POWER"),"1")))){
        fprintf(stderr,"Requires HYDRA_TX_EXPERIMENTAL=1, HYDRA_RF_SHIELDED=1 and "
                       "HYDRA_TX_ALLOW_UNCALIBRATED_POWER=1. "
                       "Optional argument is a NEW private CS16 prefix (3 phase files).\n");
        return 2;
    }
    /* Exact 2024 WM gain override (0x8300/1C508[7:0]). The default 0
     * preserves all previously qualified bounded TX experiments. Per-code
     * comparison is operator opt-in, still not a calibrated dBm setting. */
    unsigned requested_raw_gain=0;
    const char *code=getenv("HYDRA_TX_RAW_GAIN_CODE");
    if(code){
        char *end=NULL;
        errno=0;
        if(!*code || *code=='-' || *code=='+' ||
           !(*code>='0' && *code<='9'))return 2;
        unsigned long value=strtoul(code,&end,10);
        if(errno || !end || *end || value>255)return 2;
        requested_raw_gain=(unsigned)value;
    }
    const bool staged_iq=getenv("HYDRA_TX_IQ_STAGED") &&
                        !strcmp(getenv("HYDRA_TX_IQ_STAGED"),"1");
    /* A deterministic host-preloaded *cyclic* QPSK PRBS-like sequence is
     * independently recognizable in captured RX IQ, unlike the rotating
     * single-tone calibration table. This does NOT turn the PHY into a DAC
     * FIFO and is only allowed under the existing bounded staged-TX gates. */
    const char *pattern_env=getenv("HYDRA_TX_IQ_PATTERN");
    const bool prbs_qpsk=pattern_env && !strcmp(pattern_env,"prbs_qpsk");
    if(pattern_env && !prbs_qpsk)return 2;
    if(prbs_qpsk && !staged_iq)return 2;
    /* Conservative default DAC-setpoint drive; waveform is the 4-state
     * rotating complex vector. The finite N9 target is explicitly a test
     * rate, never advertised as effective / guaranteed DAC sample timing. */
    unsigned iq_amp=64,iq_period_us=10,iq_table_samples=prbs_qpsk?128u:4u;
    const char *amp=getenv("HYDRA_TX_IQ_AMPLITUDE");
    if(amp){
        char *end=NULL;errno=0;
        if(!*amp || *amp<'0' || *amp>'9')return 2;
        unsigned long v=strtoul(amp,&end,10);
        if(errno || !end || *end || v<8 || v>256)return 2;
        iq_amp=(unsigned)v;
    }
    const char *period=getenv("HYDRA_TX_IQ_PERIOD_US");
    if(period){
        char *end=NULL;errno=0;
        if(!*period || *period<'0' || *period>'9')return 2;
        unsigned long v=strtoul(period,&end,10);
        if(errno || !end || *end || v<2 || v>1000)return 2;
        iq_period_us=(unsigned)v;
    }
    const char *table=getenv("HYDRA_TX_IQ_TABLE_SAMPLES");
    if(table){
        char *end=NULL;errno=0;
        if(!*table || *table<'0' || *table>'9')return 2;
        unsigned long v=strtoul(table,&end,10);
        if(errno || !end || *end || v<4 || v>HYDRA_TX_STAGED_IQ_MAX_SAMPLES ||
           (v & (v-1)))return 2;
        iq_table_samples=(unsigned)v;
    }
    if(prbs_qpsk && iq_table_samples<32u)return 2;
    const char *capture_path=argc==2?argv[1]:NULL;
    struct hydrasdr_device_info info={0};
    if(hydrasdr_get_device_info(0,&info) || info.vid!=0x0e8d || info.pid!=0x7961)
        return 3;
    struct experiment e={0};
    atomic_init(&e.delivered,0);atomic_init(&e.phase,0);
    if(capture_path){
        for(unsigned stage=0;stage<3;stage++){
            e.capture[stage]=calloc(CAPTURE_I16,sizeof(int16_t));
            if(!e.capture[stage]){
                for(unsigned j=0;j<3;j++)free(e.capture[j]);
                return 3;
            }
        }
    }
    int rc=hydrasdr_open(&e.dev,0),opened=rc==0,started=0;
    if(rc)goto output;
    e.rx_lanes=getenv("HYDRA_TX_RX_DUAL") &&
                 !strcmp(getenv("HYDRA_TX_RX_DUAL"),"1")?2u:1u;
    /* Hardware-qualified planner permits only 2x40 Msps RAW2 in dual mode.
     * Do not silently try to run dual at the 10 Msps single-lane rate. */
    rc=hydrasdr_set_sample_rate(e.dev,e.rx_lanes==2?40000000u:10000000u);
    if(!rc)rc=hydrasdr_set_rx_mode(e.dev,e.rx_lanes==2?
                     HYDRASDR_RX_DUAL:HYDRASDR_RX_SINGLE0);
    if(!rc)rc=hydrasdr_set_center_freq(e.dev,UINT64_C(2437000000));
    pthread_t thread;
    if(!rc){
        if(pthread_create(&thread,NULL,rx_worker,&e))rc=HYDRASDR_ERROR_IO;
        else started=1;
    }
    struct hydra_tx_fe_status fe_before={0},fe_after={0};
    struct hydra_rx_fe_status rfe_before={0},rfe_after={0};
    struct hydra_tx_burst_receipt receipt={0};
    struct hydra_tx_staged_iq_receipt staged_receipt={0};
    uint32_t wd0=0,exp0=0,wd2=0,exp2=0;
    int wd_pre=-999,fe_pre=-999,fe_post=-999,wd_post=-999,tx_rc=-999;
    int rx_fe_pre=-999,rx_fe_post=-999;
    if(started){
        for(unsigned k=0;k<125 && atomic_load(&e.delivered)<200000;k++)sleep_ms(20);
        if(!rc && atomic_load(&e.delivered)<200000)rc=HYDRASDR_ERROR_TIMEOUT;
        if(!rc){
            wd_pre=hydra_tx_watchdog_status(e.dev,&wd0,&exp0);
            fe_pre=hydra_tx_fe_observe(e.dev,&fe_before);
            rx_fe_pre=hydra_rx_fe_observe(e.dev,&rfe_before);
            if(wd_pre || fe_pre || rx_fe_pre || wd0!=0 || exp0!=0 ||
               fe_before.mmio[8] || fe_before.mmio[9] ||
               fe_before.mmio[10] || fe_before.mmio[11])
                rc=HYDRASDR_ERROR_STATE;
        }
        if(!rc){
            const struct hydra_tx_burst_config cfg={.chain=0,
                .raw_power_code=(uint8_t)requested_raw_gain};
            const struct hydra_tx_iq_step steps[]={
                {.i=64,.q=0,.dwell_us=2500},
                {.i=0,.q=64,.dwell_us=2500},
                {.i=-64,.q=0,.dwell_us=2500},
                {.i=0,.q=-64,.dwell_us=2500}
            };
            if(!staged_iq || dry_run)atomic_store(&e.phase,1);
            if(dry_run){
                /* Baseline acquisition with identical RX setup and 3 capture
                 * phases, but NEVER writes any TX PHY or arms a watchdog. */
                sleep_ms(35);
                tx_rc=0;
            }else if(staged_iq){
                /* Host-generated cyclic complex rotating IQ with configurable
                 * phase quantization. There is no implicit N9 sine generator:
                 * each signed-12b sample is host-preloaded through USB. */
                struct hydra_tx_iq_sample table[HYDRA_TX_STAGED_IQ_MAX_SAMPLES];
                uint32_t prbs_state=UINT32_C(0x6d4a7951);
                for(unsigned j=0;j<iq_table_samples;j++){
                    if(prbs_qpsk){
                        /* Reproducible xorshift32 QPSK symbols, no generator
                         * shortcuts or RF-frontend changes. Not cryptographic;
                         * bounded finite cyclic test pattern only. */
                        prbs_state^=prbs_state<<13;
                        prbs_state^=prbs_state>>17;
                        prbs_state^=prbs_state<<5;
                        table[j].i=(prbs_state&1u)?(int16_t)iq_amp:-(int16_t)iq_amp;
                        table[j].q=(prbs_state&2u)?(int16_t)iq_amp:-(int16_t)iq_amp;
                    }else{
                        const double angle=6.28318530717958647693*(double)j/
                                           (double)iq_table_samples;
                        table[j].i=(int16_t)lround((double)iq_amp*cos(angle));
                        table[j].q=(int16_t)lround((double)iq_amp*sin(angle));
                    }
                }
                tx_rc=hydra_tx_run_bounded_staged_iq(e.dev,&cfg,table,
                         iq_table_samples,iq_period_us,20000,
                         &staged_receipt,tx_iq_phase,&e);
                receipt=staged_receipt.tx;
            }else tx_rc=hydra_tx_run_bounded_iq(e.dev,&cfg,steps,4,&receipt);
            if(!staged_iq || dry_run)atomic_store(&e.phase,2);
            if(tx_rc)rc=tx_rc;
            if(!rc){
                fe_post=hydra_tx_fe_observe(e.dev,&fe_after);
                rx_fe_post=hydra_rx_fe_observe(e.dev,&rfe_after);
                wd_post=hydra_tx_watchdog_status(e.dev,&wd2,&exp2);
                if(fe_post || rx_fe_post || wd_post || wd2!=0 || exp2!=0 ||
                   (!dry_run && (receipt.completed_steps!=(staged_iq?1u:4u) ||
                   memcmp(receipt.before,receipt.after,sizeof(receipt.before)))) ||
                   memcmp(fe_before.mmio,fe_after.mmio,sizeof(fe_before.mmio)) ||
                   memcmp(fe_before.adie_gain_raw,fe_after.adie_gain_raw,
                          sizeof(fe_before.adie_gain_raw)))
                    rc=HYDRASDR_ERROR_STATE;
            }
        }
        sleep_ms(120);
        int cancel_rc=hydrasdr_cancel_async(e.dev);
        pthread_join(thread,NULL);
        if(!rc)rc=cancel_rc;
        if(!rc)rc=e.stream_rc;
    }
    struct hydrasdr_stream_stats st={0};
    int sr=hydrasdr_get_stream_stats(e.dev,&st);
    int close_rc=hydrasdr_close(e.dev);opened=0;
    if(!rc)rc=sr;
    if(!rc)rc=close_rc;
    if(!rc && (st.dma_errors || st.reset_error || atomic_load(&e.delivered)<400000))
        rc=HYDRASDR_ERROR_STATE;
    if(!rc && capture_path){
        for(unsigned stage=0;stage<3;stage++)
            if(save_private_capture(capture_path,&e,stage))rc=HYDRASDR_ERROR_IO;
    }
    printf("{\"type\":\"hydra_bounded_tx_iq\",\"raw_gain_code\":%u,\"tx_attempted\":%s,"
           "\"dry_run\":%s,"
           "\"tx_rc\":%d,\"tx_completed_steps\":%u,\"rx_words\":%" PRIuFAST64 ","
           "\"rx_pre_words\":%" PRIu64 ",\"rx_during_words\":%" PRIu64 ","
           "\"rx_post_words\":%" PRIu64 ",\"pre_peak\":%u,\"during_peak\":%u,"
           "\"post_peak\":%u,\"pre_energy\":%" PRIu64 ","
           "\"during_energy\":%" PRIu64 ",\"post_energy\":%" PRIu64 ","
           "\"pre_clipped\":%" PRIu64 ",\"during_clipped\":%" PRIu64 ","
           "\"post_clipped\":%" PRIu64 ",\"tx_restored\":%s,"
           "\"watchdog_pre_rc\":%d,\"watchdog_post_rc\":%d,"
           "\"watchdog_final\":%u,\"dma_errors\":%u,"
           "\"reset_error\":%d,\"stream_rc\":%d,\"final_rc\":%d,"
           "\"external_rf_verified\":false}\n",
           requested_raw_gain,
           !dry_run && tx_rc!=-999?"true":"false",dry_run?"true":"false",
           tx_rc,receipt.completed_steps,
           atomic_load(&e.delivered),e.stats[0].words,e.stats[1].words,
           e.stats[2].words,e.stats[0].peak,e.stats[1].peak,e.stats[2].peak,
           e.stats[0].energy,e.stats[1].energy,e.stats[2].energy,
           e.stats[0].clips,e.stats[1].clips,e.stats[2].clips,
           !dry_run && !tx_rc && receipt.completed_steps==(staged_iq?1u:4u)?"true":"false",
           wd_pre,wd_post,wd2,st.dma_errors,st.reset_error,e.stream_rc,rc);
    if(!dry_run && !tx_rc && receipt.completed_steps==(staged_iq?1u:4u)){
        printf("{\"type\":\"hydra_tx_c114_candidate\","
               "\"raw_only\":true,\"units\":null,"
               "\"sample_alignment\":\"host_stage_not_sample_synchronous\","
               "\"pre\":[%u,%u],\"during\":[%u,%u],\"post\":[%u,%u]}\n",
               receipt.candidate_c114[0][0],receipt.candidate_c114[0][1],
               receipt.candidate_c114[1][0],receipt.candidate_c114[1][1],
               receipt.candidate_c114[2][0],receipt.candidate_c114[2][1]);
    }
    if(staged_iq){
        printf("{\"type\":\"hydra_staged_iq_runtime\","
               "\"staged\":true,\"mode\":\"host_preloaded_cyclic_n9_phy_setpoint\","
               "\"dac_fifo_verified\":false,\"usb_bulk_streaming\":false,"
               "\"requested_period_us\":%u,\"iq_amplitude\":%u,"
               "\"duration_us\":20000,\"sample_count\":%u,"
               "\"completed_updates\":%u,\"overrun_events\":%u,"
               "\"maximum_observed_service_delta_us\":%u,"
               "\"final_cyclic_index\":%u,\"control_off\":%s,"
               "\"n9_first_emit_us\":%u,\"n9_last_emit_us\":%u,"
               "\"n9_elapsed_us\":%u,\"n9_mean_updates_per_second\":%.2f,"
               "\"sample_interval_jitter_unqualified\":true,"
               "\"actual_sample_rate_verified\":false}\n",
               iq_period_us,iq_amp,staged_receipt.sample_count,
               staged_receipt.completed_updates,staged_receipt.overruns,
               staged_receipt.max_update_delta_us,staged_receipt.final_index,
               staged_receipt.iq_control_after?"false":"true",
               staged_receipt.n9_first_emit_us,staged_receipt.n9_last_emit_us,
               staged_receipt.n9_span_us,
               staged_receipt.n9_span_us?
                  1.0e6*(double)(staged_receipt.completed_updates-1)/
                             (double)staged_receipt.n9_span_us:0.0);
    }
    if(staged_iq){
        printf("{\"type\":\"hydra_staged_iq_pattern\",\"pattern\":\"%s\","
               "\"seed\":%u,\"cyclic_table\":true,"
               "\"host_streamed_during_rf\":false,\"dac_sample_clock_verified\":false}\n",
               prbs_qpsk?"prbs_qpsk":"rotating_tone",
               prbs_qpsk?UINT32_C(0x6d4a7951):0u);
    }
    if(staged_iq && !tx_rc && !dry_run){
        for(unsigned k=0;k<2;k++){
            const struct hydra_tx_active_snapshot *a=&staged_receipt.active[k];
            printf("{\"type\":\"hydra_n9_active_frontend\",\"slot\":%u,"
                   "\"sequence\":%u,\"begin_us\":%u,\"end_us\":%u,"
                   "\"emission\":%u,\"raw_only\":true,\"atomic\":false,\"words\":{",
                   k,a->sequence,a->begin_us,a->end_us,a->emission);
            for(unsigned j=0;j<HYDRA_TX_ACTIVE_REG_COUNT;j++)
                printf("%s\"%s\":%u",j?",":"",hydra_tx_active_reg_name(j),a->words[j]);
            printf("}}\n");
        }
    }
    if(!rx_fe_pre && !rx_fe_post){
        printf("{\"type\":\"hydra_rx_frontend_snapshot\",\"mmio_equal\":%s,"
               "\"adie_equal\":%s,\"mmio\":[",
               !memcmp(rfe_before.phy_mmio,rfe_after.phy_mmio,
                       sizeof(rfe_before.phy_mmio))?"true":"false",
               !memcmp(rfe_before.adie_gain_raw,rfe_after.adie_gain_raw,
                       sizeof(rfe_before.adie_gain_raw))?"true":"false");
        for(unsigned i=0;i<HYDRA_RX_FE_MMIO_COUNT;i++)
            printf("%s{\"reg\":\"%s\",\"before\":%u,\"after\":%u}",
                   i?",":"",hydra_rx_fe_reg_name(i),
                   rfe_before.phy_mmio[i],rfe_after.phy_mmio[i]);
        printf("],\"analog_port0\":[%u,%u],\"analog_port1\":[%u,%u]}\n",
               rfe_before.adie_gain_raw[0],rfe_after.adie_gain_raw[0],
               rfe_before.adie_gain_raw[1],rfe_after.adie_gain_raw[1]);
    }
    printf("{\"type\":\"hydra_rx_chain_power\",\"rx_lanes\":%u,"
           "\"manual_gain_profile\":\"%s\",\"raw_gain_units_only\":true,"
           "\"stages\":[",
           e.rx_lanes,getenv("HYDRA_GAIN_PROFILE")?getenv("HYDRA_GAIN_PROFILE"):"driver_default");
    for(unsigned ph=0;ph<3;ph++){
        printf("%s{\"stage\":%u,\"lanes\":[",ph?",":"",ph);
        for(unsigned lane=0;lane<e.rx_lanes;lane++){
            const struct power_stats *stp=&e.lane_stats[ph][lane];
            printf("%s{\"lane\":%u,\"words\":%" PRIu64 ","
                   "\"energy\":%" PRIu64 ",\"peak\":%u,\"clipped\":%" PRIu64 "}",
                   lane?",":"",lane,stp->words,stp->energy,stp->peak,stp->clips);
        }
        printf("]}");
    }
    printf("]}\n");
    printf("{\"type\":\"hydra_rx_capture_phases\",\"phase_words\":[%zu,%zu,%zu],"
           "\"first_offsets\":[%" PRIu64 ",%" PRIu64 ",%" PRIu64 "],"
           "\"capture_sample_exact_tx_timestamps\":false}\n",
           e.captured[0],e.captured[1],e.captured[2],
           e.first_offset[0],e.first_offset[1],e.first_offset[2]);
output:
    if(opened)(void)hydrasdr_close(e.dev);
    for(unsigned j=0;j<3;j++)free(e.capture[j]);
    return rc?1:0;
}
