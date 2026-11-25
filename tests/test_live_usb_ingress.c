/* SPDX-License-Identifier: MIT
 * Manual no-RF USB OUT -> MCU receive DMA proof and N9 sequence audit.
 * Same handle as dual RX; optional ring setup, no RF TX register writes.
 * Not part of CTest. Every session closes through native WFSYS reset.
 */
#define _POSIX_C_SOURCE 200809L
#include "internal.h"
#include "hydra_tx_control.h"
#include "ingress_iq_payload.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>

#define LIVE_RX_CAPTURE_WORDS 2000000u
struct context {
    hydrasdr_dev_t *dev;
    atomic_uint_fast64_t words;
    atomic_uint phase;
    int16_t *capture[3];
    size_t captured[3];
    int stream_rc;
};
/* Optional guarded USB-fed RF test captures actual RX samples before, during
 * and after the host PHY GO epoch. The raw files NEVER enter Git. */
static void ingress_live_phase(void *arg,int active){
    struct context *c=arg;
    atomic_store(&c->phase,active?1u:2u);
}
static int save_live_capture(const char *prefix,const struct context *c,unsigned stage){
    if(!prefix || !*prefix || !c || stage>2 || !c->capture[stage])return -1;
    static const char *names[]={"pre","tx","post"};
    char path[1024];
    int n=snprintf(path,sizeof(path),"%s.%s.cs16",prefix,names[stage]);
    if(n<0 || (size_t)n>=sizeof(path))return -1;
    int fd=open(path,O_WRONLY|O_CREAT|O_EXCL|O_NOFOLLOW,0600);
    if(fd<0)return -1;
    size_t done=0,want=c->captured[stage]*4u;
    const uint8_t *data=(const uint8_t *)c->capture[stage];
    int err=0;
    while(done<want){
        ssize_t k=write(fd,data+done,want-done);
        if(k<=0){err=-1;break;}
        done+=(size_t)k;
    }
    if(fsync(fd))err=-1;
    if(close(fd))err=-1;
    if(err)unlink(path);
    return err;
}
static void samples_cb(const int16_t *iq,uint32_t n,uint64_t offset,void *arg)
{
    (void)offset;
    struct context *c=arg;
    if(n && iq){
        if(c->capture[0]){
            unsigned phase=atomic_load(&c->phase);
            if(phase>2)phase=2;
            size_t filled=c->captured[phase];
            if(filled<LIVE_RX_CAPTURE_WORDS){
                size_t take=(size_t)n;
                if(take>LIVE_RX_CAPTURE_WORDS-filled)
                    take=LIVE_RX_CAPTURE_WORDS-filled;
                memcpy(c->capture[phase]+2u*filled,iq,take*4u);
                c->captured[phase]+=take;
            }
        }
        atomic_fetch_add(&c->words,n);
    }
}
static void *receiver(void *arg)
{
    struct context *c=arg;
    c->stream_rc=hydrasdr_read_async(c->dev,samples_cb,c,0,0);
    return NULL;
}
static void sleep_ms(int ms)
{
    struct timespec ts={ms/1000,(ms%1000)*1000000L};
    nanosleep(&ts,NULL);
}
/* Optional accelerated, RF-disabled USB OUT pipeline. One HIF frame per
 * transfer; only the *host scheduling* changes. Reuse the exact on-wire
 * representation and preserve strictly increasing frame indices. This probe
 * is off unless explicitly enabled and MUST NOT run against a live PHY
 * consumer, because there is not yet a source-verified 5MS/s TX sink. */
#define INGRESS_FRAME_BYTES 348u
#define INGRESS_ASYNC_MAX_DEPTH 8u /* 16 pending exposed a nonretiring cancel */
struct ingress_async_context;
struct ingress_async_slot {
    struct ingress_async_context *owner;
    struct libusb_transfer *transfer;
    uint8_t frame[INGRESS_FRAME_BYTES];
    uint32_t frame_id;
    unsigned length;
    bool pending, finished;
};
struct ingress_async_context {
    hydrasdr_dev_t *dev;
    struct ingress_async_slot slots[INGRESS_ASYNC_MAX_DEPTH];
    /* A separate dual-RX thread also calls libusb_handle_events(). libusb
     * may invoke OUT callbacks on that thread, so these counters/slot
     * booleans must not be accessed without this lock. */
    pthread_mutex_t lock;
    pthread_t submitter;
    uint32_t callbacks_on_rx_thread, callbacks_on_submit_thread;
    uint32_t completed, submitted, inflight;
    uint32_t callback_errors;
    int first_error_status, first_error_length;
    uint32_t first_error_frame;
    int failure;
    bool stopping;
    uint64_t initial_ns, last_callback_ns, max_callback_gap_ns;
};

static unsigned ingress_build_frame(uint8_t frame[INGRESS_FRAME_BYTES],
                                    bool stream, bool iq_payload, bool corrupt,
                                    bool rf_low_gain_qpsk,
                                    uint64_t packet, uint64_t sample,
                                    uint32_t nonce)
{
    memset(frame,0,INGRESS_FRAME_BYTES);
    unsigned body=stream?(iq_payload?340u:336u):128u,total=body+8;
    bs_put_le32(frame,body);
    bs_put_le32(frame+4,0x41000000u|body);
    bs_put_le32(frame+8,0x80010000u);
    bs_put_le16(frame+36,(uint16_t)(body-32));bs_put_le16(frame+38,0x8000);
    frame[40]=0;frame[41]=0xa0;frame[42]=3;frame[43]=1;
    if(stream){
        bs_put_le32(frame+68,0x48595131u);
        bs_put_le32(frame+72,nonce);
        bs_put_le32(frame+76,(uint32_t)packet);
        bs_put_le32(frame+80,(uint32_t)(packet>>32));
        uint32_t digest=HYDRA_INGRESS_IQ_CHECKSUM_SEED;
        for(unsigned i=0;i<64;i++){
            uint32_t word=(uint32_t)(sample+i);
            if(iq_payload){
                word=rf_low_gain_qpsk?
                    hydra_ingress_rf_qpsk_word(sample+i,packet,nonce):
                    hydra_ingress_iq_word(sample+i,packet,nonce);
                digest=hydra_ingress_iq_checksum_step(digest,word);
            }
            bs_put_le32(frame+84+i*4,word);
        }
        if(iq_payload)bs_put_le32(frame+84+64*4,digest);
        if(corrupt)frame[84+17*4]^=1;
    }else for(unsigned i=0;i<16;i++)
        bs_put_le32(frame+68+i*4,0x51490000u+i);
    return total;
}

static void LIBUSB_CALL ingress_async_callback(struct libusb_transfer *transfer)
{
    struct ingress_async_slot *slot=transfer->user_data;
    struct ingress_async_context *c=slot->owner;
    pthread_mutex_lock(&c->lock);
    slot->pending=false;
    slot->finished=true;
    if(c->inflight)c->inflight--;
    else c->failure=HYDRASDR_ERROR_STATE;
    if(pthread_equal(pthread_self(),c->submitter))c->callbacks_on_submit_thread++;
    else c->callbacks_on_rx_thread++;
    if(!c->stopping){
        uint64_t now=bs_now_ns();
        if(c->last_callback_ns){
            uint64_t gap=now-c->last_callback_ns;
            if(gap>c->max_callback_gap_ns)c->max_callback_gap_ns=gap;
        }
        c->last_callback_ns=now;
        if(transfer->status!=LIBUSB_TRANSFER_COMPLETED ||
           transfer->actual_length!=(int)slot->length){
            if(!c->callback_errors){
                c->first_error_status=(int)transfer->status;
                c->first_error_length=transfer->actual_length;
                c->first_error_frame=slot->frame_id;
            }
            c->callback_errors++;
            c->failure=HYDRASDR_ERROR_IO;
        }else c->completed++;
    }
    pthread_mutex_unlock(&c->lock);
}

static int ingress_async_submit(struct ingress_async_context *c,
                                unsigned slot_index, uint32_t frame_id,
                                uint64_t packet_start, uint64_t sample_start)
{
    struct ingress_async_slot *s=&c->slots[slot_index];
    pthread_mutex_lock(&c->lock);
    if(s->pending){pthread_mutex_unlock(&c->lock);return HYDRASDR_ERROR_STATE;}
    s->finished=false;
    s->pending=true;
    c->submitted++;
    c->inflight++;
    pthread_mutex_unlock(&c->lock);
    s->frame_id=frame_id;
    s->length=ingress_build_frame(s->frame,true,true,false,false,
                                  packet_start+frame_id,
                                  sample_start+(uint64_t)frame_id*64u,c->dev->nonce);
    libusb_fill_bulk_transfer(s->transfer,c->dev->handle,c->dev->ep_command,
                              s->frame,(int)s->length,ingress_async_callback,s,500);
    int rc=libusb_submit_transfer(s->transfer);
    if(rc){
        /* Failed submissions have no libusb callback to retire ownership. */
        pthread_mutex_lock(&c->lock);
        s->pending=false;
        s->finished=true;
        c->submitted--;
        c->inflight--;
        pthread_mutex_unlock(&c->lock);
        return HYDRASDR_ERROR_IO;
    }
    return 0;
}

/* No other caller can control the USB OUT endpoint during this probe. The RX
 * reader owns USB bulk IN and uses libusb's independent event pump; libusb
 * serializes event dispatch on the shared context. This function does NOT
 * change the proven four-descriptor N9 RX1 hardware ring. */
static int ingress_async_out(hydrasdr_dev_t *dev, uint32_t frame_count,
                             uint64_t packet_start, uint64_t sample_start,
                             unsigned depth, uint32_t *submitted)
{
    if(!depth || depth>INGRESS_ASYNC_MAX_DEPTH || !frame_count)
        return HYDRASDR_ERROR_INVALID;
    struct ingress_async_context *c=calloc(1,sizeof(*c));
    if(!c)return HYDRASDR_ERROR_NO_MEMORY;
    c->dev=dev;
    if(pthread_mutex_init(&c->lock,NULL)){free(c);return HYDRASDR_ERROR_IO;}
    c->submitter=pthread_self();
    int rc=0;
    for(unsigned i=0;i<depth;i++){
        struct ingress_async_slot *s=&c->slots[i];
        s->owner=c;
        s->transfer=libusb_alloc_transfer(0);
        if(!s->transfer){rc=HYDRASDR_ERROR_NO_MEMORY;goto cleanup;}
    }
    uint32_t next=0;
    for(unsigned i=0;i<depth && next<frame_count;i++,next++){
        rc=ingress_async_submit(c,i,next,packet_start,sample_start);
        if(rc)goto cleanup;
    }
    c->initial_ns=bs_now_ns();
    for(;;){
        pthread_mutex_lock(&c->lock);
        const bool stop_now=c->failure || c->completed>=frame_count;
        pthread_mutex_unlock(&c->lock);
        if(stop_now)break;
        struct timeval tv={0,10000};
        int er=libusb_handle_events_timeout_completed(dev->usb,&tv,NULL);
        if(er<0 && er!=LIBUSB_ERROR_INTERRUPTED){rc=HYDRASDR_ERROR_IO;break;}
        /* Never submit frame N+1 before frame N: re-arm only the slot that
         * originally carried (N-depth), even if libusb callbacks race. */
        while(next<frame_count){
            struct ingress_async_slot *slot=&c->slots[next%depth];
            pthread_mutex_lock(&c->lock);
            const bool ready=slot->finished && !slot->pending;
            const bool failed=c->failure!=0;
            pthread_mutex_unlock(&c->lock);
            /* Do not enqueue another HIF frame after a concurrent callback
             * has already marked the transfer session as failed. */
            if(failed || !ready)break;
            rc=ingress_async_submit(c,next%depth,next,packet_start,sample_start);
            if(rc)break;
            next++;
        }
        if(rc)break;
        /* Extended endurance is explicitly opt-in and only armed by the
         * no-RF experiment with two extra N9 RX1 service opportunities.
         * The old 22s timeout remains for all ordinary/manual tests. */
        const uint64_t max_run_ns=frame_count>1000000u ?
            UINT64_C(90000000000) : UINT64_C(22000000000);
        if(bs_now_ns()-c->initial_ns>max_run_ns){
            rc=HYDRASDR_ERROR_TIMEOUT;break;
        }
    }
    if(!rc){
        pthread_mutex_lock(&c->lock);
        rc=c->failure;
        pthread_mutex_unlock(&c->lock);
    }
cleanup:
    pthread_mutex_lock(&c->lock);
    c->stopping=true;
    pthread_mutex_unlock(&c->lock);
    for(unsigned i=0;i<depth;i++){
        pthread_mutex_lock(&c->lock);
        const bool pending=c->slots[i].pending;
        pthread_mutex_unlock(&c->lock);
        if(pending)libusb_cancel_transfer(c->slots[i].transfer);
    }
    /* Mandatory: no libusb buffer may be released until its completion or
     * cancellation callback has retired. Force WFSYS reset on any fault. */
    uint64_t stop=bs_now_ns();
    for(;;){
        pthread_mutex_lock(&c->lock);
        const uint32_t outstanding=c->inflight;
        pthread_mutex_unlock(&c->lock);
        if(!outstanding)break;
        struct timeval tv={0,10000};
        int er=libusb_handle_events_timeout_completed(dev->usb,&tv,NULL);
        if(er<0 && er!=LIBUSB_ERROR_INTERRUPTED && !rc)rc=HYDRASDR_ERROR_IO;
        if(bs_now_ns()-stop>UINT64_C(3000000000)){
            /* Avoid use-after-free: never release slots with USB ownership.
             * Closing the process is safer than a dangling libusb callback. */
            fprintf(stderr,"ASYNC_INGRESS_FATAL pending=%u; fail closed\n",c->inflight);
            fflush(stderr);
            _Exit(111);
        }
    }
    *submitted=c->completed;
    printf("INGRESS_ASYNC depth=%u submitted=%u completed=%u failures=%u "
           "max_callback_gap_us=%.3f callbacks_submit_thread=%u "
           "callbacks_rx_thread=%u first_error_status=%d first_error_frame=%u "
           "first_error_actual=%d rc=%d\n",depth,c->submitted,c->completed,
           c->callback_errors,(double)c->max_callback_gap_ns/1000.,
           c->callbacks_on_submit_thread,c->callbacks_on_rx_thread,
           c->first_error_status,c->first_error_frame,c->first_error_length,rc);
    if(rc)dev->needs_usb_reset=true;
    for(unsigned i=0;i<depth;i++)libusb_free_transfer(c->slots[i].transfer);
    pthread_mutex_destroy(&c->lock);
    free(c);
    return rc;
}

/* The finite R2 RF-on producer uses ONLY the existing bulk OUT endpoint;
 * the single control owner already holds the transaction mutex in the
 * watchdog-gated hydra_tx_run_bounded_usb_iq_locked routine. 3 packets are
 * DMA-staged while RF is OFF; the remaining 9 arrive after the GO epoch.
 * This is a 768-word/20us (~50k setpoint writes/s), 20ms MAX experiment,
 * NOT arbitrary 5 MS/s DAC streaming or an automatic test. */
struct ingress_live_send_ctx {
    hydrasdr_dev_t *dev;
    uint64_t packet,sample;
    unsigned frames,completed;
    uint64_t begin_ns,end_ns;
};
static int ingress_send_live_frames(void *arg){
    struct ingress_live_send_ctx *c=arg;
    c->begin_ns=bs_now_ns();
    for(unsigned j=0;j<c->frames;j++){
        uint8_t frame[INGRESS_FRAME_BYTES];
        unsigned len=ingress_build_frame(frame,true,true,false,true,
                                  c->packet+j,c->sample+(uint64_t)j*64u,
                                  c->dev->nonce);
        int actual=0;
        int er=libusb_bulk_transfer(c->dev->handle,c->dev->ep_command,
                                    frame,(int)len,&actual,10);
        if(er || actual!=(int)len)return HYDRASDR_ERROR_IO;
        c->completed++;
    }
    c->end_ns=bs_now_ns();
    return 0;
}

/* Exact Fenvi ROM 0x82ea70 initializes RX descriptors with buffer pointer,
 * LS0 bit30, capacity[29:16], DDONE bit31 clear; 0x82e446 recycles them.
 * ROM register table 0x85111c identifies RX1 base 0x54000510. Native N9
 * owns TX0 only and masks DMA interrupts. Use one packet, no ROM handler. */
static int ingress_probe(hydrasdr_dev_t *dev,struct context *rx_context) {
    /* Reject invalid stream modes BEFORE modifying the RX DMA engine. */
    const bool request_iq=getenv("HYDRA_TEST_USB_INGRESS_IQ") &&
        !strcmp(getenv("HYDRA_TEST_USB_INGRESS_IQ"),"1");
    const bool request_stream=getenv("HYDRA_TEST_USB_INGRESS_STREAM") &&
        !strcmp(getenv("HYDRA_TEST_USB_INGRESS_STREAM"),"1");
    const bool request_consume=getenv("HYDRA_TEST_USB_INGRESS_CONSUME") &&
        !strcmp(getenv("HYDRA_TEST_USB_INGRESS_CONSUME"),"1");
    const bool request_paced=getenv("HYDRA_TEST_USB_INGRESS_PACED") &&
        !strcmp(getenv("HYDRA_TEST_USB_INGRESS_PACED"),"1");
    const bool request_corrupt=getenv("HYDRA_TEST_USB_INGRESS_CORRUPT") &&
        !strcmp(getenv("HYDRA_TEST_USB_INGRESS_CORRUPT"),"1");
    const bool request_wrap=getenv("HYDRA_TEST_USB_INGRESS_WRAP") &&
        !strcmp(getenv("HYDRA_TEST_USB_INGRESS_WRAP"),"1");
    const bool request_starve=getenv("HYDRA_TEST_USB_INGRESS_STARVE") &&
        !strcmp(getenv("HYDRA_TEST_USB_INGRESS_STARVE"),"1");
    const bool request_deadline=getenv("HYDRA_TEST_USB_INGRESS_ABS_DEADLINE") &&
        !strcmp(getenv("HYDRA_TEST_USB_INGRESS_ABS_DEADLINE"),"1");
    const bool request_fill_ahead=getenv("HYDRA_TEST_USB_INGRESS_FILL_AHEAD") &&
        !strcmp(getenv("HYDRA_TEST_USB_INGRESS_FILL_AHEAD"),"1");
    const bool request_phy_dry=getenv("HYDRA_TEST_USB_INGRESS_PHY_DRY") &&
        !strcmp(getenv("HYDRA_TEST_USB_INGRESS_PHY_DRY"),"1");
    /* R2 new exact-source N9 mode3 DRY=2: stage, check and consume fresh
     * host IQ while TX GO/path/WD are strictly OFF. This is a NO-RF gate,
     * not automatically promoted to RF-enabled or USB DAC streaming. */
    const bool request_bridge_dry=getenv("HYDRA_TEST_USB_INGRESS_RF_BRIDGE_DRY") &&
        !strcmp(getenv("HYDRA_TEST_USB_INGRESS_RF_BRIDGE_DRY"),"1");
    const bool request_rf_live=getenv("HYDRA_TEST_USB_INGRESS_RF_LIVE") &&
        !strcmp(getenv("HYDRA_TEST_USB_INGRESS_RF_LIVE"),"1");
    unsigned live_period_us=20u;
    const char *live_period_env=getenv("HYDRA_TEST_USB_INGRESS_RF_PERIOD_US");
    if(live_period_env){
        if(!request_rf_live)return HYDRASDR_ERROR_INVALID;
        if(!strcmp(live_period_env,"20"))live_period_us=20u;
        else if(!strcmp(live_period_env,"10"))live_period_us=10u;
        else if(!strcmp(live_period_env,"5"))live_period_us=5u;
        else if(!strcmp(live_period_env,"2"))live_period_us=2u;
        else return HYDRASDR_ERROR_INVALID;
    }
    const bool request_abort=getenv("HYDRA_TEST_USB_INGRESS_ABORT") &&
        !strcmp(getenv("HYDRA_TEST_USB_INGRESS_ABORT"),"1");
    /* Explicit no-RF scheduling experiment: the original N9 publish loop
     * validates exactly one HIF frame per 8192B RX capture packet. Test one
     * bounded extra descriptor service without replacing RX or its DMA ring. */
    const char *extra_env=getenv("HYDRA_TEST_USB_INGRESS_EXTRA_SERVICE");
    if(extra_env && strcmp(extra_env,"1") && strcmp(extra_env,"2") &&
       strcmp(extra_env,"3"))return HYDRASDR_ERROR_INVALID;
    const unsigned request_extra=extra_env?(unsigned)(extra_env[0]-'0'):0u;
    const char *async_env=getenv("HYDRA_TEST_USB_INGRESS_ASYNC_DEPTH");
    unsigned async_depth=0;
    if(async_env){
        char *end=NULL;
        unsigned long v=strtoul(async_env,&end,10);
        if(!*async_env || !end || *end || v<1 || v>INGRESS_ASYNC_MAX_DEPTH)
            return HYDRASDR_ERROR_INVALID;
        async_depth=(unsigned)v;
    }
    /* First generation is strictly accelerated + no-RF + no-consumer.
     * Do not use it to fill an unqualified continuous RF or PHY sink. */
    if(async_depth && (!request_iq || !request_stream || request_consume ||
                       request_paced || request_corrupt || request_wrap ||
                       request_starve || request_deadline || request_fill_ahead ||
                       request_phy_dry || request_abort))
        return HYDRASDR_ERROR_INVALID;
    if((request_rf_live && (!request_consume || !request_iq || !request_stream ||
                            !request_paced || request_phy_dry || request_bridge_dry ||
                            request_corrupt || request_starve || request_abort ||
                            request_wrap || request_deadline || async_depth)) ||
       (request_bridge_dry && (!request_consume || !request_iq || !request_stream ||
                                  !request_paced || request_phy_dry ||
                                  request_corrupt || request_starve || request_abort ||
                                  request_wrap || request_deadline || async_depth)) ||
       (request_extra && (!async_depth || !request_iq || !request_stream ||
                           request_consume || request_phy_dry || request_abort)) ||
       (request_iq && !request_stream) ||
       (request_consume && (!request_iq || !request_paced || request_corrupt)) ||
       (request_starve && (!request_consume || request_wrap)) ||
       (request_deadline && (!request_consume || request_starve || request_wrap)) ||
       (request_fill_ahead && (!request_consume || request_starve || request_wrap)) ||
       (request_phy_dry && (!request_consume || !request_deadline ||
                            !request_fill_ahead || request_corrupt ||
                            request_starve || request_wrap)) ||
       (request_abort && (!request_phy_dry || !request_consume)))
        return HYDRASDR_ERROR_INVALID;
    const uint32_t ctl=0x54000208u, rb=0x54000510u;
    const uint32_t desc=0x0203c800u, payload=0x0203d000u;
    const unsigned rx1_slots=4u; /* proven physical DMA ring capacity */
    uint32_t phy_iq_before=0;
    bool phy_iq_saved=false;
    if(request_phy_dry){
        /* A separate manual non-RF test. Never enable tone GO or TX path.
         * Return the PHY IQ setpoint to its exact initial value even when
         * USB/DMA/consumer validation fails. */
        int before_rc=bs_read32(dev,BS_MMIO,0x8308fc08u,&phy_iq_before);
        if(before_rc)return before_rc;
        phy_iq_saved=true;
        uint32_t tone=0,path=0,power=0,watchdog=UINT32_MAX;
        BS_TRY(bs_read32(dev,BS_MMIO,0x8308fc00u,&tone));
        BS_TRY(bs_read32(dev,BS_MMIO,0x8308fc20u,&path));
        BS_TRY(bs_read32(dev,BS_MMIO,0x8300c508u,&power));
        BS_TRY(bs_read32(dev,BS_MMIO,0x02038360u+192u,&watchdog));
        if((tone&0x40000000u) || path || (power&0x80008000u) || watchdog)
            return HYDRASDR_ERROR_STATE;
    }
    uint32_t saved[3]={0},global=0,didx=0,done=0;
    uint32_t caps=0,abi=0;
    int rc=mt_diag_transact_locked(dev,1,0,&abi);
    if(!rc)rc=mt_diag_transact_locked(dev,4,0,&caps);
    if(rc)return rc;
    if(abi!=HYDRASDR_DIAG_ABI_MAGIC || !(caps&HYDRASDR_DIAG_CAP_USB_INGRESS_AUDIT) ||
       (request_consume && !(caps&HYDRASDR_DIAG_CAP_USB_INGRESS_PACED)) ||
       (request_phy_dry && !(caps&HYDRASDR_DIAG_CAP_USB_INGRESS_PHY_DRY)))
        return HYDRASDR_ERROR_UNSUPPORTED;
    rc=bs_read32(dev,BS_MMIO,ctl,&global);
    if(rc)return rc;
    if(global&12u)return HYDRASDR_ERROR_BUSY;
    for(unsigned j=0;j<3;j++){
        rc=bs_read32(dev,BS_MMIO,rb+4*j,&saved[j]);if(rc)return rc;
    }
    rc=bs_read32(dev,BS_MMIO,rb+12,&didx);if(rc)return rc;
    if(saved[1]!=4 || didx>=4)return HYDRASDR_ERROR_STATE;
    for(unsigned i=0;i<rx1_slots;i++){
        for(unsigned j=0;j<128;j++){
            rc=bs_write32(dev,BS_MMIO,payload+i*512+j*4,0xa5a5a5a5u);
            if(rc)return rc;
        }
        uint32_t words[4]={payload+i*512,0x42000000u,0,0};
        for(unsigned j=0;j<4;j++){
            uint32_t got=0;
            rc=bs_write32(dev,BS_MMIO,desc+i*16+j*4,words[j]);if(rc)return rc;
            rc=bs_read32(dev,BS_MMIO,desc+i*16+j*4,&got);if(rc)return rc;
            if(got!=words[j])return HYDRASDR_ERROR_STATE;
        }
    }
    rc=bs_write32(dev,BS_MMIO,rb,desc);
    /* Bounded candidate from MT792x WFDMA RST_DRX_PTR offset 0x280.
     * Require observed RX1 DIDX reset; never reset the active TX0 ring. */
    if(!rc)rc=bs_write32(dev,BS_MMIO,0x54000280u,2);
    if(!rc)rc=bs_read32(dev,BS_MMIO,rb+12,&didx);
    printf("RX1_RESET_DIDX %u rc=%d\n",didx,rc);
    if(!rc && didx)rc=HYDRASDR_ERROR_STATE;
    if(!rc)rc=bs_write32(dev,BS_MMIO,rb+8,rx1_slots-1u);
    if(!rc)rc=bs_update32(dev,BS_MMIO,ctl,0,4);
    const bool stream=getenv("HYDRA_TEST_USB_INGRESS_STREAM") &&
        !strcmp(getenv("HYDRA_TEST_USB_INGRESS_STREAM"),"1");
    const bool wrap=getenv("HYDRA_TEST_USB_INGRESS_WRAP") &&
        !strcmp(getenv("HYDRA_TEST_USB_INGRESS_WRAP"),"1");
    const bool corrupt=getenv("HYDRA_TEST_USB_INGRESS_CORRUPT") &&
        !strcmp(getenv("HYDRA_TEST_USB_INGRESS_CORRUPT"),"1");
    const bool paced=getenv("HYDRA_TEST_USB_INGRESS_PACED") &&
        !strcmp(getenv("HYDRA_TEST_USB_INGRESS_PACED"),"1");
    const bool iq_payload=getenv("HYDRA_TEST_USB_INGRESS_IQ") &&
        !strcmp(getenv("HYDRA_TEST_USB_INGRESS_IQ"),"1");
    const bool consume=request_consume;
    const bool starve=request_starve;
    const bool short_consume=consume && getenv("HYDRA_TEST_USB_INGRESS_SHORT") &&
        !strcmp(getenv("HYDRA_TEST_USB_INGRESS_SHORT"),"1");
    /* Consumer SHORT: 128 x 64 = 8192 new samples at 100k/s (~82ms).
     * Regular paced target: 2 million in ~20 seconds with both RX active. */
    /* A short 8-packet wrap test can also exercise the paced software
     * consumer with both 64-bit packet/sample counters near rollover. */
    /* Opt-in longer RF-OFF stress only when the bounded N9 extra-service
     * mode and accelerated HIF OUT test are both enabled. No new RF control,
     * no change to the four hardware RX1 descriptors. Existing defaults stay. */
    uint32_t count=request_rf_live?12u:
        stream?(corrupt?1u:wrap?8u:short_consume?128u:paced?31250u:10000u):1u;
    const char *long_env=getenv("HYDRA_TEST_USB_INGRESS_FRAME_COUNT");
    if(long_env){
        if(!request_extra || !async_depth || request_consume || !request_iq ||
           !request_stream || corrupt || wrap || starve || request_phy_dry)
            return HYDRASDR_ERROR_INVALID;
        char *end=NULL;
        errno=0;
        unsigned long v=strtoul(long_env,&end,10);
        if(errno || !*long_env || !end || *end || v<10000u || v>5000000u ||
           (v>1000000u && ((request_extra!=2u && request_extra!=3u) || async_depth!=8u)))
            return HYDRASDR_ERROR_INVALID;
        count=(uint32_t)v;
    }
    uint64_t packet=wrap?UINT64_C(0x7fffffffd):0;
    uint64_t sample=wrap?UINT64_C(0x9ffffff40):0;
    const uint64_t packet_start=packet,sample_start=sample;
    if(stream && !rc){
        uint32_t caps=0;
        rc=mt_diag_transact_locked(dev,4,0,&caps);
        if(!rc && (!(caps&HYDRASDR_DIAG_CAP_USB_INGRESS_AUDIT) ||
                   (consume && !(caps&HYDRASDR_DIAG_CAP_USB_INGRESS_PACED)) ||
                   (request_phy_dry && !(caps&HYDRASDR_DIAG_CAP_USB_INGRESS_PHY_DRY))))
            rc=HYDRASDR_ERROR_UNSUPPORTED;
        const uint32_t v[]={0,0,0,(uint32_t)packet,(uint32_t)(packet>>32),
                            (uint32_t)sample,(uint32_t)(sample>>32),0,0,0};
        for(unsigned i=0;i<10 && !rc;i++)rc=bs_write32(dev,BS_MMIO,0x02038360u+240+4*i,v[i]);
        /* All offsets are volatile N9 mailbox; verify explicit values before
         * handing the RX1 descriptor ring to DMA. No TX DAC or PA writes. */
        const uint32_t p=0x02038360u;
        const uint32_t values[]={
             0,request_rf_live?live_period_us:10u,0,0,0,0,0,0,0,0,0,0,
             (uint32_t)(sample+(uint64_t)count*64u),
             (uint32_t)((sample+(uint64_t)count*64u)>>32),
             0,0,0,0,0,(uint32_t)packet,(uint32_t)(packet>>32),
             (uint32_t)sample,(uint32_t)(sample>>32),0,0,0
        };
        /* +292..+392: consumer setup and producer/consumer stage-ring
         * indices, fill/highwater, validated packet/sample counters and
         * N9 copy scratch. Host never updates any field after GO. */
        for(unsigned i=0;i<sizeof(values)/sizeof(values[0]) && !rc;i++){
            uint32_t offset=292u+i*4u,got=UINT32_MAX;
            rc=bs_write32(dev,BS_MMIO,p+offset,values[i]);
            if(!rc)rc=bs_read32(dev,BS_MMIO,p+offset,&got);
            if(!rc && got!=values[i])rc=HYDRASDR_ERROR_STATE;
        }
        if(!rc){
            uint32_t observed=UINT32_MAX;
            rc=bs_write32(dev,BS_MMIO,p+396,request_deadline?1u:0u);
            if(!rc)rc=bs_read32(dev,BS_MMIO,p+396,&observed);
            if(!rc && observed!=(request_deadline?1u:0u))rc=HYDRASDR_ERROR_STATE;
        }
        if(!rc)rc=bs_write32(dev,BS_MMIO,p+288,iq_payload?1u:0u);
        if(!rc)rc=bs_write32(dev,BS_MMIO,p+400,0);
        if(!rc)rc=bs_write32(dev,BS_MMIO,p+404,0);
        for(unsigned i=0;i<2 && !rc;i++){
            uint32_t observed=UINT32_MAX;
            rc=bs_write32(dev,BS_MMIO,p+408u+i*4u,0);
            if(!rc)rc=bs_read32(dev,BS_MMIO,p+408u+i*4u,&observed);
            if(!rc && observed)rc=HYDRASDR_ERROR_STATE;
        }
        /* Source-qualified N9 mailbox +416..+440, initialized zero on boot;
         * write all fields with readback before RX1 service can run. */
        for(unsigned i=0;i<7 && !rc;i++){
            uint32_t value=(i==0)?request_extra:0u,got=UINT32_MAX;
            rc=bs_write32(dev,BS_MMIO,p+416u+4u*i,value);
            if(!rc)rc=bs_read32(dev,BS_MMIO,p+416u+4u*i,&got);
            if(!rc && got!=value)rc=HYDRASDR_ERROR_STATE;
        }
        /* New mode3 mailbox is reset and readback-verified BEFORE enable.
         * DRY=2 only. Never arms RF, GO, watchdog or PA. */
        for(unsigned i=0;i<5 && !rc;i++){
            const uint32_t value=(i==0 && request_bridge_dry)?2u:0u;
            uint32_t got=UINT32_MAX;
            rc=bs_write32(dev,BS_MMIO,p+444u+4u*i,value);
            if(!rc)rc=bs_read32(dev,BS_MMIO,p+444u+4u*i,&got);
            if(!rc && got!=value)rc=HYDRASDR_ERROR_STATE;
        }
        if(!rc)rc=bs_write32(dev,BS_MMIO,p+292,
                            (request_bridge_dry || request_rf_live)?3u:
                            request_phy_dry?2u:consume?1u:0u);
        if(!rc)rc=bs_write32(dev,BS_MMIO,p+240,1);
    }
    int actual=0,usb_rc=0;uint32_t submitted=0;
    uint64_t begin=bs_now_ns();
    if(request_rf_live && !rc){
        /* Source-qualified HIF ring: 3 validated packets preloaded while
         * GO/path/WD are OFF, then exactly 9 more frames while TX is ON. */
        for(unsigned n=0;n<3 && !rc;n++){
            uint8_t frame[INGRESS_FRAME_BYTES];
            unsigned len=ingress_build_frame(frame,true,true,false,true,
                                      packet+n,sample+(uint64_t)n*64u,dev->nonce);
            int act=0;
            int er=libusb_bulk_transfer(dev->handle,dev->ep_command,
                                        frame,(int)len,&act,50);
            if(er || act!=(int)len)rc=HYDRASDR_ERROR_IO;
            else submitted++;
        }
        if(!rc)rc=bs_poll32(dev,BS_MMIO,0x02038360u+356u,
                           UINT32_MAX,3u,200u);
        struct ingress_live_send_ctx sender={.dev=dev,.packet=packet+3u,
                             .sample=sample+192u,.frames=9u};
        struct hydra_tx_usb_ingress_receipt rf={0};
        const struct hydra_tx_burst_config cfg={.chain=0,.raw_power_code=0};
        if(!rc)rc=hydra_tx_run_bounded_usb_iq_locked(dev,&cfg,count*64u,20000u,
                    ingress_send_live_frames,&sender,&rf,
                    ingress_live_phase,rx_context);
        submitted+=(uint32_t)sender.completed;
        packet=packet_start+submitted;
        sample=sample_start+(uint64_t)submitted*64u;
        printf("INGRESS_RF_LIVE period_us=%u n9_rf_rate=%.2f "
               "nonce=%08x rf_qpsk_amplitude=64 "
               "host_preloaded_packets=3 live_usb_completed=%u "
               "live_usb_us=%.3f rf_phy_writes=%u n9_consumed_samples=%u "
               "n9_consumed_packets=%u n9_producer_packets=%u "
               "rf_last=%08x fifo_hwm=%u underruns=%u abort_ack=%u "
               "watchdog_status_pre_disarm=%u tx_restored=%u rc=%d\n",
               live_period_us,
               rf.rf_last_us>rf.rf_first_us && rf.rf_setpoint_writes>1u?
                 1e6*(double)(rf.rf_setpoint_writes-1u)/
                 (double)(rf.rf_last_us-rf.rf_first_us):0.,
               dev->nonce,sender.completed,
               (double)(sender.end_ns-sender.begin_ns)/1000.,
               rf.rf_setpoint_writes,rf.consumed_words,rf.consumed_packets,
               rf.producer_packets,rf.rf_last_iq,rf.fifo_highwater,
               rf.consumer_underruns,rf.abort_ack,
               rf.watchdog_state_before_disarm,
               !memcmp(rf.tx.before,rf.tx.after,sizeof(rf.tx.before)),rc);
        if(!rc && (sender.completed!=9u || rf.rf_last_iq!=
                           hydra_ingress_rf_qpsk_word(sample-1u,packet-1u,dev->nonce)))
            rc=HYDRASDR_ERROR_STATE;
    }
    if(async_depth && !rc){
        rc=ingress_async_out(dev,count,packet_start,sample_start,
                             async_depth,&submitted);
        /* Advance expected counters only after all asynchronous OUT
         * callbacks completed, never before. The N9 verifies each frame. */
        packet=packet_start+submitted;
        sample=sample_start+(uint64_t)submitted*64u;
    }
    for(unsigned n=0;n<count && !rc && !async_depth && !request_rf_live;n++){
        uint8_t frame[INGRESS_FRAME_BYTES];
        unsigned total=ingress_build_frame(frame,stream,iq_payload,corrupt,false,
                                           packet,sample,dev->nonce);
        usb_rc=libusb_bulk_transfer(dev->handle,dev->ep_command,frame,(int)total,&actual,500);
        if(usb_rc || actual!=(int)total){rc=HYDRASDR_ERROR_IO;break;}
        submitted++;packet++;sample+=64;
        if(request_abort && submitted==20u){
            /* Negative-only: force a genuine host interruption while fresh
             * PHY setpoint writes are still active. The host must request
             * and acknowledge an N9 abort BEFORE restoring PHY IQ. */
            rc=HYDRASDR_ERROR_STATE;
            break;
        }
        if(starve && submitted==20u){
            /* Deliberate no-RF producer starvation: the finite N9 software
             * queue must run dry, emit an explicit sticky underrun and
             * disable the hardware RX1 DMA engine. Never silently replay. */
            sleep_ms(60);
            break;
        }
        if(paced && !corrupt && !wrap && !request_fill_ahead){
            /* Legacy host OS-clock-pacing. The optional B3 fill-ahead mode
             * instead feeds until DMA ring backpressure, allowing the N9
             * to absorb future host OS USB scheduling stalls. No RF. */
            /* Prime a two+ descriptor cushion before starting the paced
             * producer, then replenish once per 640us packet consumption. */
            uint64_t paced_packet=consume?(submitted>3u?submitted-3u:0u):submitted;
            uint64_t target=begin+paced_packet*640000u;
            struct timespec ts={(time_t)(target/1000000000u),(long)(target%1000000000u)};
            int er;
            do{er=clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&ts,NULL);}while(er==EINTR);
            if(er){rc=HYDRASDR_ERROR_IO;break;}
        }
        if(bs_now_ns()-begin>(short_consume?UINT64_C(3000000000):UINT64_C(22000000000))){
            rc=HYDRASDR_ERROR_TIMEOUT;break;
        }
    }
    if(!stream && !rc)rc=bs_poll32(dev,BS_MMIO,desc+didx*16+4,0x80000000u,0x80000000u,500);
    if(stream && !rc){
        if(starve)rc=bs_poll32(dev,BS_MMIO,0x02038360u+244,UINT32_MAX,2,500);
        else if(corrupt)rc=bs_poll32(dev,BS_MMIO,0x02038360u+244,UINT32_MAX,1,500);
        else rc=bs_poll32(dev,BS_MMIO,0x02038360u+252,UINT32_MAX,(uint32_t)packet,1000);
    }
    uint64_t finish=bs_now_ns();
    if(stream){
        int er=bs_write32(dev,BS_MMIO,0x02038360u+240,0);
        uint32_t ack=0;if(!er)er=mt_diag_transact_locked(dev,4,0,&ack);
        if(!rc)rc=er;
        uint32_t v[9]={0};
        for(unsigned i=0;i<9;i++){
            er=bs_read32(dev,BS_MMIO,0x02038360u+244+4*i,&v[i]);if(!rc)rc=er;
        }
        uint64_t consumed=((uint64_t)v[3]<<32)|v[2];
        uint64_t samples=((uint64_t)v[5]<<32)|v[4];
        uint64_t expected_packets=corrupt?packet_start:packet;
        uint64_t expected_samples=corrupt?sample_start:sample;
        const unsigned expected_fault=starve?2u:(unsigned)corrupt;
        if(!rc && (v[0]!=expected_fault ||
              (starve?(consumed>packet || !consumed || samples>sample || !samples):
                      (consumed!=expected_packets || samples!=expected_samples))))
            rc=HYDRASDR_ERROR_PROTOCOL;
        if(consume){
            uint32_t stats[26]={0};
            for(unsigned i=0;i<26;i++){
                er=bs_read32(dev,BS_MMIO,0x02038360u+292+i*4u,&stats[i]);
                if(!rc)rc=er;
            }
            printf("INGRESS_PACED_CONSUMER mode=%u period_us=%u state=%u "
                   "word_index=%u underruns=%u max_service_delta_us=%u "
                   "first_tick=%u last_tick=%u late_service_events=%u "
                   "expected_sample_count=%llu stage_producer=%u "
                   "stage_consumer=%u stage_fill=%u stage_highwater=%u "
                   "stage_overruns=%u producer_packets=%llu deadline_mode=%u "
                   "fill_ahead=%u phy_dry=%u rc=%d\n",
                   stats[0],stats[1],stats[2],stats[3],stats[5],stats[6],
                   stats[7],stats[8],stats[10],
                   (unsigned long long)(((uint64_t)stats[13]<<32)|stats[12]),
                   stats[14],stats[15],stats[16],stats[17],stats[18],
                   (unsigned long long)(((uint64_t)stats[20]<<32)|stats[19]),
                   request_deadline?1u:0u,request_fill_ahead?1u:0u,
                   request_phy_dry?1u:0u,rc);
            if(!rc && (stats[0]!=((request_bridge_dry || request_rf_live)?3u:
                                    request_phy_dry?2u:1u) ||
                       (starve? (stats[2]!=4u || stats[5]!=1u ||
                                 stats[20]!=0u || stats[19]!=packet):
                                (stats[2]!=(request_rf_live?5u:3u) || stats[5]!=0u ||
                                 stats[3]!=64u || stats[16]!=0u ||
                                 stats[18]!=0u ||
                                 (((uint64_t)stats[20]<<32)|stats[19])!=packet))))
                rc=HYDRASDR_ERROR_STATE;
        }
        if(request_bridge_dry){
            uint32_t diag[5]={0};
            for(unsigned i=0;i<5;i++){
                er=bs_read32(dev,BS_MMIO,0x02038360u+444u+4u*i,&diag[i]);
                if(!rc)rc=er;
            }
            printf("INGRESS_RF_BRIDGE_DRY arm=%u rf_writes=%u last=%08x "
                   "first_tick=%u last_tick=%u rc=%d\n",
                   diag[0],diag[1],diag[2],diag[3],diag[4],rc);
            /* The purpose of this gate is N9 source/data integrity with
             * actual RF absolutely disabled, not even transient PHY writes. */
            if(!rc && (diag[0]!=2u || diag[1]!=0u ||
                       diag[3]!=0u || diag[4]!=0u))rc=HYDRASDR_ERROR_STATE;
        }
        if(request_extra){
            uint32_t diag[7]={0};
            for(unsigned i=0;i<7;i++){
                er=bs_read32(dev,BS_MMIO,0x02038360u+416u+4u*i,&diag[i]);
                if(!rc)rc=er;
            }
            printf("INGRESS_EXTRA_SERVICE enabled=%u attempted=%u accepted=%u "
                   "max_extra_us=%u last_start=%u prior_packet=%u "
                   "budget_remaining=%u rc=%d\n",
                   diag[0],diag[1],diag[2],diag[3],diag[4],diag[5],diag[6],rc);
            if(!rc && (diag[0]!=request_extra || diag[2]>diag[1] ||
                       diag[6]!=0 ||
                       diag[2]>(uint32_t)(samples-sample_start)/64u))
                rc=HYDRASDR_ERROR_STATE;
        }
        printf("INGRESS_STREAM submitted=%u packet_count=%llu sample_count=%llu fault=%u "
               "wrap=%u corrupt=%u host_defined_iq=%u starved=%u elapsed_ns=%llu payload_words_per_s=%.2f "
               "first_tick=%u last_tick=%u max_observed_queued=%u rc=%d\n",submitted,
               (unsigned long long)consumed,(unsigned long long)samples,v[0],wrap,corrupt,
               iq_payload?1u:0u,starve?1u:0u,
               (unsigned long long)(finish-begin),
               (double)(samples-sample_start)*1e9/(double)(finish-begin),v[6],v[7],v[8],rc);
    }
    if(phy_iq_saved){
        /* On an unexpected USB/stream error, or if the N9 has not reached
         * completion state 3, first ask N9 to stop its PHY writer and wait
         * for an explicit acknowledgement. Do not race restoration against
         * a running RF register writer. */
        uint32_t consumer_state=UINT32_MAX,abort_ack=UINT32_MAX;
        int state_rc=bs_read32(dev,BS_MMIO,0x02038360u+300u,&consumer_state);
        int abort_rc=0;
        bool writer_quiescent=!state_rc && consumer_state==3u;
        if(rc || state_rc || consumer_state!=3u){
            abort_rc=bs_write32(dev,BS_MMIO,0x02038360u+408u,1);
            if(!abort_rc)abort_rc=bs_poll32(dev,BS_MMIO,0x02038360u+412u,
                                            UINT32_MAX,1,500);
            if(!abort_rc)abort_rc=bs_read32(dev,BS_MMIO,0x02038360u+412u,&abort_ack);
            writer_quiescent=!abort_rc && abort_ack==1u;
            if(!writer_quiescent){
                dev->needs_usb_reset=true;
                if(!rc)rc=HYDRASDR_ERROR_STATE;
            }
            printf("INGRESS_PHY_DRY_ABORT requested=1 ack=%u state_before=%u rc=%d\n",
                   abort_ack,consumer_state,abort_rc);
        }
        /* Never race a PHY restore against an unacknowledged, possibly live
         * N9 writer. Force WFSYS reset through close on missing ABORT ACK;
         * the generator is disabled and no RF is active. An unsuccessful
         * restore is NOT a successful qualification. */
        if(writer_quiescent){
        /* N9 state=3 or acknowledged abort stops the writer; verify the
         * last emitted PHY setpoint before restoring exact baseline. */
        uint32_t last=UINT32_MAX,writes=0,actual_iq=UINT32_MAX;
        int verify_rc=bs_read32(dev,BS_MMIO,0x02038360u+400u,&last);
        if(!verify_rc)verify_rc=bs_read32(dev,BS_MMIO,0x02038360u+404u,&writes);
        if(!verify_rc)verify_rc=bs_read32(dev,BS_MMIO,0x8308fc08u,&actual_iq);
        if(!verify_rc && !rc){
            const uint32_t expected=hydra_ingress_iq_word(sample-1,packet-1,dev->nonce);
            if(actual_iq!=expected || last!=expected || writes!=count*64u)
                verify_rc=HYDRASDR_ERROR_STATE;
        }
        printf("INGRESS_PHY_DRY expected_last=%08x actual=%08x n9_last=%08x "
               "n9_writes=%u generator_disabled=true rc=%d\n",
               hydra_ingress_iq_word(sample-1,packet-1,dev->nonce),
               actual_iq,last,writes,verify_rc);
        int restore_rc=bs_write32(dev,BS_MMIO,0x8308fc08u,phy_iq_before);
        uint32_t restored=UINT32_MAX;
        if(!restore_rc)restore_rc=bs_read32(dev,BS_MMIO,0x8308fc08u,&restored);
        if(!restore_rc && restored!=phy_iq_before)restore_rc=HYDRASDR_ERROR_STATE;
        if(!rc)rc=verify_rc;
        if(!rc)rc=restore_rc;
        if(restore_rc)dev->needs_usb_reset=true;
        }else{
            printf("INGRESS_PHY_DRY_RESTORE_SKIPPED unsafe_unacknowledged_writer=1\n");
        }
    }
    int off=bs_update32(dev,BS_MMIO,ctl,4,0);
    if(!off)off=bs_poll32(dev,BS_MMIO,ctl,8,0,500);
    if(!rc)rc=off;
    if(!off){
        if(!stream){
        uint32_t hwidx=0;
        int rd=bs_read32(dev,BS_MMIO,rb+12,&hwidx);
        if(!rc)rc=rd;
        if(!rc)rc=bs_read32(dev,BS_MMIO,desc+didx*16+4,&done);
        printf("INGRESS_RESULT ep=%02x usb_rc=%d actual=%d initial_didx=%u final_didx=%u descriptor=%08x rc=%d\n",
               dev->ep_command,usb_rc,actual,didx,hwidx,done,rc);
        uint32_t data[128]={0};
        for(unsigned i=0;i<128 && !rc;i++)rc=bs_read32(dev,BS_MMIO,payload+didx*512+i*4,&data[i]);
        unsigned matches=0,at=0;
        for(unsigned i=0;i+16<=128;i++){
            bool match=true;
            for(unsigned j=0;j<16;j++)if(data[i+j]!=0x51490000u+j)match=false;
            if(match){matches++;at=i*4;}
        }
        printf("INGRESS_PATTERN matches=%u offset=%u bytes=%u\n",matches,at,64u);

        if(!rc && (matches!=1 || hwidx!=(didx+1)%rx1_slots))rc=HYDRASDR_ERROR_PROTOCOL;
        }
        /* Restore pointers with RX disabled. DIDX advanced; closing native
         * session resets WFSYS before this old ROM ring can be reused. */
        for(unsigned j=0;j<3;j++){
            int er=bs_write32(dev,BS_MMIO,rb+4*j,saved[j]);if(!rc)rc=er;
        }
    }
    dev->needs_usb_reset=true;
    return rc;
}
int main(int argc,char **argv)
{
    (void)argc;(void)argv;
    if(!getenv("HYDRA_TEST_USB_INGRESS") ||
       strcmp(getenv("HYDRA_TEST_USB_INGRESS"),"1"))return 2;
    /* RF-on is an entirely separate, MANUAL 20ms shielded path. The user
     * must affirm containment. Require immutable private capture destination
     * and all RF safety controls BEFORE even opening the USB radio. */
    const bool rf_live=getenv("HYDRA_TEST_USB_INGRESS_RF_LIVE") &&
                       !strcmp(getenv("HYDRA_TEST_USB_INGRESS_RF_LIVE"),"1");
    const char *private_prefix=getenv("HYDRA_TEST_USB_INGRESS_RF_CAPTURE_PREFIX");
    if(rf_live){
        const char *need[]={"HYDRA_TX_EXPERIMENTAL","HYDRA_RF_SHIELDED",
                            "HYDRA_TX_ALLOW_UNCALIBRATED_POWER",
                            "HYDRA_TEST_USB_INGRESS_TRANSFER",
                            "HYDRA_TEST_USB_INGRESS_STREAM",
                            "HYDRA_TEST_USB_INGRESS_IQ",
                            "HYDRA_TEST_USB_INGRESS_PACED",
                            "HYDRA_TEST_USB_INGRESS_CONSUME"};
        for(unsigned i=0;i<sizeof(need)/sizeof(need[0]);i++){
            const char *value=getenv(need[i]);
            if(!value || strcmp(value,"1"))return 2;
        }
        if(!private_prefix || strlen(private_prefix)>900u)return 2;
        const char *slash=strrchr(private_prefix,'/');
        if(!slash || slash==private_prefix || !slash[1])return 2;
        char folder[1024];
        size_t length=(size_t)(slash-private_prefix);
        if(length>=sizeof(folder))return 2;
        memcpy(folder,private_prefix,length);folder[length]=0;
        struct stat st={0};
        if(stat(folder,&st) || !S_ISDIR(st.st_mode) ||
           (st.st_mode&0077)!=0 || st.st_uid!=geteuid())return 2;
    }
    /* An RX-only control must never accidentally enable RX1 OUT DMA. Reject
     * mixed requests BEFORE opening or configuring the radio. */
    if(getenv("HYDRA_TEST_USB_INGRESS_RX_ONLY_MS") &&
       getenv("HYDRA_TEST_USB_INGRESS_TRANSFER") &&
       !strcmp(getenv("HYDRA_TEST_USB_INGRESS_TRANSFER"),"1"))return 2;
    struct hydrasdr_device_info info;
    if(hydrasdr_get_device_info(0,&info) || info.vid!=0x0e8d || info.pid!=0x7961)return 3;
    hydrasdr_dev_t *dev=NULL;
    int rc=hydrasdr_open(&dev,0);
    if(rc)return 4;
    /* The accelerated RF-disabled ingress path normally runs with 2RX.
     * Allow a controlled single-RX0 comparison without changing N9 RX1
     * ingress descriptor ownership, WFSYS capture loop or USB OUT framing.
     * RX-off is intentionally unsupported: the N9 ingress service currently
     * executes inside the RX capture firmware, not a standalone task. */
    const char *rx_mode_env=getenv("HYDRA_TEST_USB_INGRESS_RX_MODE");
    const bool rx_single0=rx_mode_env && !strcmp(rx_mode_env,"single0");
    if(rx_mode_env && !rx_single0 && strcmp(rx_mode_env,"dual")){
        hydrasdr_close(dev);return 2;
    }
    if(rf_live && rx_single0){hydrasdr_close(dev);return 2;}
    const char *rate_env=getenv("HYDRA_TEST_USB_INGRESS_RX_RATE");
    unsigned long rx_rate=40000000u;
    if(rate_env){
        char *end=NULL;
        errno=0;
        rx_rate=strtoul(rate_env,&end,10);
        if(errno || !*rate_env || !end || *end ||
           (rx_rate!=10000000u && rx_rate!=20000000u &&
            rx_rate!=40000000u && rx_rate!=80000000u) ||
           (!rx_single0 && rx_rate!=40000000u)){
            hydrasdr_close(dev);
            return 2;
        }
    }
    printf("INGRESS_RX_TEST_MODE mode=%s sample_rate_per_chain=%lu\n",
           rx_single0?"single0":"dual",rx_rate);
    rc=hydrasdr_set_sample_rate(dev,(uint32_t)rx_rate);
    if(!rc)rc=hydrasdr_set_center_freq(dev,UINT64_C(2437000000));
    if(!rc)rc=hydrasdr_set_rx_mode(dev,rx_single0?HYDRASDR_RX_SINGLE0:HYDRASDR_RX_DUAL);
    struct context c={.dev=dev,.stream_rc=-1};
    atomic_init(&c.words,0);
    atomic_init(&c.phase,0);
    if(rf_live && !rc){
        for(unsigned i=0;i<3;i++){
            c.capture[i]=calloc(LIVE_RX_CAPTURE_WORDS*2u,sizeof(int16_t));
            if(!c.capture[i]){rc=HYDRASDR_ERROR_NO_MEMORY;break;}
        }
    }
    pthread_t thread;bool started=false;
    if(!rc) {
        if(pthread_create(&thread,NULL,receiver,&c))rc=HYDRASDR_ERROR_IO;
        else started=true;
    }
    uint32_t state=UINT32_MAX,exp=UINT32_MAX;
    if(started) {
        int ready=HYDRASDR_ERROR_BUSY;
        for(int k=0;k<100;k++) {
            ready=hydra_tx_watchdog_status(dev,&state,&exp);
            if(!ready)break;
            if(ready!=HYDRASDR_ERROR_BUSY)break;
            sleep_ms(20);
        }
        if(!rc)rc=ready;
    }
    uint32_t after=UINT32_MAX;
    uint32_t go_before=0,path_before=0,go_after=0,path_after=0;
    if(started && !rc) {
        pthread_mutex_lock(&dev->control_mutex);
        rc=bs_read32(dev,BS_MMIO,0x8308fc00u,&go_before);
        if(!rc)rc=bs_read32(dev,BS_MMIO,0x8308fc20u,&path_before);
        if(!rc && (go_before&0x40000000u || path_before))rc=HYDRASDR_ERROR_STATE;
        if(!rc){
            struct libusb_config_descriptor *cfg=NULL;
            if(!libusb_get_active_config_descriptor(libusb_get_device(dev->handle),&cfg)){
                for(unsigned i=0;i<cfg->bNumInterfaces;i++)
                  for(int j=0;j<cfg->interface[i].num_altsetting;j++){
                    const struct libusb_interface_descriptor *it=&cfg->interface[i].altsetting[j];
                    for(unsigned e=0;e<it->bNumEndpoints;e++)
                      printf("USB_EP interface=%u ep=%02x type=%u maxpacket=%u\n",
                         it->bInterfaceNumber,it->endpoint[e].bEndpointAddress,
                         it->endpoint[e].bmAttributes,it->endpoint[e].wMaxPacketSize);
                  }
                libusb_free_config_descriptor(cfg);
            }
            const uint32_t addrs[]={0x54000208,0x5400020c,0x54000600,0x54000110,
                0x7c024208,0x7c024600,0x7c024640,0x7c024644};
            for(unsigned i=0;i<sizeof(addrs)/sizeof(addrs[0]);i++){
                uint32_t v=0;rc=bs_read32(dev,BS_MMIO,addrs[i],&v);if(rc)break;
                printf("DMA_STATE %08x %08x\n",addrs[i],v);
            }
            for(unsigned i=0;i<6 && !rc;i++)for(unsigned j=0;j<4;j++){
                uint32_t v=0,a=0x54000500+i*16+j*4;
                rc=bs_read32(dev,BS_MMIO,a,&v);if(rc)break;
                printf("RX_RING %u %08x %08x\n",i,a,v);
            }
        }
        if(!rc && getenv("HYDRA_TEST_USB_INGRESS_TRANSFER") &&
           !strcmp(getenv("HYDRA_TEST_USB_INGRESS_TRANSFER"),"1"))
            rc=ingress_probe(dev,&c);
        after=0;
        pthread_mutex_unlock(&dev->control_mutex);
        if(!rc) {
            pthread_mutex_lock(&dev->control_mutex);
            rc=bs_read32(dev,BS_MMIO,0x8308fc00u,&go_after);
            if(!rc)rc=bs_read32(dev,BS_MMIO,0x8308fc20u,&path_after);
            pthread_mutex_unlock(&dev->control_mutex);
        }
    }
    if(!rc && (after || path_before!=path_after || go_before!=go_after))
        rc=HYDRASDR_ERROR_STATE;
    if(started) {
        /* Opt-in matched RX-only control, never while configuring RX1 OUT DMA.
         * This separates low-rate native capture reliability from the
         * concurrent USB OUT TX-IQ ingress experiment. */
        unsigned rx_idle_ms=80;
        const char *rx_only_ms_env=getenv("HYDRA_TEST_USB_INGRESS_RX_ONLY_MS");
        if(rx_only_ms_env){
            char *end=NULL;
            errno=0;
            unsigned long requested=strtoul(rx_only_ms_env,&end,10);
            if(errno || !*rx_only_ms_env || !end || *end ||
               requested<80 || requested>1500 ||
               (getenv("HYDRA_TEST_USB_INGRESS_TRANSFER") &&
                !strcmp(getenv("HYDRA_TEST_USB_INGRESS_TRANSFER"),"1"))){
                if(!rc)rc=HYDRASDR_ERROR_INVALID;
            }else rx_idle_ms=(unsigned)requested;
        }
        if(rx_only_ms_env)
            printf("INGRESS_RX_ONLY_CONTROL idle_ms=%u no_usb_out=%d\n",rx_idle_ms,
              !(getenv("HYDRA_TEST_USB_INGRESS_TRANSFER") &&
                !strcmp(getenv("HYDRA_TEST_USB_INGRESS_TRANSFER"),"1")));
        sleep_ms((int)rx_idle_ms);
        int cancel=hydrasdr_cancel_async(dev);
        pthread_join(thread,NULL);
        if(!rc)rc=cancel;
        if(!rc)rc=c.stream_rc;
    }
    uint64_t delivered=atomic_load(&c.words);
    struct hydrasdr_stream_stats stats={0};
    int sr=hydrasdr_get_stream_stats(dev,&stats);
    if(!rc)rc=sr;
    if(!rc && (stats.dma_errors || stats.reset_error))rc=HYDRASDR_ERROR_STATE;
    printf("RX_INTEGRITY dma_errors=%u reset_error=%d\n",stats.dma_errors,stats.reset_error);
    printf("RX_TERMINAL_STATS firmware_state=%u stage=%d packets=%llu usb_bytes=%llu samples=%llu stream_rc=%d\n",
           stats.firmware_state,(int)stats.stage,
           (unsigned long long)stats.packets,(unsigned long long)stats.usb_bytes,
           (unsigned long long)stats.samples,c.stream_rc);
    if(rf_live && !rc){
        for(unsigned i=0;i<3;i++){
            printf("INGRESS_RF_RX_CAPTURE phase=%u iq_words=%zu external_rf_verified=false\n",
                   i,c.captured[i]);
            if(c.captured[i]<100000u || save_live_capture(private_prefix,&c,i)){
                rc=HYDRASDR_ERROR_IO;break;
            }
        }
    }
    int close_rc=hydrasdr_close(dev);
    if(!rc)rc=close_rc;
    for(unsigned i=0;i<3;i++)free(c.capture[i]);
    if(!rc && delivered<100000)rc=HYDRASDR_ERROR_STATE;
    printf("USB_INGRESS_RESULT mode=%s rf_external_verified=false rc=%d initial=%u exp=%u final=%u go=%08x/%08x path=%08x/%08x samples=%llu stream=%d close=%d\n",
           rf_live?"bounded_rf_usb_fresh_iq":"no_rf",
           rc,state,exp,after,go_before,go_after,path_before,path_after,
           (unsigned long long)delivered,c.stream_rc,close_rc);
    return rc?1:0;
}
