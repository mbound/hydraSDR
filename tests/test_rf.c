/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Luigi C.
 */
#include "chips/mt7921.h"
#include "hydra_measurement.h"
#include "hydra_tx_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(test) do { if (!(test)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #test); exit(1); } } while (0)
struct word { unsigned space; uint32_t address, value; };
struct model {
    struct word values[2048], writes[2048];
    size_t count, written;
    uint32_t command[2];
    bool stuck_standby;
    uint32_t fail_address;
    bool fail_after_write;
    bool cfg0_auto_clear, cfg0_unexpected_clear;
    uint32_t ignore_address; /* a write here is recorded but does not take */
    uint32_t wd_state;       /* simulated device watchdog state */
    bool wd_expire_on_status;
    bool wd_no_cap,wd_no_arm_ack,dual_watchdog_cap;
    bool c114_mock_stages;
    bool staged_iq_cap,staged_iq_simulate_updates;
    bool active_snapshot_missing,active_snapshot_torn;
};

/* A sparse map models register readbacks, not the production implementation. */
static uint32_t get(struct model *model, unsigned space, uint32_t address)
{
    for (size_t i = 0; i < model->count; ++i)
        if (model->values[i].space == space && model->values[i].address == address)
            return model->values[i].value;
    return 0;
}

/* Store fake state separately from the write trace, including self-clearing LOAD. */
static void put(struct model *model, unsigned space, uint32_t address, uint32_t value)
{
    for (size_t i = 0; i < model->count; ++i) {
        if (model->values[i].space == space && model->values[i].address == address) {
            model->values[i].value = value;
            return;
        }
    }
    CHECK(model->count < 2048);
    model->values[model->count++] = (struct word){space, address, value};
}

/* Decode the bridge result register into a physical bank read. */
static int read32(void *context, enum bs_space space, uint32_t address, uint32_t *value)
{
    struct model *model = context;
    if (space == BS_BRIDGE) {
        uint32_t command = model->command[address >= 0x7c004050];
        *value = address == 0x7c004000 ? 0 :
            address == 0x7c004018 || address == 0x7c004058 ?
            get(model, 16 + ((command >> 13) & 7), command & 0xfff) : command;
    } else {
        if(space==BS_MMIO && model->c114_mock_stages &&
            (address==0x8300c114u || address==0x8301c114u)){
            const uint32_t active=get(model,BS_MMIO,0x8308fc00u);
            *value=(active&0x40000000u)?UINT32_C(0x00005678):UINT32_C(0x00001234);
        }else *value = get(model, (unsigned)space, address);
    }
    return 0;
}

/* Model the SPI command/data launch and record the actual RF write once. This
 * lets a trace from Python compare against C despite their different transports.
 */
static int write32(void *context, enum bs_space space, uint32_t address, uint32_t value)
{
    struct model *model = context;
    unsigned target = (unsigned)space;
    bool fail;
    if (space == BS_BRIDGE) {
        unsigned index = address >= 0x7c004050;
        uint32_t command;
        if (address == 0x7c004010 || address == 0x7c004050) {
            model->command[index] = value;
            return 0;
        }
        command = model->command[index];
        if (command & 0x1000)
            return 0;
        target = 16 + ((command >> 13) & 7);
        address = command & 0xfff;
    }
    fail = address == model->fail_address && model->fail_address;
    if (fail) {
        model->fail_address = 0;
        if (!model->fail_after_write)
            return HYDRASDR_ERROR_IO;
    }
    CHECK(model->written < 2048);
    model->writes[model->written++] = (struct word){target, address, value};
    if (address == model->ignore_address && model->ignore_address)
        return 0;
    if(target==BS_MMIO && (address==0x8308fc0c || address==0x8309fc0c) && model->cfg0_auto_clear)
        value &= ~UINT32_C(0x00010002);
    if(target==BS_MMIO && (address==0x8308fc0c || address==0x8309fc0c) && model->cfg0_unexpected_clear)
        value &= ~UINT32_C(0x00000100); /* stable bit: must be rejected */
    if (target >= 16 && (address == 0xc68 || address == 0xca8))
        value &= 0x7fffffff;
    if (target == 21 && address == 0xb00 && model->stuck_standby)
        value |= 0x10000;
    put(model, target, address, value);
    if(target==BS_MMIO && address==0x02038360u+200u && value==1 &&
        model->staged_iq_simulate_updates){
        /* MCU is modeled as executing 2000 IQ writes over the host session.
         * This is a software sanity check, not a claim about measured timing. */
        put(model,BS_MMIO,0x02038360u+216u,2000);
        put(model,BS_MMIO,0x02038360u+224u,11);
        put(model,BS_MMIO,0x02038360u+232u,1000000);
        put(model,BS_MMIO,0x02038360u+236u,1019990);
        if(!model->active_snapshot_missing)for(unsigned k=0;k<2;k++){
            uint32_t b=0x0203c400u+k*128u;
            put(model,BS_MMIO,b,model->active_snapshot_torn?3:2);
            put(model,BS_MMIO,b+4,1000000+k*17920u);
            put(model,BS_MMIO,b+8,1000003+k*17920u);
            put(model,BS_MMIO,b+12,k?1792:1);
            put(model,BS_MMIO,b+16+8*4,0xc0830000u);
            put(model,BS_MMIO,b+16+24*4,0xfeed);
        }
    }
    if(target==BS_MMIO && address==0x02038360u+112u) {
        /* N9 mailbox mock: ACK and state remain device-owned, the host may
         * never authorize emission without CAP_TX_WATCHDOG and ARM success. */
        unsigned cmd=get(model,BS_MMIO,0x02038360u+116u);
        unsigned arg=get(model,BS_MMIO,0x02038360u+120u);
        unsigned status=0,result=0;
        switch(cmd) {
        case 1:result=HYDRASDR_DIAG_ABI_MAGIC;break;
        case 4:result=HYDRASDR_DIAG_CAP_STATS|
                  (model->wd_no_cap?0:HYDRASDR_DIAG_CAP_TX_WATCHDOG)|
                  (model->dual_watchdog_cap?HYDRASDR_DIAG_CAP_TX_WATCHDOG_DUAL_BANK:0)|
                  (model->staged_iq_cap?(HYDRASDR_DIAG_CAP_STAGED_IQ | HYDRASDR_DIAG_CAP_TX_ACTIVE_SNAPSHOT):0);break;
        case 5:
            if(model->wd_state || arg<20000u || arg>1000000u)status=3;
            else model->wd_state=1;
            break;
        case 6:
            if(model->wd_state!=1 ||
                (get(model,BS_MMIO,0x8308fc00u)&0x40000000u) ||
                get(model,BS_MMIO,0x8308fc20u) ||
                (get(model,BS_MMIO,0x8309fc00u)&0x40000000u) ||
                get(model,BS_MMIO,0x8309fc20u))status=4;
            else model->wd_state=0;
            break;
        case 7:
            if(model->wd_expire_on_status)model->wd_state=2;
            result=model->wd_state;
            break;
        default:status=2;break;
        }
        if(model->wd_no_arm_ack && cmd==5)return 0;
        put(model,BS_MMIO,0x02038360u+132u,result);
        put(model,BS_MMIO,0x02038360u+128u,status);
        put(model,BS_MMIO,0x02038360u+124u,value);
    }
    return fail ? HYDRASDR_ERROR_IO : 0;
}

/* Seed only independently specified identity, strap, completion and trim values. */
static void initialize(struct model *model, hydrasdr_dev_t *dev)
{
    memset(model, 0, sizeof(*model));
    memset(dev, 0, sizeof(*dev));
    dev->io = (struct bs_io){model, read32, write32};
    dev->frequency = UINT64_C(2437000000);
    dev->rate = 40000000;
    dev->rx_mode = HYDRASDR_RX_SINGLE0;
    dev->manual = true;
    dev->gain = (struct hydrasdr_gain){2, 8, 0};
    put(model, 0, 0x02004600, 40000000);
    put(model, 0, 0x70000148, 0x80);
    put(model, 0, 0x70011094, 4);
    put(model, 21, 0x2c, 0x79728a10);
    put(model, 21, 0xb00, 0x28);
    for (unsigned p = 16; p < 18; ++p) {
        put(model, p, 0xd7c, 1u << 21);
        put(model, p, 0xd78, 0x1b);
    }
}

/* Compare bootstrap write/order against the established Python receiver. This
 * catches missing wake steps, bank swaps, constants and ordering regressions.
 * The later matched-filter setup is covered independently by test_filter.
 */
static void test_trace(const char *path)
{
    struct model model;
    hydrasdr_dev_t dev;
    char line[200];
    size_t index = 0;
    FILE *file = fopen(path, "r");
    CHECK(file);
    initialize(&model, &dev);
    CHECK(mt_configure_capture(&dev) == 0);
    CHECK(mt_configure_rf(&dev) == 0);
    CHECK(mt_apply_gain(&dev) == 0);
    while (fgets(line, sizeof(line), file)) {
        unsigned space, address, value;
        if (line[0] == '#') continue;
        CHECK(sscanf(line, "%x %x %x", &space, &address, &value) == 3);
        CHECK(index < model.written);
        if (model.writes[index].space != space || model.writes[index].address != address ||
            model.writes[index].value != value) {
            fprintf(stderr, "Write %zu: Python %x %08x %08x, C %x %08x %08x\n", index,
                     space, address, value, model.writes[index].space,
                     model.writes[index].address, model.writes[index].value);
            exit(1);
        }
        ++index;
    }
    fclose(file);
    CHECK(index == model.written);
    CHECK(get(&model, 21, 0xb00) == 0x28);
    CHECK(get(&model, 16, 0x1e4) == 0x23c1);
    CHECK(get(&model, 17, 0x1e4) == 0x3fb0b);
    printf("%zu ordered writes match the Python receiver\n", index);
}

/* Inject identity, wake-readback and gain-write failures. Failure must propagate;
 * a partial manual-gain update must restore only the previous owned fields.
 */
static void test_failures(void)
{
    struct model model;
    hydrasdr_dev_t dev;
    initialize(&model, &dev);
    put(&model, 21, 0x2c, 0);
    CHECK(mt_configure_rf(&dev) == HYDRASDR_ERROR_UNSUPPORTED && model.written == 0);
    initialize(&model, &dev);
    model.stuck_standby = true;
    CHECK(mt_configure_rf(&dev) == HYDRASDR_ERROR_STATE);
    CHECK(get(&model, 16, 0xca8) == 0);
    initialize(&model, &dev);
    put(&model, 0, 0x830000c4, 0x08080808);
    put(&model, 17, 0x768, 0x00551234);
    model.fail_address = 0x83000150;
    CHECK(mt_apply_gain(&dev) == HYDRASDR_ERROR_IO);
    CHECK(get(&model, 0, 0x830000c4) == 0x08080808);
    CHECK(get(&model, 17, 0x768) == 0x00551234);
    CHECK(get(&model, 0, 0x83000108) == 0);
    initialize(&model, &dev);
    put(&model, 0, 0x830000c4, 0x08080808);
    put(&model, 17, 0x768, 0x00551234);
    CHECK(mt_apply_gain(&dev) == 0 && dev.mt7921.gain_saved);
    CHECK(get(&model, 17, 0x768) & 0x80000000);
    CHECK(mt_restore_gain(&dev) == 0 && !dev.mt7921.gain_saved);
    CHECK(get(&model, 0, 0x830000c4) == 0x08080808);
    CHECK(get(&model, 17, 0x768) == 0x00551234);
    CHECK(mt_apply_gain(&dev) == 0); /* A fresh unforced baseline is reusable. */
    model.fail_address = 0x83000108;
    CHECK(mt_restore_gain(&dev) == HYDRASDR_ERROR_IO && dev.mt7921.gain_saved);
    CHECK(mt_restore_gain(&dev) == 0 && !dev.mt7921.gain_saved);
}

/* Repeated updates retain the boot baseline, preserve unowned fields and roll
 * back to the preceding manual setting when a later register write fails.
 */
static void test_gain_updates(void)
{
    struct model model;
    hydrasdr_dev_t dev;
    struct hydrasdr_gain gain = {1, 4, 6};
    uint32_t previous[4];
    initialize(&model, &dev);
    dev.manual = false;
    put(&model, 0, 0x830000c4, 0x08080808);
    put(&model, 17, 0x768, 0x00551234);
    CHECK(mt_change_gain(&dev, true, &gain) == HYDRASDR_ERROR_STATE);
    CHECK(mt_apply_gain(&dev) == 0 && dev.mt7921.gain_saved);
    CHECK(model.written == 0);
    CHECK(mt_change_gain(&dev, true, &gain) == 0);
    CHECK(get(&model, 0, 0x830000c4) == 0x4949490f);
    CHECK(get(&model, 0, 0x83000108) == 0x06060000);
    CHECK(get(&model, 17, 0x768) == 0x8a551234);
    previous[0] = get(&model, 0, 0x830000c4);
    previous[1] = get(&model, 0, 0x83000108);
    previous[2] = get(&model, 0, 0x83000150);
    previous[3] = get(&model, 17, 0x768);
    gain = (struct hydrasdr_gain){2, 8, 0};
    model.fail_address = 0x83000150;
    CHECK(mt_change_gain(&dev, true, &gain) == HYDRASDR_ERROR_IO);
    CHECK(get(&model, 0, 0x830000c4) == previous[0]);
    CHECK(get(&model, 0, 0x83000108) == previous[1]);
    CHECK(get(&model, 0, 0x83000150) == previous[2]);
    CHECK(get(&model, 17, 0x768) == previous[3]);
    CHECK(mt_change_gain(&dev, true, &gain) == 0);
    CHECK(mt_change_gain(&dev, false, &gain) == 0 && dev.mt7921.gain_saved);
    CHECK(get(&model, 0, 0x830000c4) == 0x08080808);
    CHECK(get(&model, 17, 0x768) == 0x00551234);
    CHECK(mt_change_gain(&dev, true, &gain) == 0);
    CHECK(mt_restore_gain(&dev) == 0 && !dev.mt7921.gain_saved);
    CHECK(get(&model, 17, 0x768) == 0x00551234);
    gain.digital = 13;
    CHECK(mt_change_gain(&dev, true, &gain) == HYDRASDR_ERROR_INVALID);
}

/* A parked retune must target SPI 0's secondary oscillator, leave the other
 * oscillator untouched, and restore routing on both success and failure.
 * An active capture/GO must reject the operation without any RF writes.
 */
static void test_retune(void)
{
    struct model model;
    hydrasdr_dev_t dev;
    size_t before;
    initialize(&model, &dev);
    atomic_init(&dev.run_state, BS_IDLE);
    put(&model, 0, MT_MAILBOX + 28, 1);
    put(&model, 16, 0x240, 0x00123456);
    put(&model, 16, 0xc68, 0x06f00000);
    CHECK(mt_retune_2g_probe(&dev, UINT64_C(2400000114), 18) == 0);
    CHECK(get(&model, 16, 0xca8) == 0x05000004);
    CHECK(get(&model, 16, 0xc68) == 0x06f00000);
    CHECK(get(&model, 16, 0x240) == 0x00123456);
    CHECK(get(&model, 17, 0xca8) == 0);
    CHECK(dev.frequency == UINT64_C(2437000000));
    model.fail_address = 0xce0;
    CHECK(mt_retune_2g_probe(&dev, UINT64_C(2400000114), 18) == HYDRASDR_ERROR_IO);
    CHECK(get(&model, 16, 0x240) == 0x00123456);
    CHECK((get(&model, 16, 0xcc4) & 0x60000000) == 0);
    CHECK((get(&model, 16, 0xcc8) & 1) == 0);
    CHECK(get(&model, 16, 0xce0) == 0);
    CHECK((get(&model, 16, 0xd20) & 4) != 0);
    before = model.written;
    put(&model, 0, MT_MAILBOX + 60, 1);
    CHECK(mt_retune_2g_probe(&dev, UINT64_C(2437000000), 18) == HYDRASDR_ERROR_STATE);
    CHECK(model.written == before);
    put(&model, 0, MT_MAILBOX + 60, 0);
    put(&model, 0, 0x80021090, 0x4f3);
    CHECK(mt_retune_2g_probe(&dev, UINT64_C(2437000000), 18) == HYDRASDR_ERROR_STATE);
    CHECK(model.written == before);
    put(&model, 0, 0x80021090, 0x4f1);
    atomic_store(&dev.run_state, BS_RUNNING);
    CHECK(mt_retune_2g_probe(&dev, UINT64_C(2437000000), 18) == HYDRASDR_ERROR_BUSY);
    CHECK(model.written == before);
    atomic_store(&dev.run_state, BS_IDLE);
    put(&model, 16, 0xd7c, 0);
    CHECK(mt_retune_2g_probe(&dev, UINT64_C(2437000000), 18) == HYDRASDR_ERROR_TIMEOUT);
    CHECK(get(&model, 16, 0x240) == 0x00123456);
    CHECK((get(&model, 16, 0xcc4) & 0x60000000) == 0);
    CHECK((get(&model, 16, 0xcc8) & 1) == 0);
    CHECK(get(&model, 16, 0xce0) == 0);
}

/* Startup must preserve the companion setup before moving the selected LO.
 * Live retuning must leave capture/GO and gain untouched, restore routing and
 * roll back a failed calibration to the previous center without an API commit.
 */
static void test_public_tuning(void)
{
    struct model model;
    hydrasdr_dev_t dev;
    size_t before;
    initialize(&model, &dev);
    dev.frequency = UINT64_C(2300000000);
    CHECK(mt_configure_rf(&dev) == 0);
    CHECK(get(&model, 16, 0xca8) == 0x04caaaab);
    CHECK(get(&model, 16, 0xc68) == 0x06f00000);
    CHECK(get(&model, 17, 0xc68) == 0x0513bbbc);
    CHECK(get(&model, 16, 0x240) == 0);
    put(&model, 0, MT_MAILBOX + 60, 1);
    put(&model, 0, 0x80021090, 0x24f3);
    put(&model, 17, 0x768, 0x94123456);
    before = model.written;
    CHECK(mt_change_frequency(&dev, UINT64_C(2437123456)) == 0);
    CHECK(get(&model, 16, 0xca8) == 0x0513cc97);
    CHECK(get(&model, 0, MT_MAILBOX + 60) == 1);
    CHECK(get(&model, 0, 0x80021090) == 0x24f3);
    CHECK(get(&model, 17, 0x768) == 0x94123456);
    for (size_t i = before; i < model.written; ++i)
        CHECK(model.writes[i].space == 16); /* Only the selected analog bank. */
    dev.frequency = UINT64_C(2437123456); /* Simulate the API's successful commit. */
    model.fail_address = 0xce0;
    CHECK(mt_change_frequency(&dev, MT_RF_MIN_HZ) == HYDRASDR_ERROR_IO);
    CHECK(get(&model, 16, 0xca8) == 0x0513cc97);
    CHECK(dev.frequency == UINT64_C(2437123456));
    CHECK(get(&model, 16, 0x240) == 0);
    CHECK((get(&model, 16, 0xcc4) & 0x60000000) == 0);
    before = model.written;
    CHECK(mt_change_frequency(&dev, MT_RF_MAX_HZ + 1) == HYDRASDR_ERROR_UNSUPPORTED);
    CHECK(model.written == before);
    put(&model, 16, 0xd74, 0xfffff400);
    CHECK(mt_set_frequency(&dev, MT_RF_MIN_HZ) == HYDRASDR_ERROR_STATE);
    put(&model, 16, 0xd74, 0x00000400);
    CHECK(mt_set_frequency(&dev, MT_RF_MAX_HZ) == HYDRASDR_ERROR_STATE);
}

/* Crossing bands moves the VCO range, LO path and front end together, and the
 * way back restores 2.4 GHz. Retunes within a band write no band fields.
 */
static void test_band_switch(void)
{
    struct model model;
    hydrasdr_dev_t dev;
    size_t before;
    initialize(&model, &dev);
    dev.frequency = UINT64_C(5735000000);
    CHECK(mt_configure_rf(&dev) == 0);
    CHECK(get(&model, 16, 0xca8) == 0x072b3333);
    CHECK(get(&model, 16, 0xcac) == 27526);
    CHECK(get(&model, 16, 0x160) & (1u << 29));
    CHECK(get(&model, 17, 0x160) & (1u << 29));
    CHECK((get(&model, 16, 0xc00) & 0x3c) == 0x38);
    before = model.written;
    CHECK(mt_change_frequency(&dev, UINT64_C(5815000000)) == 0);
    for (size_t i = before; i < model.written; ++i)
        CHECK(model.writes[i].space == 16 && model.writes[i].address != 0xc00);
    CHECK(mt_change_frequency(&dev, UINT64_C(2437000000)) == 0);
    CHECK(get(&model, 16, 0xca8) == 0x0513bbbc);
    CHECK(!(get(&model, 16, 0x160) & (1u << 29)));
    CHECK(!(get(&model, 17, 0x160) & (1u << 29)));
    CHECK((get(&model, 16, 0xc00) & 0x3c) == 0x28);
    CHECK(mt_change_frequency(&dev, UINT64_C(3000000000)) == HYDRASDR_ERROR_UNSUPPORTED);
}

/* A failed band write may already have reached the radio. In either direction,
 * rollback must restore both band fields, the VCO and the committed center.
 */
static void test_band_switch_failures(void)
{
    static const uint64_t frequencies[] = {MT_RF_ANCHOR_HZ, UINT64_C(5735000000)};
    static const uint32_t addresses[] = {0xc00, 0x160};
    for (unsigned band = 0; band < 2; ++band) {
        for (unsigned field = 0; field < 2; ++field) {
            for (unsigned applied = 0; applied < 2; ++applied) {
                struct model model;
                hydrasdr_dev_t dev;
                uint32_t lo, synth, receiver, word, cycles;
                initialize(&model, &dev);
                dev.backend = &bs_mt7921_backend;
                dev.frequency = frequencies[band];
                atomic_init(&dev.run_state, BS_IDLE);
                CHECK(pthread_mutex_init(&dev.control_mutex, NULL) == 0);
                CHECK(mt_configure_rf(&dev) == 0);
                lo = get(&model, 16, 0xc00);
                synth = get(&model, 16, 0x160);
                receiver = get(&model, 17, 0x160);
                word = get(&model, 16, 0xca8);
                cycles = get(&model, 16, 0xcac);
                atomic_store(&dev.run_state, BS_RUNNING);
                dev.control_ready = true;
                model.fail_address = addresses[field];
                model.fail_after_write = applied != 0;
                CHECK(hydrasdr_set_center_freq(&dev, frequencies[1 - band]) == HYDRASDR_ERROR_IO);
                CHECK(model.fail_address == 0);
                CHECK(get(&model, 16, 0xc00) == lo);
                CHECK(get(&model, 16, 0x160) == synth);
                CHECK(get(&model, 17, 0x160) == receiver);
                CHECK(get(&model, 16, 0xca8) == word);
                CHECK(get(&model, 16, 0xcac) == cycles);
                CHECK(hydrasdr_get_center_freq(&dev) == frequencies[band]);
                CHECK(atomic_load(&dev.run_state) == BS_CONTROL_FAILED);
                CHECK(dev.control_error == HYDRASDR_ERROR_IO);
                CHECK(pthread_mutex_destroy(&dev.control_mutex) == 0);
            }
        }
    }
}

/* Exercise both low fractional bits through normal startup and the live public
 * setter. A failed update must restore the previous fine word and API metadata,
 * not round the rollback to the old 18-bit grid. No physical resolution is tested.
 */
static void test_fine_tuning(void)
{
    struct model model;
    hydrasdr_dev_t dev;
    initialize(&model, &dev);
    dev.backend = &bs_mt7921_backend;
    atomic_init(&dev.run_state, BS_IDLE);
    CHECK(pthread_mutex_init(&dev.control_mutex, NULL) == 0);
    CHECK(hydrasdr_set_center_freq(&dev, UINT64_C(2400000029)) == 0);
    CHECK(mt_configure_rf(&dev) == 0);
    CHECK(get(&model, 16, 0xca8) == 0x05000001);
    atomic_store(&dev.run_state, BS_RUNNING);
    dev.control_ready = true;
    CHECK(hydrasdr_set_center_freq(&dev, UINT64_C(2400000057)) == 0);
    CHECK(get(&model, 16, 0xca8) == 0x05000002);
    CHECK(hydrasdr_set_center_freq(&dev, UINT64_C(2400000086)) == 0);
    CHECK(get(&model, 16, 0xca8) == 0x05000003);
    model.fail_address = 0xce0;
    CHECK(hydrasdr_set_center_freq(&dev, MT_RF_ANCHOR_HZ) == HYDRASDR_ERROR_IO);
    CHECK(get(&model, 16, 0xca8) == 0x05000003);
    CHECK(hydrasdr_get_center_freq(&dev) == UINT64_C(2400000086));
    CHECK(atomic_load(&dev.run_state) == BS_CONTROL_FAILED);
    CHECK(dev.control_error == HYDRASDR_ERROR_IO);
    CHECK(pthread_mutex_destroy(&dev.control_mutex) == 0);
}

/* Entire RF regression suite is read/write-model based and never opens USB. */
/* Capture setup at every rate must match the 40 MSps write sequence except for
 * the tap rate in 0x830ad448 and the packer bit in 0x800210b8. 20 and 80 MSps
 * use the 80 MSps tap, and 10 and 20 MSps set the packer.
 */
static void test_capture_rates(void)
{
    static const uint32_t rates[] = {10000000, 20000000, 40000000, 80000000};
    static struct model reference, model;
    hydrasdr_dev_t dev;
    initialize(&reference, &dev);
    CHECK(mt_configure_capture(&dev) == 0);
    for (unsigned r = 0; r < sizeof(rates) / sizeof(rates[0]); ++r) {
        const bool fast_tap = rates[r] == 20000000 || rates[r] == 80000000;
        const bool packed = rates[r] == 10000000 || rates[r] == 20000000;
        unsigned taps = 0, packers = 0;
        initialize(&model, &dev);
        dev.rate = rates[r];
        CHECK(mt_configure_capture(&dev) == 0);
        CHECK(model.written == reference.written);
        for (size_t i = 0; i < model.written; ++i) {
            const struct word *w = &model.writes[i], *ref = &reference.writes[i];
            CHECK(w->space == ref->space && w->address == ref->address);
            if (w->space == BS_MMIO && w->address == 0x830ad448) {
                CHECK((w->value & 0x0fffffffu) == (fast_tap ? 0x00880400u : 0x008c0400u));
                ++taps;
            } else if (w->space == BS_MMIO && w->address == 0x800210b8) {
                CHECK((w->value & 0x000f0000u) == (packed ? 0x00020000u : 0));
                CHECK((w->value & 0x9c000000u) == 0x9c000000u);
                ++packers;
            } else {
                CHECK(w->value == ref->value);
            }
        }
        CHECK(taps == 1 && packers == 1);

        /* An ignored tap write is caught at every rate, and an ignored packer
         * write wherever the packer must change.
         */
        initialize(&model, &dev);
        dev.rate = rates[r];
        model.ignore_address = 0x830ad448;
        CHECK(mt_configure_capture(&dev) == HYDRASDR_ERROR_STATE);
        if (packed) {
            initialize(&model, &dev);
            dev.rate = rates[r];
            model.ignore_address = 0x800210b8;
            CHECK(mt_configure_capture(&dev) == HYDRASDR_ERROR_STATE);
        }
    }
}

static void test_gain_pair(void)
{
    struct model model;hydrasdr_dev_t d;initialize(&model,&d);
    d.backend=&bs_mt7921_backend;d.rx_mode=HYDRASDR_RX_DUAL;
    CHECK(!pthread_mutex_init(&d.control_mutex,NULL));atomic_init(&d.run_state,BS_IDLE);
    struct hydrasdr_gain g[2]={{1,2,3},{4,5,6}},out[2];int manual;
    CHECK(!hydra_set_rx_gain_pair(&d,g));CHECK(model.written==0);
    CHECK(!mt_apply_gain(&d));
    CHECK(((get(&model,17,0x768)>>27)&15)==1);
    CHECK(((get(&model,16,0x768)>>27)&15)==4);
    CHECK(((get(&model,BS_MMIO,0x83000108)>>16)&63)==3);
    CHECK(((get(&model,BS_MMIO,0x83010108)>>16)&63)==6);
    CHECK(hydrasdr_get_gain_codes(&d,&out[0],&manual)==HYDRASDR_ERROR_STATE);
    CHECK(!hydra_get_rx_gain_pair(&d,out,&manual));CHECK(out[1].lna==4 && manual);
    atomic_store(&d.run_state,BS_RUNNING);d.control_ready=true;
    g[0].lna=2;g[1].lna=3;model.fail_address=0x83010150;
    CHECK(hydra_set_rx_gain_pair(&d,g)==HYDRASDR_ERROR_IO);
    CHECK(atomic_load(&d.run_state)==BS_CONTROL_FAILED);
    CHECK(((get(&model,17,0x768)>>27)&15)==1);
    CHECK(((get(&model,16,0x768)>>27)&15)==4);
    CHECK(!hydra_get_rx_gain_pair(&d,out,&manual));CHECK(out[0].lna==1 && out[1].lna==4);
    CHECK(!mt_restore_gain(&d));
    CHECK(!(get(&model,17,0x768)&0x80000000u));CHECK(!(get(&model,16,0x768)&0x80000000u));
    atomic_store(&d.run_state,BS_IDLE);
    CHECK(!hydrasdr_set_gain_codes(&d,&g[0]));CHECK(!d.mt7921.gain_pair_enabled);
    g[1].digital=13;size_t writes=model.written;
    CHECK(hydra_set_rx_gain_pair(&d,g)==HYDRASDR_ERROR_INVALID);CHECK(model.written==writes);
    CHECK(!pthread_mutex_destroy(&d.control_mutex));
}


/* Host-timed TX is deliberately testable without powering the RF frontend.
 * Verify start/stop ordering, exact restoration, rejection, and rollback after
 * a partial I/O failure. Actual RF power and physical chain are unqualified.
 */
static void tx_seed(struct model *m,hydrasdr_dev_t *d)
{
    initialize(m,d);d->backend=&bs_mt7921_backend;
    CHECK(!pthread_mutex_init(&d->control_mutex,NULL));
    atomic_init(&d->run_state,BS_RUNNING);d->control_ready=true;
    put(m,BS_MMIO,0x8300c000,0xc0c80806);
    put(m,BS_MMIO,0x8301c000,0xc0c80806);
    put(m,BS_MMIO,0x8300d008,0x1d10);
    put(m,BS_MMIO,0x8301d008,0x1d10);
    put(m,BS_MMIO,0x830ad4d0,0x01010101);
    put(m,BS_MMIO,0x830ad4d8,0x01010101);
}
static void test_tx_control(void)
{
    struct model m;hydrasdr_dev_t d;
    struct hydra_tx_burst_config cfg={0,0};
    const struct hydra_tx_iq_step steps[]={
        {64,0,100},{0,64,100},{-64,0,100},{0,-64,100}
    };
    struct hydra_tx_burst_receipt receipt={0};
    CHECK(!setenv("HYDRA_TX_EXPERIMENTAL","1",1));
    tx_seed(&m,&d);
    m.c114_mock_stages=true;
    uint32_t before[HYDRA_TX_REG_COUNT],after[HYDRA_TX_REG_COUNT];
    CHECK(!hydra_tx_read_raw(&d,before));
    uint32_t wd_state=UINT32_MAX,wd_exp=UINT32_MAX;
    CHECK(!hydra_tx_watchdog_status(&d,&wd_state,&wd_exp));
    CHECK(wd_state==0 && wd_exp==0);
    CHECK(hydra_tx_reg_name(0) && hydra_tx_reg_name(19));
    CHECK(hydra_tx_reg_name(20)==NULL);
    CHECK(!hydra_tx_run_bounded_iq(&d,&cfg,steps,4,&receipt));
    CHECK(receipt.completed_steps==4);
    CHECK(receipt.host_start_ns && receipt.host_stop_ns>=receipt.host_start_ns);
    for(unsigned bank=0;bank<2;bank++){
        CHECK(receipt.candidate_c114[0][bank]==UINT32_C(0x1234));
        CHECK(receipt.candidate_c114[1][bank]==UINT32_C(0x5678));
        CHECK(receipt.candidate_c114[2][bank]==UINT32_C(0x1234));
    }
    CHECK(!hydra_tx_read_raw(&d,after));
    CHECK(!memcmp(before,after,sizeof(before)));
    CHECK(!memcmp(receipt.before,receipt.after,sizeof(before)));
    CHECK(m.wd_state==0);
    bool go_seen=false,stop_seen=false,arm_before_go=false,disarm_after_stop=false;
    bool armed=false;
    for(size_t i=0;i<m.written;i++){
        if(m.writes[i].address==0x02038360u+116u && m.writes[i].value==5)
            armed=true;
        if(m.writes[i].address==0x02038360u+116u && m.writes[i].value==6 && stop_seen)
            disarm_after_stop=true;
        if(m.writes[i].address==0x8308fc00 &&
           m.writes[i].value==0xc0830000u) {
            go_seen=true;arm_before_go=armed;
        }
        if(go_seen && m.writes[i].address==0x8308fc00 &&
           !(m.writes[i].value&(1u<<30)))stop_seen=true;
    }
    CHECK(go_seen && stop_seen && arm_before_go && disarm_after_stop);
    CHECK(!pthread_mutex_destroy(&d.control_mutex));
    /* Hardware-accurate CFG0 self-clear readback: 0x10122 -> 0x120.
     * Only two observed transient bits are exempted from the expected-value
     * comparison; TX must still run, disarm and restore with exact baseline.
     */
    tx_seed(&m,&d);
    m.cfg0_auto_clear=true;
    CHECK(!hydra_tx_run_bounded_iq(&d,&cfg,steps,4,&receipt));
    CHECK(receipt.completed_steps==4 && m.wd_state==0);
    CHECK(!memcmp(receipt.before,receipt.after,sizeof(receipt.before)));
    CHECK(get(&m,BS_MMIO,0x8308fc0c)==0);
    CHECK(!pthread_mutex_destroy(&d.control_mutex));
    tx_seed(&m,&d);
    m.cfg0_auto_clear=true;m.cfg0_unexpected_clear=true;
    CHECK(hydra_tx_run_bounded_iq(&d,&cfg,steps,1,&receipt)==HYDRASDR_ERROR_STATE);
    CHECK(atomic_load(&d.run_state)==BS_CONTROL_FAILED);
    CHECK(get(&m,BS_MMIO,0x8308fc00)==0);
    CHECK(m.wd_state==0);
    CHECK(!pthread_mutex_destroy(&d.control_mutex));
    tx_seed(&m,&d);
    cfg.raw_power_code=1;
    CHECK(hydra_tx_run_bounded_iq(&d,&cfg,steps,1,&receipt)==HYDRASDR_ERROR_UNSUPPORTED);
    CHECK(m.written==0);
    cfg.raw_power_code=0;
    put(&m,BS_MMIO,0x8308fc00,0xc0830000u);
    CHECK(hydra_tx_run_bounded_iq(&d,&cfg,steps,1,&receipt)==HYDRASDR_ERROR_STATE);
    CHECK(m.written==0);
    CHECK(!pthread_mutex_destroy(&d.control_mutex));
    tx_seed(&m,&d);
    m.fail_address=0x8300c508;m.fail_after_write=true;
    CHECK(hydra_tx_run_bounded_iq(&d,&cfg,steps,1,&receipt)==HYDRASDR_ERROR_IO);
    CHECK(atomic_load(&d.run_state)==BS_CONTROL_FAILED);
    CHECK(d.control_error==HYDRASDR_ERROR_IO && !d.control_ready);
    CHECK(get(&m,BS_MMIO,0x8300c508)==0);
    CHECK(get(&m,BS_MMIO,0x8300c000)==0xc0c80806u);
    CHECK(get(&m,BS_MMIO,0x8308fc00)==0);
    CHECK(!pthread_mutex_destroy(&d.control_mutex));
    tx_seed(&m,&d);
    m.ignore_address=0x8308fc00;
    CHECK(hydra_tx_run_bounded_iq(&d,&cfg,steps,1,&receipt)==HYDRASDR_ERROR_STATE);
    CHECK(atomic_load(&d.run_state)==BS_CONTROL_FAILED);
    CHECK(get(&m,BS_MMIO,0x8300c508)==0);
    CHECK(get(&m,BS_MMIO,0x8308fc00)==0);
    CHECK(!pthread_mutex_destroy(&d.control_mutex));
    tx_seed(&m,&d);
    CHECK(!setenv("HYDRA_TX_ALLOW_UNCALIBRATED_POWER","1",1));
    cfg.raw_power_code=42;
    CHECK(!hydra_tx_run_bounded_iq(&d,&cfg,steps,1,&receipt));
    bool power_seen=false,incorrect_high_gain_field=false;
    for(size_t i=0;i<m.written;i++)
        if(m.writes[i].address==0x8300c508){
            if(m.writes[i].value==(0x80008000u|42u))power_seen=true;
            if(m.writes[i].value==(0x80008000u|(42u<<16)))
                incorrect_high_gain_field=true;
        }
    CHECK(power_seen && !incorrect_high_gain_field);
    CHECK(get(&m,BS_MMIO,0x8300c508)==0);
    CHECK(!memcmp(receipt.before,receipt.after,sizeof(receipt.before)));
    CHECK(!pthread_mutex_destroy(&d.control_mutex));
    /* Vendor helper ZEB33 must preserve the full unsigned 8-bit code.
     * Bit31 and bit15 are the tone override enables; 23:16 is NOT gain. */
    tx_seed(&m,&d);
    cfg.raw_power_code=255;
    CHECK(!hydra_tx_run_bounded_iq(&d,&cfg,steps,1,&receipt));
    power_seen=false;
    for(size_t i=0;i<m.written;i++)
        if(m.writes[i].address==0x8300c508 &&
           m.writes[i].value==UINT32_C(0x800080ff))power_seen=true;
    CHECK(power_seen && get(&m,BS_MMIO,0x8300c508)==0);
    CHECK(!pthread_mutex_destroy(&d.control_mutex));
    tx_seed(&m,&d);
    m.wd_no_cap=true;
    CHECK(hydra_tx_run_bounded_iq(&d,&cfg,steps,1,&receipt)==HYDRASDR_ERROR_UNSUPPORTED);
    CHECK(get(&m,BS_MMIO,0x8308fc00)==0);
    CHECK(m.written>0 && m.wd_state==0);
    CHECK(!pthread_mutex_destroy(&d.control_mutex));
    tx_seed(&m,&d);
    m.wd_no_arm_ack=true;
    CHECK(hydra_tx_run_bounded_iq(&d,&cfg,steps,1,&receipt)==HYDRASDR_ERROR_TIMEOUT);
    CHECK(d.needs_usb_reset && atomic_load(&d.run_state)==BS_CONTROL_FAILED);
    CHECK(get(&m,BS_MMIO,0x8308fc00)==0);
    CHECK(!pthread_mutex_destroy(&d.control_mutex));
    tx_seed(&m,&d);
    m.wd_expire_on_status=true;
    cfg.raw_power_code=0;
    CHECK(hydra_tx_run_bounded_iq(&d,&cfg,steps,1,&receipt)==HYDRASDR_ERROR_STATE);
    CHECK(atomic_load(&d.run_state)==BS_CONTROL_FAILED);
    CHECK(d.needs_usb_reset && !d.control_ready);
    CHECK(!pthread_mutex_destroy(&d.control_mutex));
    CHECK(!unsetenv("HYDRA_TX_EXPERIMENTAL"));
    CHECK(!unsetenv("HYDRA_TX_ALLOW_UNCALIBRATED_POWER"));
}


static int dual_tx_mock_hook(hydrasdr_dev_t *d,void *ctx)
{
    bool *seen=ctx; uint32_t v=0;
    if(bs_read32(d,BS_MMIO,0x8308fc00u,&v) || v!=0xc0830000u)return HYDRASDR_ERROR_STATE;
    if(bs_read32(d,BS_MMIO,0x8309fc00u,&v) || v!=0xc0830000u)return HYDRASDR_ERROR_STATE;
    if(bs_read32(d,BS_MMIO,0x8308fc20u,&v) || v!=1u)return HYDRASDR_ERROR_STATE;
    if(bs_read32(d,BS_MMIO,0x8309fc20u,&v) || v!=1u)return HYDRASDR_ERROR_STATE;
    if(bs_read32(d,BS_MMIO,0x8300d008u,&v) || v!=0x1dd0u)return HYDRASDR_ERROR_STATE;
    if(bs_read32(d,BS_MMIO,0x8301d008u,&v) || v!=0x1dd0u)return HYDRASDR_ERROR_STATE;
    if(bs_read32(d,BS_MMIO,0x830ad4d0u,&v) || v!=0x131f1f01u)return HYDRASDR_ERROR_STATE;
    if(bs_read32(d,BS_MMIO,0x8308fc08u,&v) || v!=0x00200040u)return HYDRASDR_ERROR_STATE;
    if(bs_read32(d,BS_MMIO,0x8309fc08u,&v) || v!=0x0fe00fc0u)return HYDRASDR_ERROR_STATE;
    *seen=true; return 0;
}
static void test_dual_tx_control(void)
{
    struct model m; hydrasdr_dev_t d; struct hydra_tx_burst_receipt receipt={0};
    const struct hydra_tx_dual_iq_seed seed={.i0=64,.q0=32,.i1=-64,.q1=-32,.dwell_us=1000};
    bool hook=false;
    CHECK(!setenv("HYDRA_TX_EXPERIMENTAL","1",1));
    CHECK(!setenv("HYDRA_TX_DUAL_EXPERIMENTAL","1",1));
    CHECK(!setenv("HYDRA_RF_SHIELDED","1",1));
    CHECK(!setenv("HYDRA_TX_ALLOW_UNCALIBRATED_POWER","1",1));
    CHECK(!setenv("HYDRA_TEST_DUAL_TX_RF_BURST","1",1));
    tx_seed(&m,&d);m.dual_watchdog_cap=true;m.cfg0_auto_clear=true;
    CHECK(!hydra_tx_run_bounded_dual_iq_with_hook(&d,0,&seed,&receipt,dual_tx_mock_hook,&hook));
    CHECK(hook && receipt.completed_steps==1 && m.wd_state==0);
    CHECK(!memcmp(receipt.before,receipt.after,sizeof(receipt.before)));
    bool go0=false,go1=false,path0=false,path1=false,mux=false,arm=false,go_after_arm=false;
    for(size_t i=0;i<m.written;i++){
        if(m.writes[i].address==0x02038360u+116u && m.writes[i].value==5)arm=true;
        if(m.writes[i].address==0x8308fc00u && m.writes[i].value==0xc0830000u){go0=true;if(arm)go_after_arm=true;}
        if(m.writes[i].address==0x8309fc00u && m.writes[i].value==0xc0830000u)go1=true;
        if(m.writes[i].address==0x8300d008u && m.writes[i].value==0x1dd0u)path0=true;
        if(m.writes[i].address==0x8301d008u && m.writes[i].value==0x1dd0u)path1=true;
        if(m.writes[i].address==0x830ad4d0u && m.writes[i].value==0x131f1f01u)mux=true;
    }
    CHECK(go0&&go1&&path0&&path1&&mux&&go_after_arm);
    CHECK(get(&m,BS_MMIO,0x8308fc00u)==0 && get(&m,BS_MMIO,0x8309fc00u)==0);
    CHECK(get(&m,BS_MMIO,0x8308fc20u)==0 && get(&m,BS_MMIO,0x8309fc20u)==0);
    CHECK(!pthread_mutex_destroy(&d.control_mutex));
    tx_seed(&m,&d);m.cfg0_auto_clear=true;
    hook=false;
    CHECK(hydra_tx_run_bounded_dual_iq_with_hook(&d,0,&seed,&receipt,dual_tx_mock_hook,&hook)==HYDRASDR_ERROR_UNSUPPORTED);
    CHECK(!hook && m.written>0); /* capability query only, no RF register mutation */
    CHECK(get(&m,BS_MMIO,0x8308fc00u)==0 && get(&m,BS_MMIO,0x8309fc00u)==0);
    CHECK(!pthread_mutex_destroy(&d.control_mutex));
    CHECK(!unsetenv("HYDRA_TX_EXPERIMENTAL"));
    CHECK(!unsetenv("HYDRA_TX_DUAL_EXPERIMENTAL"));
    CHECK(!unsetenv("HYDRA_RF_SHIELDED"));
    CHECK(!unsetenv("HYDRA_TX_ALLOW_UNCALIBRATED_POWER"));
    CHECK(!unsetenv("HYDRA_TEST_DUAL_TX_RF_BURST"));
}

/* Host-delivered staged IQ is a separate transaction, explicitly guarded
 * before USB, and must never weaken legacy bounded host-timed TX control. */
static void test_staged_iq_control(void)
{
    struct model m;hydrasdr_dev_t d;
    const struct hydra_tx_iq_sample iq[4]={
       {64,0},{0,64},{-64,0},{0,-64}
    };
    struct hydra_tx_staged_iq_receipt receipt={0};
    struct hydra_tx_burst_config cfg={.chain=0,.raw_power_code=0};
    tx_seed(&m,&d);
    CHECK(hydra_tx_run_bounded_staged_iq(&d,&cfg,iq,0,10,20000,&receipt,NULL,NULL)==HYDRASDR_ERROR_INVALID);
    CHECK(hydra_tx_run_bounded_staged_iq(&d,&cfg,iq,4,1,20000,&receipt,NULL,NULL)==HYDRASDR_ERROR_INVALID);
    CHECK(hydra_tx_run_bounded_staged_iq(&d,&cfg,iq,4,10,90000,&receipt,NULL,NULL)==HYDRASDR_ERROR_INVALID);
    CHECK(hydra_tx_run_bounded_staged_iq(&d,&cfg,iq,4,1000,1000,&receipt,NULL,NULL)==HYDRASDR_ERROR_INVALID);
    CHECK(hydra_tx_run_bounded_staged_iq(&d,&cfg,iq,4,10,20000,&receipt,NULL,NULL)==HYDRASDR_ERROR_UNSUPPORTED);
    CHECK(m.written==0);
    CHECK(!setenv("HYDRA_TX_IQ_STAGED","1",1));
    CHECK(hydra_tx_run_bounded_staged_iq(&d,&cfg,iq,4,10,20000,&receipt,NULL,NULL)==HYDRASDR_ERROR_UNSUPPORTED);
    CHECK(m.written==0);
    CHECK(!setenv("HYDRA_RF_SHIELDED","1",1));
    CHECK(!setenv("HYDRA_TX_EXPERIMENTAL","1",1));
    CHECK(hydra_tx_run_bounded_staged_iq(&d,&cfg,iq,4,10,20000,&receipt,NULL,NULL)==HYDRASDR_ERROR_UNSUPPORTED);
    CHECK(get(&m,BS_MMIO,0x8308fc00u)==0);
    CHECK(!pthread_mutex_destroy(&d.control_mutex));
    tx_seed(&m,&d);m.staged_iq_cap=true;m.staged_iq_simulate_updates=true;
    CHECK(!hydra_tx_run_bounded_staged_iq(&d,&cfg,iq,4,10,20000,&receipt,NULL,NULL));
    CHECK(receipt.sample_count==4 && receipt.period_us==10 && receipt.duration_us==20000);
    CHECK(receipt.completed_updates==2000 && receipt.overruns==0);
    CHECK(receipt.max_update_delta_us==11 && receipt.final_index==0 && !receipt.iq_control_after);
    CHECK(receipt.n9_first_emit_us==1000000u && receipt.n9_last_emit_us==1019990u);
    CHECK(receipt.n9_span_us==19990u);
    CHECK(receipt.active[0].emission==1 && receipt.active[1].emission==1792);
    CHECK(receipt.active[0].words[24]==0xfeed);
    CHECK(hydra_tx_active_reg_name(25) && !hydra_tx_active_reg_name(26));
    CHECK(!memcmp(receipt.tx.before,receipt.tx.after,sizeof(receipt.tx.before)));
    CHECK(get(&m,BS_MMIO,0x02038360u+200u)==0 && m.wd_state==0);
    CHECK(get(&m,BS_MMIO,0x0203c000u)==64u);
    CHECK(get(&m,BS_MMIO,0x0203c004u)==64u<<16);
    CHECK(get(&m,BS_MMIO,0x0203c008u)==(uint32_t)0x00000fc0u);
    CHECK(get(&m,BS_MMIO,0x0203c00cu)==(uint32_t)0x0fc00000u);
    bool staged_before_go=false,guard_armed=false,saw_go=false,saw_playback=false,stopped_before_restore=false;
    for(size_t j=0;j<m.written;j++){
      const struct word *w=&m.writes[j];
      if(w->address==0x0203c000u)staged_before_go=!saw_go;
      if(w->address==0x02038360u+116u && w->value==5)guard_armed=true;
      if(w->address==0x8308fc00u && w->value==0xc0830000u){saw_go=true;CHECK(guard_armed);}
      if(w->address==0x02038360u+200u && w->value==1){CHECK(saw_go);saw_playback=true;}
      if(w->address==0x02038360u+200u && w->value==0 && saw_playback)stopped_before_restore=true;
      if(stopped_before_restore && w->address==0x8300c508u)break;
    }
    CHECK(staged_before_go && saw_go && saw_playback && stopped_before_restore);
    CHECK(!pthread_mutex_destroy(&d.control_mutex));
    /* No N9 staged-IQ capability: fail without touching tone or arming. */
    tx_seed(&m,&d);m.staged_iq_simulate_updates=true;
    CHECK(hydra_tx_run_bounded_staged_iq(&d,&cfg,iq,4,10,20000,&receipt,NULL,NULL)==HYDRASDR_ERROR_UNSUPPORTED);
    CHECK(get(&m,BS_MMIO,0x8308fc00u)==0 && m.wd_state==0);
    CHECK(!pthread_mutex_destroy(&d.control_mutex));
    /* If firmware ignores START, safe rollback, sticky control failure. */
    tx_seed(&m,&d);m.staged_iq_cap=true;m.staged_iq_simulate_updates=true;
    m.ignore_address=0x02038360u+200u;
    CHECK(hydra_tx_run_bounded_staged_iq(&d,&cfg,iq,4,10,20000,&receipt,NULL,NULL)==HYDRASDR_ERROR_STATE);
    CHECK(d.needs_usb_reset && atomic_load(&d.run_state)==BS_CONTROL_FAILED);
    CHECK(get(&m,BS_MMIO,0x8308fc00u)==0 && m.wd_state==0);
    CHECK(!pthread_mutex_destroy(&d.control_mutex));
    /* Missing or in-progress device publication must fail after safe rollback. */
    for(unsigned fault=0;fault<2;fault++){
        tx_seed(&m,&d);m.staged_iq_cap=true;m.staged_iq_simulate_updates=true;
        m.active_snapshot_missing=!fault;m.active_snapshot_torn=!!fault;
        CHECK(hydra_tx_run_bounded_staged_iq(&d,&cfg,iq,4,10,20000,&receipt,NULL,NULL)==HYDRASDR_ERROR_PROTOCOL);
        CHECK(get(&m,BS_MMIO,0x8308fc00u)==0 && m.wd_state==0);
        CHECK(d.needs_usb_reset);
        CHECK(!pthread_mutex_destroy(&d.control_mutex));
    }
    CHECK(!unsetenv("HYDRA_TX_IQ_STAGED"));
    CHECK(!unsetenv("HYDRA_RF_SHIELDED"));
    CHECK(!unsetenv("HYDRA_TX_EXPERIMENTAL"));
}

/* HydraGen4M passive TX observability never writes TX or calibration MMIO. */
static void test_tx_frontend_snapshot(void)
{
    struct model m;hydrasdr_dev_t d;
    tx_seed(&m,&d);
    put(&m,BS_MMIO,0x8300c700,0x01100000);
    put(&m,BS_MMIO,0x8301c700,0x01100000);
    put(&m,16,0x768,0x12345678);
    put(&m,17,0x768,0x00abcdef);
    struct hydra_tx_fe_status o={0};
    CHECK(!hydra_tx_fe_observe(&d,&o));
    CHECK(o.host_begin_ns && o.host_end_ns>=o.host_begin_ns);
    CHECK(o.mmio[4]==0x01100000 && o.mmio[5]==0x01100000);
    CHECK(o.mmio[0]==0xc0c80806 && o.mmio[1]==0xc0c80806);
    CHECK(o.mmio[6]==0x1d10 && o.mmio[7]==0x1d10);
    CHECK(o.adie_gain_raw[0]==0x12345678 && o.adie_gain_raw[1]==0x00abcdef);
    CHECK(m.written==0);
    CHECK(hydra_tx_fe_reg_name(0) && hydra_tx_fe_reg_name(15));
    CHECK(hydra_tx_fe_reg_name(HYDRA_TX_FE_MMIO_COUNT)==NULL);
    struct hydra_tx_fe_status saved=o;
    d.control_ready=false;
    CHECK(hydra_tx_fe_observe(&d,&o)==HYDRASDR_ERROR_BUSY);
    CHECK(!memcmp(&saved,&o,sizeof(o)));
    CHECK(hydra_tx_fe_observe(NULL,&o)==HYDRASDR_ERROR_INVALID);
    CHECK(hydra_tx_fe_observe(&d,NULL)==HYDRASDR_ERROR_INVALID);
    CHECK(!pthread_mutex_destroy(&d.control_mutex));
}

static void test_rx_frontend_snapshot(void)
{
    struct model m;hydrasdr_dev_t d;tx_seed(&m,&d);
    put(&m,BS_MMIO,0x830000c4u,0x12340007u);
    put(&m,BS_MMIO,0x830100c4u,0x45670007u);
    put(&m,BS_MMIO,0x83000108u,0x02020000u);
    put(&m,BS_MMIO,0x83010108u,0x03030000u);
    put(&m,16,0x768u,0x12340000u);
    put(&m,17,0x768u,0x56780000u);
    struct hydra_rx_fe_status o={0};
    CHECK(!hydra_rx_fe_observe(&d,&o));
    CHECK(o.host_begin_ns && o.host_end_ns>=o.host_begin_ns);
    CHECK(o.phy_mmio[0]==0x12340007u && o.phy_mmio[4]==0x45670007u);
    CHECK(o.phy_mmio[1]==0x02020000u && o.phy_mmio[5]==0x03030000u);
    CHECK(o.adie_gain_raw[0]==0x12340000u && o.adie_gain_raw[1]==0x56780000u);
    CHECK(m.written==0);
    CHECK(hydra_rx_fe_reg_name(0) && hydra_rx_fe_reg_name(7));
    CHECK(hydra_rx_fe_reg_name(8)==NULL);
    struct hydra_rx_fe_status before=o;
    d.control_ready=false;
    CHECK(hydra_rx_fe_observe(&d,&o)==HYDRASDR_ERROR_BUSY);
    CHECK(!memcmp(&o,&before,sizeof(before)));
    CHECK(hydra_rx_fe_observe(&d,NULL)==HYDRASDR_ERROR_INVALID);
    CHECK(hydra_rx_fe_observe(NULL,&o)==HYDRASDR_ERROR_INVALID);
    CHECK(!pthread_mutex_destroy(&d.control_mutex));
}

int main(int argc, char **argv)
{
    CHECK(argc == 2);
    test_trace(argv[1]);
    test_failures();
    test_gain_updates();
    test_gain_pair();
    test_tx_control();
    test_dual_tx_control();
    test_staged_iq_control();
    test_tx_frontend_snapshot();
    test_rx_frontend_snapshot();
    test_retune();
    test_public_tuning();
    test_fine_tuning();
    test_band_switch();
    test_band_switch_failures();
    test_capture_rates();
    return 0;
}
