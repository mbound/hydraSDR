/*
MIT License

Copyright (c) 2026 Luigi C.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
.equ MAILBOX, 0x2038360
.equ BOOT_MAGIC, 0x41495242
.equ RESPONSE_XOR, 0xb1771a5d
.equ RING, 0x2040000
.equ HEADERS, 0x2040100
.equ PAYLOAD, 0x2041000
.equ SLOT_STRIDE, 0x3000
.equ RING_COUNT, 0x4
.equ PAYLOAD_BYTES, 0x2000
/* Research firmware diagnostic mailbox, ABI v1; guarded internal commands. */
.equ DIAG_REQ_SEQ, 112
.equ DIAG_CMD, 116
.equ DIAG_ARG, 120
.equ DIAG_DONE_SEQ, 124
.equ DIAG_STATUS, 128
.equ DIAG_RESULT, 132
/* Prototype ROM read output: 4 data words and 1 validity word. */
.equ OTP_DATA, 144
.equ OTP_VALID, 160
.equ OTP_SAVE_R2, 164
.equ OTP_SAVE_R4, 168
.equ OTP_SAVE_R5, 172
.equ OTP_SAVE_LP, 176
.equ OTP_ROM_RC, 180
/* Device-side *polled* TX watchdog. Volatile RAM only, initialized by host. */
.equ TX_WD_ARMED_AT, 184
.equ TX_WD_DURATION_US, 188
.equ TX_WD_STATE, 192         /* 0 disarmed, 1 armed, 2 expired */
.equ TX_WD_EXPIRATIONS, 196
/* Host-fed N9 TX IQ playback table; 128x32-bit signed-I/Q words. Shared
 * volatile system SRAM, NOT the ICAP SRAM, USB DMA ring or OTP.
 * Stage while TX idle. Host owns generator GO, TSSI/power and watchdog.
 * N9 only emits register IQ writes while permission+watchdog remain active.
 * This is MCU-paced playback of a host-uploaded cyclic table, NOT yet a
 * continuous USB bulk OUT RX-ring implementation. */
.equ TX_IQ_TABLE, 0x0203c000
.equ TX_ACTIVE_FIRST, 0x0203c400
.equ TX_ACTIVE_LAST, 0x0203c480
.equ TX_IQ_CONTROL, 200      /* 0 idle; 1 playback; host only sets */
.equ TX_IQ_LENGTH, 204       /* 1..128 complex words */
.equ TX_IQ_PERIOD_US, 208    /* target timer period, >= 1us */
.equ TX_IQ_INDEX, 212
.equ TX_IQ_COMPLETED, 216
.equ TX_IQ_LAST_TICK, 220
.equ TX_IQ_MAX_DELTA, 224
.equ TX_IQ_OVERRUN, 228
.equ TX_IQ_FIRST_EMIT, 232  /* N9 timer tick of first actual setpoint write */
.equ TX_IQ_LAST_EMIT, 236   /* N9 timer tick of last actual setpoint write */
/* B2 NO-RF host-defined signed12 ingress sample consumer.
 * Mode0 validates a DMA RX1 packet and recycles the descriptor immediately.
 * Mode1 verifies/copies each packet into a separate 16-slot N9 SRAM FIFO
 * before recycling its 4-slot *hardware* RX1 DMA descriptor, then the
 * timer-paced consumer independently reads those immutable queued IQ words.
 * Stage ownership is held until the last word and checksum have been read.
 * NO PHY/TX MMIO writes in this consumer. */
.equ RX1_CONSUMER_MODE, 292 /* 0 legacy, 1 paced no-RF consume */
.equ RX1_CONSUMER_PERIOD, 296
.equ RX1_CONSUMER_STATE, 300 /* 0 priming, 1 wait-next, 2 running, 3 target reached, 4 fault */
.equ RX1_CONSUMER_INDEX, 304 /* word offset in current packet, 0..63 */
.equ RX1_CONSUMER_LAST, 308
.equ RX1_CONSUMER_UNDERRUN, 312
.equ RX1_CONSUMER_MAX_DELTA, 316
.equ RX1_CONSUMER_FIRST, 320
.equ RX1_CONSUMER_FINAL, 324
.equ RX1_CONSUMER_DIGEST, 328
.equ RX1_CONSUMER_LATE, 332
.equ RX1_CONSUMER_EXPECT_LO, 340
.equ RX1_CONSUMER_EXPECT_HI, 344
.equ RX1_RING_COUNT, 4        /* physically qualified RX1 DMA count */
.equ RX1_RING_MASK, 3
/* 16-deep N9 SOFT staging FIFO independent of 4-slot hardware DMA ring.
 * DMA buffer0x203d000..0x203d800; stage0x203e000..0x2040000.
 * Does not overlap RX0 DMA ring0x2040000 or TX IQ table0x203c000. */
.equ RX1_STAGE_BASE, 0x0203e000
.equ RX1_STAGE_COUNT, 16
.equ RX1_STAGE_MASK, 15
.equ RX1_STAGE_PROD, 348
.equ RX1_STAGE_CONS, 352
.equ RX1_STAGE_FILL, 356
.equ RX1_STAGE_HIGHWATER, 360
.equ RX1_STAGE_OVERRUN, 364
.equ RX1_STAGE_PACKET_LO, 368
.equ RX1_STAGE_PACKET_HI, 372
.equ RX1_STAGE_SAMPLE_LO, 376
.equ RX1_STAGE_SAMPLE_HI, 380
.equ RX1_STAGE_SRC_SCRATCH, 384
.equ RX1_STAGE_DST_SCRATCH, 388
.equ RX1_STAGE_DIGEST_SCRATCH, 392
/* B3 no-RF pacing experiment, source-qualified only to the N9 1us timer:
 * 0 = legacy last-actual-sample relative scheduling;
 * 1 = phase-locked deadline accumulator. This can recover average rate but
 * DOES NOT constrain per-sample jitter or create a hardware DAC clock.
 * Only used in mode1 no-RF sample-consumption tests. */
.equ RX1_CLOCK_MODE, 396
/* Mode2 is a NO-RF *write* to the actual PHY IQ setpoint. Verify generator
 * GO/path/override/WDT idle at each write; no RF enable register changes.
 * The host independently restores and readback-verifies this PHY value on
 * teardown. This mode is intentionally NOT a TX streaming feature. */
.equ RX1_PHY_DRY_LAST, 400
.equ RX1_PHY_DRY_WRITES, 404
/* Host may abort after a USB error with the software consumer still live.
 * Explicit N9 acknowledgement ensures no subsequent PHY IQ store races
 * the host's readback-verified restoration. */
.equ RX1_PHY_DRY_ABORT_REQ, 408
.equ RX1_PHY_DRY_ABORT_ACK, 412
/* B3 opt-in no-RF experiment: bounded 1/2 extra RX1 descriptor checks,
 * or mode3 alternating 1 and 2 extra attempts per capture RX packet.
 * Mode0 remains the proven legacy path.
 * Only active with RX1 audit enabled and no staged/PHY-dry playback.
 * No PHY/TX, DMA register configuration, capture ring or IRQ changes.
 * Counter units: extra attempts, extra consumed packets and microseconds. */
.equ RX1_EXTRA_SERVICE, 416
.equ RX1_EXTRA_ATTEMPTS, 420
.equ RX1_EXTRA_ACCEPTED, 424
.equ RX1_EXTRA_MAX_US, 428
.equ RX1_EXTRA_START_TICK, 432
.equ RX1_EXTRA_PREV_PACKET, 436
.equ RX1_EXTRA_REMAIN, 440
/* R2 source-qualified USB OUT -> checked N9 SRAM FIFO -> PHY IQ test
 * register. Mode3 only: RF_ARM=0 idle/prime, 1 short live watchdog-gated
 * RF writer, 2 NO-RF dry consumer. Other modes unchanged. The HOST alone
 * controls TX GO/PA/path; N9 does not enable them. An ABORT request is
 * acknowledged only after the N9 writer is quiescent and GO forced off. */
.equ RX1_RF_ARM, 444
.equ RX1_RF_WRITES, 448
.equ RX1_RF_LAST, 452
.equ RX1_RF_FIRST_TICK, 456
.equ RX1_RF_LAST_TICK, 460




/*
 * Original continuous I/Q exporter from capture SRAM to serial USB DMA.
 * The only ROM call powers on the PHY. No vendor RAM firmware is downloaded.
 * The immutable boot ROM handles USB enumeration and program download.
 * The host owns RF tuning and gain, then releases GO after queuing USB reads.
 * All descriptors and headers reside in dedicated MCU data memory.
 * The program parks after publishing a terminal state until a subsystem reset.
 * Opted-in continuous reads instead recover source faults inside this program.
 *
 * Mailbox offsets are bytes from MAILBOX. The host supplies the nonce at +4,
 * packet limit at +32 (zero means until STOP), STOP at +36 and GO at +60.
 * The program reports state at +28, submitted/completed counts at +40/+44,
 * elapsed microseconds at +48 and DMA error status at +52.
 * The exporter submits one SRAM descriptor at a time and waits for completion
 * before releasing its source bytes. It tracks producer progress while waiting
 * for both data and DMA, reporting source overrun or ambiguous producer timing
 * as explicit gaps or terminal failures rather than hiding overwritten samples.
 *
 * State values: 1 ready/running, 2 finite done, 3 host stop, 4 DMA timeout,
 * 5 GO timeout, 6 initial DMA busy, 8 ambiguous wrap, 9 source overrun and
 * 10 source timeout. Source faults and normal stops drain the descriptor before
 * stopping capture.
 * Extra mailbox offsets: +64 produced units, +68 final capture control,
 * +72 producer position, +76 ROM power-on result, +80 maximum backlog and
 * +84 maximum poll interval in microseconds. The scratch words at +88/+92
 * hold the preceding poll time and completed source units.
 * The host enables local recovery at +96. The words at +100/+104 hold the
 * packet count at the current SRAM epoch and the cumulative gap count.
 * The scratch flag at +108 records whether ICAP is actively producing.
 * A 48-byte event (extended ID 0xf6) precedes each new epoch. It carries the
 * fault at header +8, gap count at +12, next data sequence at +36 and nonce
 * at +40. Its four-byte trailer is not data. The host must discard the last
 * held data packet because DMA may have read it while ICAP overwrote it.
 * Recovery drains DMA, stops ICAP, sends and drains this marker, then restarts
 * ICAP at offset zero. It does not reset USB, the PHY, gain or the synthesizer.
 * Only known source faults are recoverable. DMA errors and timeouts still park.
 */
    .section .text.entry,"ax"
    .global _start
    .type _start, @function
    .macro load32 reg,value
    sethi \reg, (\value >> 12)
    ori \reg, \reg, (\value & 0xfff)
    .endm
    /* Observe the live capture even while USB is stalled. One address unit is
     * four SRAM bytes: one complex sample at 40 and 80 MSps, or one of the
     * four words in a packed record at 10 and 20 MSps, where each record holds
     * two samples and two zero words. The ring contains 65536 units, which
     * last 1.6384 ms at 20 and 40 MSps and 0.8192 ms at 80 MSps. The 750 us
     * poll guard, still shorter than the 80 MSps ring, rejects ambiguous
     * producer movement before a whole ring can pass unnoticed.
     * Scratch r0/r1/r3, preserve the DMA wait counter (r2) and ring registers. */
    .macro observe_source
    load32 $r0, 0x89020068
    lwi $r0, [$r0]
    lwi $r1, [$fp + 88]
    swi $r0, [$fp + 88]
    sub $r1, $r0, $r1
    lwi $r3, [$fp + 84]
    slt $r3, $r3, $r1
    beqz $r3, 90f
    swi $r1, [$fp + 84]
90:
    slti $r3, $r1, 750
    beqz $r3, timing_fault
    load32 $r0, 0x800210b4
    lwi $r0, [$r0]
    zeh33 $r0, $r0
    lwi $r1, [$fp + 72]
    swi $r0, [$fp + 72]
    sub $r1, $r0, $r1
    zeh33 $r1, $r1
    lwi $r0, [$fp + 64]
    add $r0, $r0, $r1
    swi $r0, [$fp + 64]
    lwi $r1, [$fp + 92]
    sub $r0, $r0, $r1
    lwi $r3, [$fp + 80]
    slt $r3, $r3, $r0
    beqz $r3, 91f
    swi $r0, [$fp + 80]
91:
    movi $r3, 61440
    slt $r3, $r0, $r3
    beqz $r3, source_overrun
    .endm
    /* The host may crash while TX GO is set. Device watchdog uses N9's own
     * microsecond timer. It is polled from the RX loop and wait states; it is
     * NOT an interrupt and cannot act if the N9 MCU itself is wedged.
     * Only $r0/$r1/$r3 are clobbered so the RX sequence/USB DMA is preserved. */
    .macro tx_shutdown
    load32 $r0, 0x8308fc00
    lwi $r1, [$r0]
    bclr $r1, $r1, 30
    swi $r1, [$r0]
    load32 $r0, 0x8308fc20
    movi $r1, 0
    swi $r1, [$r0]
    /* Clear the TX-active path state on every logical RF path. Exact ROM
     * active value 0x1dd0 differs from safe forced-off 0x1d90 by bit6.
     * Paths2/3 are normally zero at idle; host rollback restores snapshots. */
    load32 $r1, 0x00001d90
    load32 $r0, 0x8300d008
    swi $r1, [$r0]
    load32 $r0, 0x8301d008
    swi $r1, [$r0]
    load32 $r0, 0x8302d008
    swi $r1, [$r0]
    load32 $r0, 0x8303d008
    swi $r1, [$r0]
    /* Peer PHY generator must also be stopped. Host rollback restores the
     * complete saved frontend/path state after watchdog intervention. */
    load32 $r0, 0x8309fc00
    lwi $r1, [$r0]
    bclr $r1, $r1, 30
    swi $r1, [$r0]
    load32 $r0, 0x8309fc20
    movi $r1, 0
    swi $r1, [$r0]
    dsb
    .endm
_start:
    load32 $fp, MAILBOX
    mfsr $r0, $psw
    swi $r0, [$fp + 12]
    swi $sp, [$fp + 16]
    swi $gp, [$fp + 20]
    swi $lp, [$fp + 24]
    lwi $r0, [$fp + 4]
    load32 $r1, RESPONSE_XOR
    xor $r0, $r0, $r1
    swi $r0, [$fp + 8]
    /* Use the immutable ROM only for PHY power-on. */
    load32 $r2, 0x00837142
    jral5 $r2
    swi $r0, [$fp + 76]
    load32 $r6, 0x54000300
    load32 $r7, RING
    /* ROM handoff disables both directions. Only take an idle engine. */
    lwi $r0, [$r6 - 248]
    andi $r1, $r0, 10
    bnez $r1, initial_busy
    bitci $r0, $r0, 5
    bclr $r0, $r0, 30
    swi $r0, [$r6 - 248]
    movi $r0, 0
    swi $r0, [$r6 - 252]  /* 0x54000204: mask DMA interrupts */
    /* Clear all our ring slots and 64-byte event headers. */
    load32 $r1, RING
    movi $r2, (RING_COUNT * 4)
clear_ring:
    swi.bi $r0, [$r1], 4
    addi $r2, $r2, -1
    bnez $r2, clear_ring
    load32 $r1, HEADERS
    movi $r2, (RING_COUNT * 16)
clear_headers:
    swi.bi $r0, [$r1], 4
    addi $r2, $r2, -1
    bnez $r2, clear_headers
    /* Initialize each event header with this acquisition's nonce. */
    load32 $r8, HEADERS
    load32 $r9, PAYLOAD
    movi $r10, RING_COUNT
initialize_slot:
    load32 $r0, (0x3800002c + PAYLOAD_BYTES)
    swi $r0, [$r8 + 0]
    load32 $r0, (0x38000014 + PAYLOAD_BYTES)
    swi $r0, [$r8 + 24]
    movi $r0, 0xed
    swi $r0, [$r8 + 28]
    movi $r0, 0xf5
    swi $r0, [$r8 + 32]
    lwi $r0, [$fp + 4]
    swi $r0, [$r8 + 40]
    addi $r8, $r8, 64
    addi $r9, $r9, SLOT_STRIDE
    addi $r10, $r10, -1
    bnez $r10, initialize_slot
    dsb
    swi $r7, [$r6 + 0]
    movi $r0, RING_COUNT
    swi $r0, [$r6 + 4]
    movi $r0, 1
    swi $r0, [$r6 - 244] /* 0x5400020c: reset TX0 DMA index */
    movi $r0, 0
    swi $r0, [$r6 + 8]
    lwi $r0, [$r6 - 248]
    ori $r0, $r0, 1      /* TX enabled; RX remains disabled */
    swi $r0, [$r6 - 248]
    movi $r0, 1
    swi $r0, [$fp + 28]
    dsb
    load32 $r0, BOOT_MAGIC
    swi $r0, [$fp]
    dsb
    /* Fresh ROM boot initializes all TX ring-control fields before GO.
     * Existing RX/USB mailbox offsets and authenticated ROM operations stay. */
    movi $r0, 0
    swi $r0, [$fp + 240] /* no-RF USB ingress audit disabled on boot */
    swi $r0, [$fp + RX1_CONSUMER_MODE]
    swi $r0, [$fp + RX1_CONSUMER_STATE]
    swi $r0, [$fp + RX1_EXTRA_SERVICE]
    swi $r0, [$fp + RX1_EXTRA_ATTEMPTS]
    swi $r0, [$fp + RX1_EXTRA_ACCEPTED]
    swi $r0, [$fp + RX1_EXTRA_MAX_US]
    swi $r0, [$fp + RX1_EXTRA_START_TICK]
    swi $r0, [$fp + RX1_EXTRA_PREV_PACKET]
    swi $r0, [$fp + RX1_EXTRA_REMAIN]
    swi $r0, [$fp + RX1_RF_ARM]
    swi $r0, [$fp + RX1_RF_WRITES]
    swi $r0, [$fp + RX1_RF_LAST]
    swi $r0, [$fp + RX1_RF_FIRST_TICK]
    swi $r0, [$fp + RX1_RF_LAST_TICK]
    swi $r0, [$fp + TX_IQ_CONTROL]
    swi $r0, [$fp + TX_IQ_LENGTH]
    swi $r0, [$fp + TX_IQ_PERIOD_US]
    swi $r0, [$fp + TX_IQ_INDEX]
    swi $r0, [$fp + TX_IQ_COMPLETED]
    swi $r0, [$fp + TX_IQ_LAST_TICK]
    swi $r0, [$fp + TX_IQ_MAX_DELTA]
    swi $r0, [$fp + TX_IQ_OVERRUN]
    swi $r0, [$fp + TX_IQ_FIRST_EMIT]
    swi $r0, [$fp + TX_IQ_LAST_EMIT]
    movi $r4, 0          /* sequence */
    movi $r5, 0          /* producer index */
    swi $r4, [$fp + 108]
    load32 $r2, 100000000
wait_go:
    jal tx_watchdog_service
    jal tx_iq_service
    jal diag_service
    lwi $r0, [$fp + 60]
    bnez $r0, begin_stream
    lwi $r0, [$fp + 36]
    bnez $r0, stopped
    addi $r2, $r2, -1
    bnez $r2, wait_go
    movi $r0, 5
    j finish
begin_stream:
    load32 $r0, 0x89020068
    lwi $r0, [$r0]
    swi $r0, [$fp + 56]
    movi $r1, 0
    swi $r1, [$fp + 104]
    swi $r1, [$fp + 80]
    swi $r1, [$fp + 84]
restart_capture:
    /* Reset only ICAP's capture cursor, after all DMA readers have drained. */
    load32 $r0, 0x80021090
    movi $r1, 0x4f1
    swi $r1, [$r0]
    dsb
    swi $r4, [$fp + 100]
    load32 $r0, 0x89020068
    lwi $r0, [$r0]
    swi $r0, [$fp + 88]
    movi $r0, 0
    swi $r0, [$fp + 64]  /* produced address units, modulo 2^32 */
    swi $r0, [$fp + 72]  /* last 16-bit producer index */
    swi $r0, [$fp + 92]  /* completed address units, modulo 2^32 */
    load32 $r0, 0x80021090
    movi $r1, 0x24f3
    movi $r3, 1
    swi $r3, [$fp + 108]
    swi $r1, [$r0]
publish:
    jal ingress_service
    /* Gated experiment on SAME N9 RX1 descriptor/nonce/sequence validator:
     * no more than 1 or 2 additional packet checks per RX capture publish.
     * Never run on staged/PHY-dry mode, RF TX, or idle RX1 DMA. The next
     * packet path reinitializes the RX DMA countdown; ingress_service saves
     * r2 and clobbers no long-lived RX state. This is NOT a DAC clock. */
    lwi $r0, [$fp + RX1_EXTRA_SERVICE]
    beqz $r0, publish_extra_done
    movi $r1, 3
    bne $r0, $r1, publish_extra_fixed_budget
    /* Mode3 spreads work: alternating 1 and 2 extra packet services.
     * Source-qualified r4 is this N9 RX capture publish sequence index. */
    andi $r0, $r4, 1
    addi $r0, $r0, 1
publish_extra_fixed_budget:
    slti $r3, $r0, 3
    beqz $r3, publish_extra_done /* fail closed for invalid >3 mode */
    lwi $r1, [$fp + 240]
    beqz $r1, publish_extra_done
    lwi $r1, [$fp + RX1_CONSUMER_MODE]
    bnez $r1, publish_extra_done
    swi $r0, [$fp + RX1_EXTRA_REMAIN]
publish_extra_loop:
    /* No waiting for RX1 DONE; retry is bounded by host-supplied budget. */
    load32 $r1, 0x89020068
    lwi $r0, [$r1]
    swi $r0, [$fp + RX1_EXTRA_START_TICK]
    lwi $r0, [$fp + RX1_EXTRA_ATTEMPTS]
    addi $r0, $r0, 1
    swi $r0, [$fp + RX1_EXTRA_ATTEMPTS]
    lwi $r0, [$fp + 252]
    swi $r0, [$fp + RX1_EXTRA_PREV_PACKET]
    jal ingress_service
    lwi $r1, [$fp + RX1_EXTRA_PREV_PACKET]
    lwi $r0, [$fp + 252]
    sub $r0, $r0, $r1
    lwi $r1, [$fp + RX1_EXTRA_ACCEPTED]
    add $r1, $r1, $r0
    swi $r1, [$fp + RX1_EXTRA_ACCEPTED]
    load32 $r1, 0x89020068
    lwi $r0, [$r1]
    lwi $r1, [$fp + RX1_EXTRA_START_TICK]
    sub $r0, $r0, $r1
    lwi $r1, [$fp + RX1_EXTRA_MAX_US]
    slt $r3, $r1, $r0
    beqz $r3, publish_extra_next
    swi $r0, [$fp + RX1_EXTRA_MAX_US]
publish_extra_next:
    lwi $r0, [$fp + RX1_EXTRA_REMAIN]
    addi $r0, $r0, -1
    swi $r0, [$fp + RX1_EXTRA_REMAIN]
    bnez $r0, publish_extra_loop
publish_extra_done:
    jal ingress_paced_service
    jal tx_watchdog_service
    jal tx_iq_service
    /* RX USB DMA continues independently while the N9 is running. N9 was
     * updating IQ only once per ~25.6us RX packet (39.8k/s actual). A short
     * inter-packet service window permits additional timer-paced IQ writes
     * without stalling for host EP0 traffic. It is bounded to 20us and
     * enabled ONLY while the watchdog-gated staged playback is active. The
     * RX producer/ring overrun checks remain authoritative and a failed RX
     * stream is NOT a successful concurrent SDR test. Scratch r2 is safe
     * here: next_packet initializes the DMA countdown on every iteration. */
    lwi $r0, [$fp + TX_IQ_CONTROL]
    bnez $r0, publish_begin_iq_window
    lwi $r0, [$fp + 240]
    beqz $r0, publish_skip_iq_window
    lwi $r0, [$fp + RX1_CONSUMER_MODE]
    beqz $r0, publish_skip_iq_window
publish_begin_iq_window:
    load32 $r1, 0x89020068
    lwi $r2, [$r1]
publish_iq_service_window:
    jal tx_iq_service
    jal ingress_paced_service
    load32 $r1, 0x89020068
    lwi $r1, [$r1]
    sub $r1, $r1, $r2
    slti $r3, $r1, 20
    bnez $r3, publish_iq_service_window
publish_skip_iq_window:
    /* At most one mailbox poll per 64 I/Q packets (~3.28ms at 40MSps).
     * Guarded internal commands include bounded RF-off DMA timing; no reboot. */
    andi $r0, $r4, 63
    bnez $r0, diag_skip_publish
    jal diag_service
diag_skip_publish:
    lwi $r0, [$fp + 36]
    bnez $r0, stopped
    lwi $r0, [$fp + 32]
    beqz $r0, next_packet
    beq $r4, $r0, done
next_packet:
    /* Wait for the preceding SRAM descriptor before publishing the next one. */
    addi $r10, $r5, 1
    andi $r10, $r10, (RING_COUNT - 1)
    load32 $r2, 100000000
wait_space:
    /* Check during stalls as well as the normal packet-publish cadence. */
    andi $r0, $r2, 255
    bnez $r0, tx_watchdog_skip_wait_space
    jal tx_watchdog_service
tx_watchdog_skip_wait_space:
    /* Host-preloaded TX-IQ test: sample-timing service is much more
     * frequent than watchdog maintenance. The previous WIP put both in
     * the same /256 poll path, limiting TX to ~39.8 kupdates/s on device.
     * Keep RX ring progress bounded and explicitly report late updates. */
    andi $r0, $r2, 15
    bnez $r0, tx_iq_skip_wait_space
    jal tx_iq_service
tx_iq_skip_wait_space:
    lwi $r0, [$r6 + 12]
    sub $r1, $r5, $r0
    andi $r1, $r1, (RING_COUNT - 1)
    sub $r1, $r4, $r1
    swi $r1, [$fp + 44]
    lwi $r0, [$fp + 100]
    sub $r1, $r1, $r0
    slli $r1, $r1, 11
    swi $r1, [$fp + 92]
    observe_source
    lwi $r0, [$r6 + 12]
    /* SRAM reads require serial descriptors on this part. Multiple queued
     * SRAM packets have reproduced silent payload corruption. */
    beq $r0, $r5, space_ready
    lwi $r0, [$fp + 36]
    bnez $r0, stopped
    addi $r2, $r2, -1
    bnez $r2, wait_space
    movi $r0, 4
    j finish
space_ready:
    lwi $r1, [$fp + 100]
    sub $r1, $r4, $r1
    slli $r1, $r1, 11
    swi $r1, [$fp + 92]
    load32 $r2, 1000000
wait_source:
    /* Check during stalls as well as the normal packet-publish cadence. */
    andi $r0, $r2, 255
    bnez $r0, tx_watchdog_skip_wait_source
    jal tx_watchdog_service
tx_watchdog_skip_wait_source:
    /* Host-preloaded TX-IQ test: sample-timing service is much more
     * frequent than watchdog maintenance. The previous WIP put both in
     * the same /256 poll path, limiting TX to ~39.8 kupdates/s on device.
     * Keep RX ring progress bounded and explicitly report late updates. */
    andi $r0, $r2, 15
    bnez $r0, tx_iq_skip_wait_source
    jal tx_iq_service
tx_iq_skip_wait_source:
    lwi $r0, [$fp + 36]
    bnez $r0, stopped
    observe_source
    slti $r3, $r0, (PAYLOAD_BYTES / 4)
    beqz $r3, source_ready
    addi $r2, $r2, -1
    bnez $r2, wait_source
    movi $r10, 10
    j drain
source_ready:
    slli $r0, $r5, 6
    load32 $r8, HEADERS
    add $r8, $r8, $r0
    /* A slot previously used for a gap must become an ordinary data header. */
    load32 $r0, (0x3800002c + PAYLOAD_BYTES)
    swi $r0, [$r8]
    load32 $r0, (0x38000014 + PAYLOAD_BYTES)
    swi $r0, [$r8 + 24]
    movi $r0, 0xf5
    swi $r0, [$r8 + 32]
    movi $r0, 0
    swi $r0, [$r8 + 8]
    swi $r0, [$r8 + 12]
    /* Exact payload DMA length, following BirriaSDR e9ea0e8 (Yury MonZon).
     * USB consumes four bytes past RXD0 length; take them from the cleared
     * header slot, not from SRAM beyond the payload. The former odd tail
     * was associated upstream with duplication/loss in 128-byte lines. */
    lwi $r0, [$fp + 100]
    sub $r0, $r4, $r0
    andi $r0, $r0, 31
    slli $r0, $r0, 13
    load32 $r9, 0xe0200000
    add $r9, $r9, $r0
    swi $r4, [$r8 + 36]
    slli $r0, $r5, 4
    add $r0, $r7, $r0
    swi $r8, [$r0]
    swi $r9, [$r0 + 8]
    movi $r1, 0
    swi $r1, [$r0 + 12]
    load32 $r1, ((48 << 16) | (1 << 14) | PAYLOAD_BYTES)
    swi $r1, [$r0 + 4]
    dsb
    addi $r5, $r10, 0
    swi $r5, [$r6 + 8]
    addi $r4, $r4, 1
    swi $r4, [$fp + 40]
    j publish
done:
    movi $r10, 2
    j drain
stopped:
    movi $r10, 3
drain:
    /* Drain before disabling SRAM. The final source packet is untrusted after
     * a source fault, so the ordered marker tells the host to discard it. */
    load32 $r2, 100000000
wait_dma:
    /* Check during stalls as well as the normal packet-publish cadence. */
    andi $r0, $r2, 255
    bnez $r0, tx_watchdog_skip_wait_dma
    jal tx_watchdog_service
tx_watchdog_skip_wait_dma:
    /* Host-preloaded TX-IQ test: sample-timing service is much more
     * frequent than watchdog maintenance. The previous WIP put both in
     * the same /256 poll path, limiting TX to ~39.8 kupdates/s on device.
     * Keep RX ring progress bounded and explicitly report late updates. */
    andi $r0, $r2, 15
    bnez $r0, tx_iq_skip_wait_dma
    jal tx_iq_service
tx_iq_skip_wait_dma:
    /* A normal STOP must still detect overwrite of its final descriptor. */
    lwi $r0, [$fp + 108]
    beqz $r0, drain_fault
    slti $r0, $r10, 8
    beqz $r0, drain_fault
    observe_source
drain_fault:
    lwi $r0, [$r6 + 12]
    beq $r0, $r5, completed
    addi $r2, $r2, -1
    bnez $r2, wait_dma
    movi $r0, 4
    j finish
completed:
    swi $r4, [$fp + 44]
    slti $r0, $r10, 8
    bnez $r0, terminal
    slti $r0, $r10, 10
    beqz $r0, terminal
    lwi $r0, [$fp + 96]
    beqz $r0, terminal
    lwi $r0, [$fp + 32]
    bnez $r0, terminal
    lwi $r0, [$r6 - 496]
    bnez $r0, terminal
    j recover_source
terminal:
    addi $r0, $r10, 0
    j finish
recover_source:
    /* The RF chain stays configured. Stop only the SRAM writer, and keep it
     * stopped until the ordered gap marker has left the device DMA engine. */
    load32 $r0, 0x80021090
    movi $r1, 0x4f0
    swi $r1, [$r0]
    movi $r1, 0
    swi $r1, [$fp + 108]
    dsb
    slli $r0, $r5, 6
    load32 $r8, HEADERS
    add $r8, $r8, $r0
    load32 $r0, 0x3800002c
    swi $r0, [$r8]
    swi $r10, [$r8 + 8]
    lwi $r0, [$fp + 104]
    addi $r0, $r0, 1
    swi $r0, [$fp + 104]
    swi $r0, [$r8 + 12]
    load32 $r0, 0x38000014
    swi $r0, [$r8 + 24]
    movi $r0, 0xf6
    swi $r0, [$r8 + 32]
    swi $r4, [$r8 + 36]
    /* Both marker fragments are in MCU data RAM, not the stopped SRAM path. */
    slli $r0, $r5, 4
    add $r0, $r7, $r0
    swi $r8, [$r0]
    addi $r1, $r8, 44
    swi $r1, [$r0 + 8]
    movi $r1, 0
    swi $r1, [$r0 + 12]
    load32 $r1, ((44 << 16) | (1 << 14) | 4)
    swi $r1, [$r0 + 4]
    dsb
    addi $r5, $r5, 1
    andi $r5, $r5, (RING_COUNT - 1)
    swi $r5, [$r6 + 8]
    load32 $r2, 100000000
wait_marker:
    /* Check during stalls as well as the normal packet-publish cadence. */
    andi $r0, $r2, 255
    bnez $r0, tx_watchdog_skip_wait_marker
    jal tx_watchdog_service
tx_watchdog_skip_wait_marker:
    /* Host-preloaded TX-IQ test: sample-timing service is much more
     * frequent than watchdog maintenance. The previous WIP put both in
     * the same /256 poll path, limiting TX to ~39.8 kupdates/s on device.
     * Keep RX ring progress bounded and explicitly report late updates. */
    andi $r0, $r2, 15
    bnez $r0, tx_iq_skip_wait_marker
    jal tx_iq_service
tx_iq_skip_wait_marker:
    lwi $r0, [$r6 + 12]
    beq $r0, $r5, marker_complete
    addi $r2, $r2, -1
    bnez $r2, wait_marker
    movi $r0, 4
    j finish
marker_complete:
    lwi $r0, [$r6 - 496]
    bnez $r0, terminal
    lwi $r0, [$fp + 36]
    bnez $r0, stopped
    j restart_capture
initial_busy:
    movi $r0, 6
finish:
    addi $r10, $r0, 0
    load32 $r0, 0x80021090
    movi $r1, 0x4f0
    swi $r1, [$r0]
    lwi $r1, [$r0]
    swi $r1, [$fp + 68]
    lwi $r1, [$r0 + 36]
    lwi $r2, [$fp + 72]
    swi $r1, [$fp + 72]
    sub $r1, $r1, $r2
    zeh33 $r1, $r1
    lwi $r2, [$fp + 64]
    add $r1, $r1, $r2
    swi $r1, [$fp + 64]
    load32 $r0, 0x89020068
    lwi $r0, [$r0]
    lwi $r1, [$fp + 56]
    sub $r0, $r0, $r1
    swi $r0, [$fp + 48]
    lwi $r0, [$r6 - 496] /* 0x54000110: DMA errors */
    swi $r0, [$fp + 52]
    dsb
    swi $r10, [$fp + 28] /* Publish terminal state after all result fields. */
    dsb
park:
    jal tx_watchdog_service
    jal tx_iq_service
    jal diag_service
    j park
timing_fault:
    movi $r10, 8
    j drain
source_overrun:
    movi $r10, 9
    j drain
    /* Called via JAL from the RX loop. Status 2 stays sticky and keeps
     * disabling GO, so late host register writes cannot leave RF on. Timer
     * wraps modulo 2^32; the armed interval is <=1s (<2^31 ticks). */
tx_watchdog_service:
    lwi $r0, [$fp + TX_WD_STATE]
    beqz $r0, tx_wd_exit
    movi $r3, 2
    beq $r0, $r3, tx_wd_kill
    lwi $r0, [$fp + TX_WD_ARMED_AT]
    load32 $r1, 0x89020068
    lwi $r1, [$r1]
    sub $r1, $r1, $r0
    lwi $r0, [$fp + TX_WD_DURATION_US]
    slt $r3, $r1, $r0
    bnez $r3, tx_wd_exit
    movi $r0, 2
    swi $r0, [$fp + TX_WD_STATE]
    lwi $r1, [$fp + TX_WD_EXPIRATIONS]
    addi $r1, $r1, 1
    swi $r1, [$fp + TX_WD_EXPIRATIONS]
tx_wd_kill:
    tx_shutdown
tx_wd_exit:
    ret5
    /* Host-preloaded bounded table playback, using N9's own us timer.
     * Only r0/r1/r3 clobbered, exactly like the watchdog service.
     * No PA/TSSI/clock/TX source control here; host owns start/rollback.
     * The loop never catches up by writing bursts of late samples: it counts
     * missed intervals as overruns and resumes at the next timer observation.
     * Watchdog is mandatory and can asynchronously disable GO. */
tx_iq_service:
    lwi $r0, [$fp + TX_IQ_CONTROL]
    beqz $r0, tx_iq_exit
    lwi $r0, [$fp + TX_WD_STATE]
    movi $r3, 1
    bne $r0, $r3, tx_iq_disable
    lwi $r0, [$fp + TX_IQ_LENGTH]
    beqz $r0, tx_iq_disable
    slti $r3, $r0, 129
    beqz $r3, tx_iq_disable
    lwi $r0, [$fp + TX_IQ_PERIOD_US]
    beqz $r0, tx_iq_disable
    load32 $r1, 0x89020068
    lwi $r1, [$r1]
    lwi $r3, [$fp + TX_IQ_LAST_TICK]
    sub $r3, $r1, $r3
    slt $r0, $r3, $r0
    bnez $r0, tx_iq_exit
    lwi $r0, [$fp + TX_IQ_MAX_DELTA]
    slt $r0, $r0, $r3
    beqz $r0, tx_iq_max_set
    swi $r3, [$fp + TX_IQ_MAX_DELTA]
tx_iq_max_set:
    /* Late updates are visible and never silently counted as synchronous. */
    lwi $r0, [$fp + TX_IQ_PERIOD_US]
    slli $r0, $r0, 1
    slt $r0, $r0, $r3
    beqz $r0, tx_iq_no_overrun
    lwi $r0, [$fp + TX_IQ_OVERRUN]
    addi $r0, $r0, 1
    swi $r0, [$fp + TX_IQ_OVERRUN]
tx_iq_no_overrun:
    swi $r1, [$fp + TX_IQ_LAST_TICK]
    lwi $r0, [$fp + TX_IQ_INDEX]
    slli $r1, $r0, 2
    load32 $r3, TX_IQ_TABLE
    add $r3, $r3, $r1
    lwi $r1, [$r3]
    load32 $r3, 0x8308fc08
    swi $r1, [$r3]
    addi $r0, $r0, 1
    lwi $r1, [$fp + TX_IQ_LENGTH]
    slt $r3, $r0, $r1
    bnez $r3, tx_iq_no_wrap
    movi $r0, 0
tx_iq_no_wrap:
    swi $r0, [$fp + TX_IQ_INDEX]
    /* Exact MCU time span of emitted samples, not a host sleep estimate.
     * u32 subtraction modulo 2^32 is safe for sub-second burst windows. */
    lwi $r1, [$fp + TX_IQ_LAST_TICK]
    lwi $r0, [$fp + TX_IQ_COMPLETED]
    bnez $r0, tx_iq_has_first_tick
    swi $r1, [$fp + TX_IQ_FIRST_EMIT]
tx_iq_has_first_tick:
    swi $r1, [$fp + TX_IQ_LAST_EMIT]
    addi $r0, $r0, 1
    swi $r0, [$fp + TX_IQ_COMPLETED]
    /* Only observe at first and every 256th emission. Reserve 2 x 128B
     * after the staged table, before USB ring 0x02040000. r2/r4..r10,
     * fp and lp are untouched. No nested call or RF-SPI ownership change. */
    movi $r1, 1
    load32 $r3, TX_ACTIVE_FIRST
    beq $r0, $r1, tx_active_snapshot
    andi $r1, $r0, 255
    bnez $r1, tx_iq_exit
    load32 $r3, TX_ACTIVE_LAST
tx_active_snapshot:
    lwi $r0, [$r3]
    addi $r0, $r0, 1
    swi $r0, [$r3]       /* odd = being written */
    dsb
    lwi $r0, [$fp + TX_IQ_COMPLETED]
    swi $r0, [$r3 + 12]
    load32 $r0, 0x89020068
    lwi $r1, [$r0]
    swi $r1, [$r3 + 4]
    load32 $r0, 0x8300c000
    lwi $r1, [$r0]
    swi $r1, [$r3 + 16]
    load32 $r0, 0x8301c000
    lwi $r1, [$r0]
    swi $r1, [$r3 + 20]
    load32 $r0, 0x8300c508
    lwi $r1, [$r0]
    swi $r1, [$r3 + 24]
    load32 $r0, 0x8301c508
    lwi $r1, [$r0]
    swi $r1, [$r3 + 28]
    load32 $r0, 0x8300c700
    lwi $r1, [$r0]
    swi $r1, [$r3 + 32]
    load32 $r0, 0x8301c700
    lwi $r1, [$r0]
    swi $r1, [$r3 + 36]
    load32 $r0, 0x8300d008
    lwi $r1, [$r0]
    swi $r1, [$r3 + 40]
    load32 $r0, 0x8301d008
    lwi $r1, [$r0]
    swi $r1, [$r3 + 44]
    load32 $r0, 0x8308fc00
    lwi $r1, [$r0]
    swi $r1, [$r3 + 48]
    load32 $r0, 0x8309fc00
    lwi $r1, [$r0]
    swi $r1, [$r3 + 52]
    load32 $r0, 0x8308fc20
    lwi $r1, [$r0]
    swi $r1, [$r3 + 56]
    load32 $r0, 0x8309fc20
    lwi $r1, [$r0]
    swi $r1, [$r3 + 60]
    load32 $r0, 0x8308fc08
    lwi $r1, [$r0]
    swi $r1, [$r3 + 64]
    load32 $r0, 0x8309fc08
    lwi $r1, [$r0]
    swi $r1, [$r3 + 68]
    load32 $r0, 0x830ad4d0
    lwi $r1, [$r0]
    swi $r1, [$r3 + 72]
    load32 $r0, 0x830ad4d8
    lwi $r1, [$r0]
    swi $r1, [$r3 + 76]
    load32 $r0, 0x830000c4
    lwi $r1, [$r0]
    swi $r1, [$r3 + 80]
    load32 $r0, 0x83000108
    lwi $r1, [$r0]
    swi $r1, [$r3 + 84]
    load32 $r0, 0x83000150
    lwi $r1, [$r0]
    swi $r1, [$r3 + 88]
    load32 $r0, 0x830000d0
    lwi $r1, [$r0]
    swi $r1, [$r3 + 92]
    load32 $r0, 0x830100c4
    lwi $r1, [$r0]
    swi $r1, [$r3 + 96]
    load32 $r0, 0x83010108
    lwi $r1, [$r0]
    swi $r1, [$r3 + 100]
    load32 $r0, 0x83010150
    lwi $r1, [$r0]
    swi $r1, [$r3 + 104]
    load32 $r0, 0x830100d0
    lwi $r1, [$r0]
    swi $r1, [$r3 + 108]
    load32 $r0, 0x8300c114
    lwi $r1, [$r0]
    swi $r1, [$r3 + 112]
    load32 $r0, 0x8301c114
    lwi $r1, [$r0]
    swi $r1, [$r3 + 116]
    load32 $r0, 0x89020068
    lwi $r1, [$r0]
    swi $r1, [$r3 + 8]
    dsb
    lwi $r0, [$r3]
    addi $r0, $r0, 1
    swi $r0, [$r3]       /* even = published */
    dsb
    j tx_iq_exit
tx_iq_disable:
    movi $r0, 0
    swi $r0, [$fp + TX_IQ_CONTROL]
tx_iq_exit:
    ret5

/* No-RF receive ring validator and optional staged IQ FIFO producer.
 * All host payloads and packet IDs are checked before DMA descriptor recycle.
 * Mode1 stages 64 signed12 IQ words for a separate timer-paced consumer.
 * Neither mode is a DAC feeder.
 * ABI mailbox +240 enable, +244 sticky fault, +248 slot, +252/256 packet
 * count, +260/264 sample count (both u64), +268/272 first/last tick.
 * Descriptor ring 0x0203c800, four 512B buffers from 0x0203d000.
 * Host disables RX DMA and fences service via diagnostic ACK on teardown. */
ingress_service:
    lwi $r0, [$fp + 240]
    beqz $r0, ingress_exit
    swi $r2, [$fp + 280]
    load32 $r0, 0x5400051c
    lwi $r0, [$r0]
    lwi $r1, [$fp + 248]
    sub $r0, $r0, $r1
    andi $r0, $r0, RX1_RING_MASK
    lwi $r3, [$fp + 276]
    slt $r3, $r3, $r0
    beqz $r3, ingress_no_highwater
    swi $r0, [$fp + 276]
ingress_no_highwater:
    lwi $r1, [$fp + 248]
    slti $r0, $r1, RX1_RING_COUNT
    beqz $r0, ingress_bad
    slli $r1, $r1, 4
    load32 $r3, 0x0203c800
    add $r1, $r1, $r3
    swi $r1, [$fp + 284]
    lwi $r0, [$r1 + 4]
    srli $r3, $r0, 31
    beqz $r3, ingress_return
    dsb
    /* The DMA descriptor reports actual host payload byte count: 336B
     * for monotonic words, 340B for signed12 IQ + trailing digest. */
    lwi $r3, [$fp + 288]
    bnez $r3, ingress_desc_iq
    load32 $r3, 0xc1500000
    j ingress_desc_check
ingress_desc_iq:
    load32 $r3, 0xc1540000
ingress_desc_check:
    bne $r0, $r3, ingress_bad
    lwi $r0, [$fp + 248]
    slli $r0, $r0, 9
    load32 $r3, 0x0203d000
    add $r0, $r0, $r3
    lwi $r1, [$r1]
    bne $r0, $r1, ingress_bad
    lwi $r0, [$r1 + 64]
    load32 $r3, 0x48595131
    bne $r0, $r3, ingress_bad
    lwi $r0, [$r1 + 68]
    lwi $r3, [$fp + 4]
    bne $r0, $r3, ingress_bad
    lwi $r0, [$r1 + 72]
    lwi $r3, [$fp + RX1_CONSUMER_MODE]
    beqz $r3, ingress_old_packet_low
    lwi $r3, [$fp + RX1_STAGE_PACKET_LO]
    j ingress_check_packet_low
ingress_old_packet_low:
    lwi $r3, [$fp + 252]
ingress_check_packet_low:
    bne $r0, $r3, ingress_bad
    lwi $r0, [$r1 + 76]
    lwi $r3, [$fp + RX1_CONSUMER_MODE]
    beqz $r3, ingress_old_packet_high
    lwi $r3, [$fp + RX1_STAGE_PACKET_HI]
    j ingress_check_packet_high
ingress_old_packet_high:
    lwi $r3, [$fp + 256]
ingress_check_packet_high:
    bne $r0, $r3, ingress_bad
    addi $r1, $r1, 80
    /* Mode1 moves validated host IQ into a separately owned N9 FIFO.
     * The 4-slot hardware RX1 DMA ring can then recycle immediately. */
    lwi $r3, [$fp + RX1_CONSUMER_MODE]
    bnez $r3, ingress_stage_store
    /* Mode0: original strictly increasing u32 sequence (counter test).
     * Mode1: arbitrary host-selected signed12 IQ payload; packet-boundary
     * counters are still checked, while a rolling order-sensitive 32-bit
     * checksum covers all 64 exact words. This is an accidental corruption
     * detector, not cryptographic authentication. Host stores the expected
     * checksum in the next u32 after the 64 words, still within 512B slot.
     * Do not consume words into RF here: this path is NO-RF validation. */
    lwi $r3, [$fp + 288]
    bnez $r3, ingress_validate_iq
    lwi $r3, [$fp + 260]
    movi $r2, 64
ingress_validate_word:
    lwi $r0, [$r1]
    bne $r0, $r3, ingress_bad
    addi $r1, $r1, 4
    addi $r3, $r3, 1
    addi $r2, $r2, -1
    bnez $r2, ingress_validate_word
    j ingress_validated_payload
ingress_stage_store:
    /* FIFO full: retain current DONE descriptor, allowing USB hardware to
     * backpressure the host. Never overwrite unconsumed staging bytes. */
    lwi $r0, [$fp + RX1_STAGE_FILL]
    movi $r3, RX1_STAGE_COUNT
    beq $r0, $r3, ingress_return
    swi $r1, [$fp + RX1_STAGE_SRC_SCRATCH]
    lwi $r0, [$fp + RX1_STAGE_PROD]
    slli $r0, $r0, 9
    load32 $r3, RX1_STAGE_BASE
    add $r0, $r0, $r3
    swi $r0, [$fp + RX1_STAGE_DST_SCRATCH]
    load32 $r0, 0x9e3779b9
    swi $r0, [$fp + RX1_STAGE_DIGEST_SCRATCH]
    movi $r2, 64
ingress_stage_copy_word:
    lwi $r1, [$fp + RX1_STAGE_SRC_SCRATCH]
    lwi $r0, [$r1]
    addi $r1, $r1, 4
    swi $r1, [$fp + RX1_STAGE_SRC_SCRATCH]
    lwi $r1, [$fp + RX1_STAGE_DST_SCRATCH]
    swi $r0, [$r1]
    addi $r1, $r1, 4
    swi $r1, [$fp + RX1_STAGE_DST_SCRATCH]
    lwi $r3, [$fp + RX1_STAGE_DIGEST_SCRATCH]
    slli $r1, $r3, 5
    add $r3, $r3, $r1
    add $r3, $r3, $r0
    swi $r3, [$fp + RX1_STAGE_DIGEST_SCRATCH]
    addi $r2, $r2, -1
    bnez $r2, ingress_stage_copy_word
    /* Verify payload BEFORE making it visible to the N9 sample consumer. */
    lwi $r1, [$fp + RX1_STAGE_SRC_SCRATCH]
    lwi $r0, [$r1]
    lwi $r3, [$fp + RX1_STAGE_DIGEST_SCRATCH]
    bne $r0, $r3, ingress_bad
    lwi $r1, [$fp + RX1_STAGE_DST_SCRATCH]
    swi $r3, [$r1] /* trailing 32bit verification tag in staging slot */
    dsb
    lwi $r0, [$fp + RX1_STAGE_PROD]
    addi $r0, $r0, 1
    andi $r0, $r0, RX1_STAGE_MASK
    swi $r0, [$fp + RX1_STAGE_PROD]
    lwi $r0, [$fp + RX1_STAGE_FILL]
    addi $r0, $r0, 1
    swi $r0, [$fp + RX1_STAGE_FILL]
    lwi $r3, [$fp + RX1_STAGE_HIGHWATER]
    slt $r3, $r3, $r0
    beqz $r3, ingress_stage_hwm_done
    swi $r0, [$fp + RX1_STAGE_HIGHWATER]
ingress_stage_hwm_done:
    lwi $r0, [$fp + RX1_STAGE_PACKET_LO]
    addi $r0, $r0, 1
    swi $r0, [$fp + RX1_STAGE_PACKET_LO]
    bnez $r0, ingress_stage_no_packet_carry
    lwi $r0, [$fp + RX1_STAGE_PACKET_HI]
    addi $r0, $r0, 1
    swi $r0, [$fp + RX1_STAGE_PACKET_HI]
ingress_stage_no_packet_carry:
    lwi $r3, [$fp + RX1_STAGE_SAMPLE_LO]
    addi $r0, $r3, 64
    swi $r0, [$fp + RX1_STAGE_SAMPLE_LO]
    slt $r1, $r0, $r3 /* detect u32 carry for ANY starting offset */
    beqz $r1, ingress_stage_no_sample_carry
    lwi $r0, [$fp + RX1_STAGE_SAMPLE_HI]
    addi $r0, $r0, 1
    swi $r0, [$fp + RX1_STAGE_SAMPLE_HI]
ingress_stage_no_sample_carry:
    load32 $r0, 0x89020068
    lwi $r0, [$r0]
    lwi $r1, [$fp + 268]
    bnez $r1, ingress_stage_time_set
    swi $r0, [$fp + 268]
ingress_stage_time_set:
    swi $r0, [$fp + 272]
    j ingress_release_descriptor

ingress_validate_iq:
    load32 $r3, 0x9e3779b9
    movi $r2, 64
ingress_validate_iq_word:
    slli $r0, $r3, 5
    add $r3, $r3, $r0
    lwi $r0, [$r1]
    add $r3, $r3, $r0
    addi $r1, $r1, 4
    addi $r2, $r2, -1
    bnez $r2, ingress_validate_iq_word
    lwi $r0, [$r1]     /* trailing 32-bit checksum, host-generated */
    bne $r3, $r0, ingress_bad
ingress_validated_payload:
    lwi $r3, [$fp + 260]
    addi $r0, $r3, 64
    swi $r0, [$fp + 260]
    slt $r1, $r0, $r3
    beqz $r1, ingress_sample_no_carry
    lwi $r0, [$fp + 264]
    addi $r0, $r0, 1
    swi $r0, [$fp + 264]
ingress_sample_no_carry:
    lwi $r0, [$fp + 252]
    addi $r0, $r0, 1
    swi $r0, [$fp + 252]
    bnez $r0, ingress_packet_no_carry
    lwi $r0, [$fp + 256]
    addi $r0, $r0, 1
    swi $r0, [$fp + 256]
ingress_packet_no_carry:
    load32 $r0, 0x89020068
    lwi $r0, [$r0]
    lwi $r1, [$fp + 268]
    bnez $r1, ingress_has_time
    swi $r0, [$fp + 268]
ingress_has_time:
    swi $r0, [$fp + 272]
ingress_release_descriptor:
    lwi $r1, [$fp + 284]
    load32 $r0, 0x42000000 /* capacity 512, LS0; ownership to DMA */
    swi $r0, [$r1 + 4]
    dsb
    lwi $r1, [$fp + 248]
    load32 $r0, 0x54000518
    swi $r1, [$r0]
    addi $r1, $r1, 1
    andi $r1, $r1, RX1_RING_MASK
    swi $r1, [$fp + 248]
    j ingress_return
ingress_bad:
    movi $r0, 1
    swi $r0, [$fp + 244]
    movi $r0, 0
    swi $r0, [$fp + 240]
    load32 $r0, 0x54000208
    lwi $r1, [$r0]
    bclr $r1, $r1, 2
    swi $r1, [$r0]
    dsb
ingress_return:
    lwi $r2, [$fp + 280]
ingress_exit:
    ret5

/* Timer-paced, NO-RF sample consumer of a separate 16-slot N9 SRAM FIFO.
 * Only r0/r1/r3 clobbered; never touches PHY/PA/GO or DMA descriptors.
 * Each 64-word FIFO slot is immutable once the ingress producer increments
 * fill. The consumer checks its digest a second time before releasing the
 * stage slot; u64 consumed sample counters advance per actual timed read. */
ingress_paced_service:
    /* A separate abort request prevents a host failure from restoring
     * PHY IQ while the N9 writer is still active. No radio GO/path writes. */
    lwi $r0, [$fp + RX1_PHY_DRY_ABORT_REQ]
    beqz $r0, ingress_paced_normal
    lwi $r0, [$fp + RX1_CONSUMER_MODE]
    movi $r3, 2
    beq $r0, $r3, ingress_paced_abort_mode2
    movi $r3, 3
    bne $r0, $r3, ingress_paced_normal
    /* Mode3 host-initiated quiescence: stop RF first if live, then ACK. */
    lwi $r0, [$fp + RX1_RF_ARM]
    movi $r3, 1
    bne $r0, $r3, ingress_paced_abort_mode2
    tx_shutdown
    movi $r0, 0
    swi $r0, [$fp + RX1_RF_ARM]
ingress_paced_abort_mode2:
    movi $r0, 0
    swi $r0, [$fp + 240]
    movi $r0, 5             /* halted by host, explicit terminal state */
    swi $r0, [$fp + RX1_CONSUMER_STATE]
    movi $r0, 1
    swi $r0, [$fp + RX1_PHY_DRY_ABORT_ACK]
    dsb
    ret5
ingress_paced_normal:
    lwi $r0, [$fp + 240]
    beqz $r0, ingress_paced_exit
    lwi $r0, [$fp + RX1_CONSUMER_MODE]
    beqz $r0, ingress_paced_exit
    movi $r3, 3
    bne $r0, $r3, ingress_paced_mode_ready
    /* A mode3 packet may be staged while GO is off, but never consumed
     * until an explicit host-verified no-RF DRY=2 or armed LIVE=1 epoch. */
    lwi $r0, [$fp + RX1_RF_ARM]
    beqz $r0, ingress_paced_exit
    slti $r3, $r0, 3
    beqz $r3, ingress_consumer_bad
ingress_paced_mode_ready:
    lwi $r0, [$fp + RX1_CONSUMER_STATE]
    beqz $r0, ingress_paced_priming
    movi $r3, 1
    beq $r0, $r3, ingress_paced_waiting
    movi $r3, 2
    bne $r0, $r3, ingress_paced_exit
    j ingress_paced_due
ingress_paced_priming:
    /* Start with >=3 prevalidated packets buffered, for host scheduling
     * headroom; don't count normal startup latency as an underrun. */
    lwi $r0, [$fp + RX1_STAGE_FILL]
    slti $r3, $r0, 3
    bnez $r3, ingress_paced_exit
    load32 $r0, 0x89020068
    lwi $r0, [$r0]
    swi $r0, [$fp + RX1_CONSUMER_LAST]
    j ingress_paced_begin_packet
ingress_paced_waiting:
    lwi $r0, [$fp + RX1_STAGE_FILL]
    bnez $r0, ingress_paced_begin_packet
    load32 $r1, 0x89020068
    lwi $r1, [$r1]
    lwi $r3, [$fp + RX1_CONSUMER_LAST]
    sub $r3, $r1, $r3
    lwi $r0, [$fp + RX1_CONSUMER_PERIOD]
    slt $r0, $r3, $r0
    bnez $r0, ingress_paced_exit
    /* All expected samples consumed? Final packet marks state3 immediately,
     * so reaching this label means real starvation. */
    lwi $r0, [$fp + RX1_CONSUMER_UNDERRUN]
    addi $r0, $r0, 1
    swi $r0, [$fp + RX1_CONSUMER_UNDERRUN]
    j ingress_consumer_bad
ingress_paced_begin_packet:
    movi $r0, 0
    swi $r0, [$fp + RX1_CONSUMER_INDEX]
    load32 $r0, 0x9e3779b9
    swi $r0, [$fp + RX1_CONSUMER_DIGEST]
    movi $r0, 2
    swi $r0, [$fp + RX1_CONSUMER_STATE]
ingress_paced_due:
    load32 $r1, 0x89020068
    lwi $r1, [$r1]
    lwi $r3, [$fp + RX1_CONSUMER_LAST]
    sub $r3, $r1, $r3
    lwi $r0, [$fp + RX1_CONSUMER_PERIOD]
    slt $r0, $r3, $r0
    bnez $r0, ingress_paced_exit
    lwi $r0, [$fp + RX1_CONSUMER_MAX_DELTA]
    slt $r0, $r0, $r3
    beqz $r0, ingress_paced_no_max
    swi $r3, [$fp + RX1_CONSUMER_MAX_DELTA]
ingress_paced_no_max:
    lwi $r0, [$fp + RX1_CONSUMER_PERIOD]
    slli $r0, $r0, 1
    slt $r0, $r0, $r3
    beqz $r0, ingress_paced_not_late
    lwi $r0, [$fp + RX1_CONSUMER_LATE]
    addi $r0, $r0, 1
    swi $r0, [$fp + RX1_CONSUMER_LATE]
ingress_paced_not_late:
    /* The legacy B2 scheduler sets last=actual now, accumulating the time
     * spent between 10us deadline and N9 service (observed 96-98k/s mean).
     * Experimental deadline mode instead advances by EXACTLY one requested
     * period per consumed IQ word; lateness is still measured against the
     * absolute schedule. Long stalls may produce closely spaced catch-up
     * reads. RF TX must NOT use this until pacing is independently proven. */
    lwi $r0, [$fp + RX1_CLOCK_MODE]
    beqz $r0, ingress_paced_last_actual
    lwi $r0, [$fp + RX1_CONSUMER_LAST]
    lwi $r3, [$fp + RX1_CONSUMER_PERIOD]
    add $r0, $r0, $r3
    swi $r0, [$fp + RX1_CONSUMER_LAST]
    j ingress_paced_last_done
ingress_paced_last_actual:
    swi $r1, [$fp + RX1_CONSUMER_LAST]
ingress_paced_last_done:
    lwi $r0, [$fp + RX1_CONSUMER_FIRST]
    bnez $r0, ingress_paced_first_seen
    swi $r1, [$fp + RX1_CONSUMER_FIRST]
ingress_paced_first_seen:
    swi $r1, [$fp + RX1_CONSUMER_FINAL]
    /* Read exactly one host-provided signed12 IQ word (no regeneration). */
    lwi $r0, [$fp + RX1_STAGE_CONS]
    slli $r0, $r0, 9
    load32 $r1, RX1_STAGE_BASE
    add $r1, $r1, $r0
    lwi $r0, [$fp + RX1_CONSUMER_INDEX]
    slli $r0, $r0, 2
    add $r1, $r1, $r0
    lwi $r0, [$r1]
    /* B3 experimental dry bridge from new host sample to actual PHY
     * IQ SETPOINT. No GO/path/PA enabling and no TX watchdog ARM. Do not
     * use these writes as evidence of analog RF modulation. Ensure no
     * other writer owns the TX hardware, and fail-closed on an active TX. */
    lwi $r3, [$fp + RX1_CONSUMER_MODE]
    movi $r1, 2
    beq $r3, $r1, ingress_paced_dry_begin
    movi $r1, 3
    beq $r3, $r1, ingress_paced_rf_begin
    j ingress_paced_dry_done
ingress_paced_dry_begin:
    swi $r0, [$fp + RX1_PHY_DRY_LAST]
    /* MMIO reads from multiple RF blocks on every 10us service event
     * caused RX source starvation and USB stall in the first live test.
     * The host already checks baseline once under the USB control mutex.
     * Recheck generator/PA/ownership at the first word of each immutable
     * 64-word packet (~640us), not on every I/Q sample. No RF generator GO
     * is ever enabled by this test. */
    lwi $r1, [$fp + RX1_CONSUMER_INDEX]
    bnez $r1, ingress_paced_dry_guards_done
    lwi $r1, [$fp + TX_IQ_CONTROL]
    bnez $r1, ingress_consumer_bad
    lwi $r1, [$fp + TX_WD_STATE]
    bnez $r1, ingress_consumer_bad
    load32 $r1, 0x8308fc00
    lwi $r1, [$r1]
    load32 $r3, 0x40000000
    and $r1, $r1, $r3
    bnez $r1, ingress_consumer_bad
    load32 $r1, 0x8308fc20
    lwi $r1, [$r1]
    bnez $r1, ingress_consumer_bad
    load32 $r1, 0x8300c508
    lwi $r1, [$r1]
    load32 $r3, 0x80008000
    and $r1, $r1, $r3
    bnez $r1, ingress_consumer_bad
ingress_paced_dry_guards_done:
    lwi $r0, [$fp + RX1_PHY_DRY_LAST]
    load32 $r1, 0x8308fc08
    swi $r0, [$r1]
    lwi $r1, [$fp + RX1_PHY_DRY_WRITES]
    addi $r1, $r1, 1
    swi $r1, [$fp + RX1_PHY_DRY_WRITES]
    j ingress_paced_dry_done
ingress_paced_rf_begin:
    /* Both mode3 dry=2 and live=1 read one source-defined IQ word from
     * the 16-slot FIFO, with post-packet checksum verified below. The live
     * case has a short explicit watchdog+GO+path ownership gate. */
    swi $r0, [$fp + RX1_RF_LAST]
    lwi $r1, [$fp + RX1_CONSUMER_INDEX]
    bnez $r1, ingress_paced_rf_guards_done
    lwi $r1, [$fp + TX_IQ_CONTROL]
    bnez $r1, ingress_consumer_bad
    lwi $r1, [$fp + RX1_RF_ARM]
    movi $r3, 1
    beq $r1, $r3, ingress_paced_rf_live_guards
    movi $r3, 2
    bne $r1, $r3, ingress_consumer_bad
    /* Dry mode3 MUST have no RF/WD enabled. */
    lwi $r1, [$fp + TX_WD_STATE]
    bnez $r1, ingress_consumer_bad
    load32 $r1, 0x8308fc00
    lwi $r1, [$r1]
    load32 $r3, 0x40000000
    and $r1, $r1, $r3
    bnez $r1, ingress_consumer_bad
    load32 $r1, 0x8308fc20
    lwi $r1, [$r1]
    bnez $r1, ingress_consumer_bad
    j ingress_paced_rf_guards_done
ingress_paced_rf_live_guards:
    lwi $r1, [$fp + TX_WD_STATE]
    movi $r3, 1
    bne $r1, $r3, ingress_consumer_bad
    load32 $r1, 0x8308fc00
    lwi $r1, [$r1]
    load32 $r3, 0x40000000
    and $r1, $r1, $r3
    beqz $r1, ingress_consumer_bad
    load32 $r1, 0x8308fc20
    lwi $r1, [$r1]
    movi $r3, 1
    bne $r1, $r3, ingress_consumer_bad
ingress_paced_rf_guards_done:
    /* Host must explicitly enable LIVE after its watchdog-arm ACK and
     * first PHY GO. No accidental writes during DRY=2 or after ARM=0. */
    lwi $r1, [$fp + RX1_RF_ARM]
    movi $r3, 1
    bne $r1, $r3, ingress_paced_rf_no_write
    lwi $r0, [$fp + RX1_RF_LAST]
    load32 $r1, 0x8308fc08
    swi $r0, [$r1]
    lwi $r0, [$fp + RX1_RF_WRITES]
    bnez $r0, ingress_paced_rf_had_first
    load32 $r1, 0x89020068
    lwi $r1, [$r1]
    swi $r1, [$fp + RX1_RF_FIRST_TICK]
ingress_paced_rf_had_first:
    load32 $r1, 0x89020068
    lwi $r1, [$r1]
    swi $r1, [$fp + RX1_RF_LAST_TICK]
    addi $r0, $r0, 1
    swi $r0, [$fp + RX1_RF_WRITES]
    lwi $r0, [$fp + RX1_RF_LAST]
    j ingress_paced_dry_done
ingress_paced_rf_no_write:
    /* The shared digest step must see the original signed12 IQ word even
     * in NO-RF mode3 DRY=2, never the last mailbox/guard value. */
    lwi $r0, [$fp + RX1_RF_LAST]
ingress_paced_dry_done:
    lwi $r3, [$fp + RX1_CONSUMER_DIGEST]
    slli $r1, $r3, 5
    add $r3, $r3, $r1
    add $r3, $r3, $r0
    swi $r3, [$fp + RX1_CONSUMER_DIGEST]
    lwi $r0, [$fp + 260]
    addi $r0, $r0, 1
    swi $r0, [$fp + 260]
    bnez $r0, ingress_paced_no_carry
    lwi $r0, [$fp + 264]
    addi $r0, $r0, 1
    swi $r0, [$fp + 264]
ingress_paced_no_carry:
    lwi $r0, [$fp + RX1_CONSUMER_INDEX]
    addi $r0, $r0, 1
    swi $r0, [$fp + RX1_CONSUMER_INDEX]
    movi $r3, 64
    bne $r0, $r3, ingress_paced_exit
    /* Post-consumption integrity: slot could not be recycled before here. */
    lwi $r0, [$fp + RX1_STAGE_CONS]
    slli $r0, $r0, 9
    load32 $r1, 0x0203e100
    add $r1, $r1, $r0
    lwi $r0, [$r1]
    lwi $r3, [$fp + RX1_CONSUMER_DIGEST]
    bne $r0, $r3, ingress_consumer_bad
    lwi $r0, [$fp + 252]
    addi $r0, $r0, 1
    swi $r0, [$fp + 252]
    bnez $r0, ingress_paced_no_packet_carry
    lwi $r0, [$fp + 256]
    addi $r0, $r0, 1
    swi $r0, [$fp + 256]
ingress_paced_no_packet_carry:
    lwi $r0, [$fp + RX1_STAGE_CONS]
    addi $r0, $r0, 1
    andi $r0, $r0, RX1_STAGE_MASK
    swi $r0, [$fp + RX1_STAGE_CONS]
    lwi $r0, [$fp + RX1_STAGE_FILL]
    addi $r0, $r0, -1
    swi $r0, [$fp + RX1_STAGE_FILL]
    lwi $r0, [$fp + 260]
    lwi $r3, [$fp + RX1_CONSUMER_EXPECT_LO]
    bne $r0, $r3, ingress_paced_next_packet
    lwi $r0, [$fp + 264]
    lwi $r3, [$fp + RX1_CONSUMER_EXPECT_HI]
    bne $r0, $r3, ingress_paced_next_packet
    movi $r0, 3
    swi $r0, [$fp + RX1_CONSUMER_STATE]
    j ingress_paced_exit
ingress_paced_next_packet:
    movi $r0, 1
    swi $r0, [$fp + RX1_CONSUMER_STATE]
    j ingress_paced_exit
ingress_consumer_bad:
    /* Fail-closed on any live-RF consumer integrity/underrun fault: stop
     * RF GO/path before reporting sticky error, without caller invention. */
    lwi $r0, [$fp + RX1_CONSUMER_MODE]
    movi $r3, 3
    bne $r0, $r3, ingress_consumer_bad_no_rf
    lwi $r0, [$fp + RX1_RF_ARM]
    movi $r3, 1
    bne $r0, $r3, ingress_consumer_bad_no_rf
    tx_shutdown
    movi $r0, 0
    swi $r0, [$fp + RX1_RF_ARM]
ingress_consumer_bad_no_rf:
    movi $r0, 2
    swi $r0, [$fp + 244]
    movi $r0, 4
    swi $r0, [$fp + RX1_CONSUMER_STATE]
    movi $r0, 0
    swi $r0, [$fp + 240]
    load32 $r0, 0x54000208
    lwi $r1, [$r0]
    bclr $r1, $r1, 2
    swi $r1, [$r0]
    dsb
ingress_paced_exit:
    ret5

    /* Host-to-N9 diagnostic RPC, serviced from wait_go, publish and park.
     * Exactly one request in flight. HOST: set ARG, CMD, then REQ_SEQ last.
     * DEVICE: publish RESULT, STATUS, then DONE_SEQ last after DSB.
     * $r0/$r1/$r3 only; preserve $r2 countdown, RX DMA $r6..$r10,
     * firmware stream sequence $r4, slot index $r5, and frame pointer.
     * Command 3 (eFuse) fails closed until an independently verified
     * direct-controller or ROM-service implementation exists.
     */
diag_service:
    lwi $r0, [$fp + DIAG_REQ_SEQ]
    lwi $r1, [$fp + DIAG_DONE_SEQ]
    beq $r0, $r1, diag_exit
    lwi $r1, [$fp + DIAG_CMD]
    movi $r3, 1
    beq $r1, $r3, diag_version
    movi $r3, 2
    beq $r1, $r3, diag_stat
    movi $r3, 4
    beq $r1, $r3, diag_caps
    movi $r3, 3
    beq $r1, $r3, diag_rom_efuse_probe
    movi $r3, 5
    beq $r1, $r3, diag_tx_wd_arm
    movi $r3, 6
    beq $r1, $r3, diag_tx_wd_disarm
    movi $r3, 7
    beq $r1, $r3, diag_tx_wd_status
    movi $r3, 8
    beq $r1, $r3, diag_dma_timing
    movi $r3, 9
    beq $r1, $r3, diag_dma_dual_phy
    movi $r3, 11
    beq $r1, $r3, diag_rom_tx_path23_start
    /* Unknown commands remain unsupported. */
    movi $r1, 2
    j diag_reply_status
/* Command 8: bounded, RF-off local timing of a host-prequalified channel3
 * transfer. No address/length is accepted from the caller: argument selects
 * one of three exact previously qualified 4 KiB transfer configurations.
 * Host retains exclusive ownership and all original rollback obligations.
 * Preserve RX loop registers. No RF enabling, no autonomous rearm.
 */
diag_dma_timing:
    lwi $r1, [$fp + TX_WD_STATE]
    bnez $r1, diag_dma_invalid
    lwi $r1, [$fp + TX_IQ_CONTROL]
    bnez $r1, diag_dma_invalid
    lwi $r1, [$fp + RX1_CONSUMER_MODE]
    bnez $r1, diag_dma_invalid
    lwi $r1, [$fp + RX1_RF_ARM]
    bnez $r1, diag_dma_invalid
    lwi $r1, [$fp + 240]
    bnez $r1, diag_dma_invalid
    load32 $r0, 0x8308fc00
    load32 $r3, 0x40000000
    lwi $r1, [$r0]
    and $r1, $r1, $r3
    bnez $r1, diag_dma_invalid
    lwi $r1, [$r0 + 0x20]
    bnez $r1, diag_dma_invalid
    load32 $r0, 0x8309fc00
    lwi $r1, [$r0]
    and $r1, $r1, $r3
    bnez $r1, diag_dma_invalid
    lwi $r1, [$r0 + 0x20]
    bnez $r1, diag_dma_invalid
    load32 $r0, 0x80010300
    lwi $r1, [$r0 + 0x00]
    bnez $r1, diag_dma_invalid
    lwi $r1, [$r0 + 0x04]
    bnez $r1, diag_dma_invalid
    lwi $r1, [$r0 + 0x08]
    bnez $r1, diag_dma_invalid
    lwi $r1, [$r0 + 0x1c]
    load32 $r3, 0x0203e000
    bne $r1, $r3, diag_dma_invalid
    /* Dedicated dry-only burst1 probe: mode 3, exact scalar destination,
     * first 64 bytes, then 4096. No RF-on option or arbitrary MMIO. */
    lwi $r1, [$fp + DIAG_ARG]
    movi $r3, 3
    beq $r1, $r3, diag_dma_phy_burst1
    movi $r3, 4
    beq $r1, $r3, diag_dma_phy_second
    lwi $r1, [$r0 + 0x24]
    load32 $r3, 4096
    bne $r1, $r3, diag_dma_invalid
    lwi $r1, [$fp + DIAG_ARG]
    beqz $r1, diag_dma_fixed_sram
    movi $r3, 1
    beq $r1, $r3, diag_dma_inc_sram
    movi $r3, 2
    bne $r1, $r3, diag_dma_invalid
    load32 $r3, 0x8308fc08
    lwi $r1, [$r0 + 0x20]
    bne $r1, $r3, diag_dma_invalid
    load32 $r3, 0x22000008
    j diag_dma_config_check
diag_dma_phy_burst1:
    lwi $r1, [$r0 + 0x24]
    movi $r3, 64
    beq $r1, $r3, diag_dma_phy_burst1_dst
    load32 $r3, 4096
    bne $r1, $r3, diag_dma_invalid
diag_dma_phy_burst1_dst:
    load32 $r3, 0x8308fc08
    lwi $r1, [$r0 + 0x20]
    bne $r1, $r3, diag_dma_invalid
    load32 $r3, 0x22010008
    j diag_dma_config_check
diag_dma_phy_second:
    /* Only the exact second, stock-WM-written scalar IQ latch; burst=0.
     * Both GO and PATH remain disabled by the common command-8 gate. */
    lwi $r1, [$r0 + 0x24]
    movi $r3, 64
    beq $r1, $r3, diag_dma_phy_second_dst
    load32 $r3, 4096
    bne $r1, $r3, diag_dma_invalid
diag_dma_phy_second_dst:
    load32 $r3, 0x8309fc08
    lwi $r1, [$r0 + 0x20]
    bne $r1, $r3, diag_dma_invalid
    load32 $r3, 0x22000008
    j diag_dma_config_check
diag_dma_fixed_sram:
    load32 $r3, 0x0203f000
diag_dma_fixed_check:
    lwi $r1, [$r0 + 0x20]
    bne $r1, $r3, diag_dma_invalid
    load32 $r3, 0x22000008
    j diag_dma_sram_config_check
diag_dma_inc_sram:
    load32 $r3, 0x0203f000
    lwi $r1, [$r0 + 0x20]
    bne $r1, $r3, diag_dma_invalid
    load32 $r3, 0x22000000
diag_dma_sram_config_check:
    /* Allow only ROM-qualified BURST[18:16] variation in SRAM tests.
     * PHY fixed configuration uses separately enumerated strict equality. */
    lwi $r1, [$r0 + 0x18]
    xor $r1, $r1, $r3
    load32 $r3, 0xfff8ffff
    and $r1, $r1, $r3
    bnez $r1, diag_dma_invalid
    j diag_dma_config_ok
diag_dma_config_check:
    lwi $r1, [$r0 + 0x18]
    bne $r1, $r3, diag_dma_invalid
diag_dma_config_ok:
    swi $r2, [$fp + OTP_SAVE_R2]
    swi $r4, [$fp + OTP_SAVE_R4]
    swi $r5, [$fp + OTP_SAVE_R5]
    load32 $r4, 0x89020068
    load32 $r2, 16384
    movi $r1, 1
    dsb
    lwi $r5, [$r4]
    swi $r1, [$r0 + 0x08]
    dsb
diag_dma_poll:
    lwi $r1, [$r0 + 0x08]
    lwi $r3, [$r4]
    sub $r3, $r3, $r5
    andi $r1, $r1, 1
    beqz $r1, diag_dma_complete
    slti $r1, $r3, 600
    beqz $r1, diag_dma_timeout
    addi $r2, $r2, -1
    bnez $r2, diag_dma_poll
diag_dma_timeout:
    /* RF remains disabled. Host sees failure and stops/flushes before
     * restoration; do not claim completion on timeout. */
    movi $r1, 4
    j diag_dma_restore
diag_dma_complete:
    swi $r3, [$fp + DIAG_RESULT]
    movi $r1, 0
diag_dma_restore:
    lwi $r2, [$fp + OTP_SAVE_R2]
    lwi $r4, [$fp + OTP_SAVE_R4]
    lwi $r5, [$fp + OTP_SAVE_R5]
    lwi $r0, [$fp + DIAG_REQ_SEQ]
    j diag_reply_status
diag_dma_invalid:
    movi $r1, 3
    lwi $r0, [$fp + DIAG_REQ_SEQ]
    j diag_reply_status
/* Command 11: shielded/manual-only stock-ROM TX path initializer.
 * ARG is exact ROM path selector 0..3; mode/subtype bytes are zero.
 * Host owns complete snapshot/rollback and has already armed watchdog.
 * No autonomous repeat/rearm. This wrapper exists only to qualify the exact
 * immutable ROM path initialization before any DBDC implementation. */
diag_rom_tx_path23_start:
    lwi $r1, [$fp + DIAG_ARG]
    slti $r3, $r1, 4
    beqz $r3, diag_dma_dual_invalid
diag_rom_tx_path23_arg_ok:
    lwi $r1, [$fp + TX_WD_STATE]
    movi $r3, 1
    bne $r1, $r3, diag_dma_dual_invalid
    lwi $r1, [$fp + TX_IQ_CONTROL]
    bnez $r1, diag_dma_dual_invalid
    load32 $r0, 0x8308fc00
    lwi $r1, [$r0]
    load32 $r3, 0x40000000
    and $r1, $r1, $r3
    bnez $r1, diag_dma_dual_invalid
    lwi $r1, [$r0 + 0x20]
    bnez $r1, diag_dma_dual_invalid
    load32 $r0, 0x8309fc00
    lwi $r1, [$r0]
    and $r1, $r1, $r3
    bnez $r1, diag_dma_dual_invalid
    lwi $r1, [$r0 + 0x20]
    bnez $r1, diag_dma_dual_invalid
    /* Preserve caller-owned N9 state across ROM ABI call. */
    swi $r2, [$fp + OTP_SAVE_R2]
    swi $r4, [$fp + OTP_SAVE_R4]
    swi $r5, [$fp + OTP_SAVE_R5]
    swi $lp, [$fp + OTP_SAVE_LP]
    lwi $r0, [$fp + DIAG_ARG]
    load32 $r5, 0x0083f074
    jral5 $r5
    lwi $r2, [$fp + OTP_SAVE_R2]
    lwi $r4, [$fp + OTP_SAVE_R4]
    lwi $r5, [$fp + OTP_SAVE_R5]
    lwi $lp, [$fp + OTP_SAVE_LP]
    /* Require ROM to have started both mirrored PHY generators/tonepaths. */
    load32 $r0, 0x8308fc00
    lwi $r1, [$r0]
    load32 $r3, 0x40000000
    and $r1, $r1, $r3
    beqz $r1, diag_dma_dual_invalid
    lwi $r1, [$r0 + 0x20]
    beqz $r1, diag_dma_dual_invalid
    load32 $r0, 0x8309fc00
    lwi $r1, [$r0]
    and $r1, $r1, $r3
    beqz $r1, diag_dma_dual_invalid
    lwi $r1, [$r0 + 0x20]
    beqz $r1, diag_dma_dual_invalid
    load32 $r0, 0x830ad4d0
    lwi $r1, [$r0]
    swi $r1, [$fp + DIAG_RESULT]
    movi $r1, 0
    lwi $r0, [$fp + DIAG_REQ_SEQ]
    j diag_reply_status

/* Command 9: launch two already-programmed, exact 4-KiB fixed-destination
 * DMA transfers to the two source-verified PHY scalar IQ banks. ARG=0 requires
 * both TX banks OFF. ARG=1 is reserved for the manual bounded 2TX experiment
 * and requires watchdog ARMED plus both GO/tonepath/path gates active. Host must
 * back up both sources, scalar latches and DMA channels in either mode.
 * RESULT bit31 confirms both START bits were observed set after the second
 * START write; low31 is device-timer elapsed microseconds until both idle. */
diag_dma_dual_phy:
    lwi $r1, [$fp + DIAG_ARG]
    beqz $r1, diag_dma_dual_offline
    movi $r3, 1
    beq $r1, $r3, diag_dma_dual_live
    movi $r3, 2
    beq $r1, $r3, diag_dma_dual_sram_fixed
    movi $r3, 3
    beq $r1, $r3, diag_dma_dual_sram_inc_burst1
    j diag_dma_dual_invalid
diag_dma_dual_live:
    /* Live 2TX mode: host already armed the device watchdog and owns GO. */
    lwi $r1, [$fp + TX_WD_STATE]
    bne $r1, $r3, diag_dma_dual_invalid
    lwi $r1, [$fp + TX_IQ_CONTROL]
    bnez $r1, diag_dma_dual_invalid
    lwi $r1, [$fp + RX1_CONSUMER_MODE]
    bnez $r1, diag_dma_dual_invalid
    lwi $r1, [$fp + RX1_RF_ARM]
    bnez $r1, diag_dma_dual_invalid
    lwi $r1, [$fp + 240]
    bnez $r1, diag_dma_dual_invalid
    load32 $r0, 0x8308fc00
    load32 $r3, 0x40000000
    lwi $r1, [$r0]
    and $r1, $r1, $r3
    beqz $r1, diag_dma_dual_invalid
    lwi $r1, [$r0 + 0x20]
    beqz $r1, diag_dma_dual_invalid
    load32 $r0, 0x8309fc00
    lwi $r1, [$r0]
    and $r1, $r1, $r3
    beqz $r1, diag_dma_dual_invalid
    lwi $r1, [$r0 + 0x20]
    beqz $r1, diag_dma_dual_invalid
    load32 $r0, 0x8300d008
    lwi $r1, [$r0]
    load32 $r3, 0x00001dd0
    bne $r1, $r3, diag_dma_dual_invalid
    load32 $r0, 0x8301d008
    lwi $r1, [$r0]
    bne $r1, $r3, diag_dma_dual_invalid
    j diag_dma_dual_config
diag_dma_dual_offline:
    lwi $r1, [$fp + TX_WD_STATE]
    bnez $r1, diag_dma_dual_invalid
    lwi $r1, [$fp + TX_IQ_CONTROL]
    bnez $r1, diag_dma_dual_invalid
    lwi $r1, [$fp + RX1_CONSUMER_MODE]
    bnez $r1, diag_dma_dual_invalid
    lwi $r1, [$fp + RX1_RF_ARM]
    bnez $r1, diag_dma_dual_invalid
    lwi $r1, [$fp + 240]
    bnez $r1, diag_dma_dual_invalid
    load32 $r0, 0x8308fc00
    load32 $r3, 0x40000000
    lwi $r1, [$r0]
    and $r1, $r1, $r3
    bnez $r1, diag_dma_dual_invalid
    lwi $r1, [$r0 + 0x20]
    bnez $r1, diag_dma_dual_invalid
    load32 $r0, 0x8309fc00
    lwi $r1, [$r0]
    and $r1, $r1, $r3
    bnez $r1, diag_dma_dual_invalid
    lwi $r1, [$r0 + 0x20]
    bnez $r1, diag_dma_dual_invalid
    j diag_dma_dual_config
diag_dma_dual_sram_fixed:
    /* RF-off paired SRAM control, exact 256-byte table halves, fixed dest. */
    lwi $r1, [$fp + TX_WD_STATE]
    bnez $r1, diag_dma_dual_invalid
    load32 $r0, 0x8308fc00
    load32 $r3, 0x40000000
    lwi $r1, [$r0]
    and $r1, $r1, $r3
    bnez $r1, diag_dma_dual_invalid
    lwi $r1, [$r0 + 0x20]
    bnez $r1, diag_dma_dual_invalid
    load32 $r0, 0x8309fc00
    lwi $r1, [$r0]
    and $r1, $r1, $r3
    bnez $r1, diag_dma_dual_invalid
    lwi $r1, [$r0 + 0x20]
    bnez $r1, diag_dma_dual_invalid
    j diag_dma_dual_config
diag_dma_dual_sram_inc_burst1:
    /* Same exact source/destination regions, incrementing destination, burst1. */
    lwi $r1, [$fp + TX_WD_STATE]
    bnez $r1, diag_dma_dual_invalid
    load32 $r0, 0x8308fc00
    load32 $r3, 0x40000000
    lwi $r1, [$r0]
    and $r1, $r1, $r3
    bnez $r1, diag_dma_dual_invalid
    lwi $r1, [$r0 + 0x20]
    bnez $r1, diag_dma_dual_invalid
    load32 $r0, 0x8309fc00
    lwi $r1, [$r0]
    and $r1, $r1, $r3
    bnez $r1, diag_dma_dual_invalid
    lwi $r1, [$r0 + 0x20]
    bnez $r1, diag_dma_dual_invalid
diag_dma_dual_config:
    /* DIAG_ARG mode: 0/1 PHY 4KiB; 2 SRAM fixed 256B; 3 SRAM inc burst1 256B. */
    load32 $r0, 0x80010200
    lwi $r1, [$r0 + 0x00]
    bnez $r1, diag_dma_dual_invalid
    lwi $r1, [$r0 + 0x04]
    bnez $r1, diag_dma_dual_invalid
    lwi $r1, [$r0 + 0x08]
    bnez $r1, diag_dma_dual_invalid
    lwi $r1, [$r0 + 0x1c]
    load32 $r3, 0x0203e000
    bne $r1, $r3, diag_dma_dual_invalid
    lwi $r1, [$fp + DIAG_ARG]
    beqz $r1, diag_dma_dual_ch2_phy
    movi $r3, 1
    beq $r1, $r3, diag_dma_dual_ch2_phy
    /* SRAM modes use two non-overlapping halves of qualified TX_IQ_TABLE. */
    lwi $r1, [$r0 + 0x20]
    load32 $r3, 0x0203c000
    bne $r1, $r3, diag_dma_dual_invalid
    lwi $r1, [$r0 + 0x24]
    movi $r3, 256
    bne $r1, $r3, diag_dma_dual_invalid
    lwi $r1, [$fp + DIAG_ARG]
    movi $r3, 2
    beq $r1, $r3, diag_dma_dual_ch2_fixed_check
    lwi $r1, [$r0 + 0x18]
    load32 $r3, 0x22010000
    bne $r1, $r3, diag_dma_dual_invalid
    j diag_dma_dual_ch3
diag_dma_dual_ch2_fixed_check:
    lwi $r1, [$r0 + 0x18]
    load32 $r3, 0x22000008
    bne $r1, $r3, diag_dma_dual_invalid
    j diag_dma_dual_ch3
diag_dma_dual_ch2_phy:
    lwi $r1, [$r0 + 0x20]
    load32 $r3, 0x8308fc08
    bne $r1, $r3, diag_dma_dual_invalid
    lwi $r1, [$r0 + 0x24]
    load32 $r3, 4096
    bne $r1, $r3, diag_dma_dual_invalid
    lwi $r1, [$r0 + 0x18]
    load32 $r3, 0x22000008
    bne $r1, $r3, diag_dma_dual_invalid
diag_dma_dual_ch3:
    load32 $r3, 0x80010300
    lwi $r1, [$r3 + 0x00]
    bnez $r1, diag_dma_dual_invalid
    lwi $r1, [$r3 + 0x04]
    bnez $r1, diag_dma_dual_invalid
    lwi $r1, [$r3 + 0x08]
    bnez $r1, diag_dma_dual_invalid
    lwi $r1, [$r3 + 0x1c]
    load32 $r2, 0x0203f000
    bne $r1, $r2, diag_dma_dual_invalid
    lwi $r1, [$fp + DIAG_ARG]
    beqz $r1, diag_dma_dual_ch3_phy
    movi $r2, 1
    beq $r1, $r2, diag_dma_dual_ch3_phy
    lwi $r1, [$r3 + 0x20]
    load32 $r2, 0x0203c100
    bne $r1, $r2, diag_dma_dual_invalid
    lwi $r1, [$r3 + 0x24]
    movi $r2, 256
    bne $r1, $r2, diag_dma_dual_invalid
    lwi $r1, [$fp + DIAG_ARG]
    movi $r2, 2
    beq $r1, $r2, diag_dma_dual_ch3_fixed_check
    lwi $r1, [$r3 + 0x18]
    load32 $r2, 0x22010000
    bne $r1, $r2, diag_dma_dual_invalid
    j diag_dma_dual_launch
diag_dma_dual_ch3_fixed_check:
    lwi $r1, [$r3 + 0x18]
    load32 $r2, 0x22000008
    bne $r1, $r2, diag_dma_dual_invalid
    j diag_dma_dual_launch
diag_dma_dual_ch3_phy:
    lwi $r1, [$r3 + 0x20]
    load32 $r2, 0x8309fc08
    bne $r1, $r2, diag_dma_dual_invalid
    lwi $r1, [$r3 + 0x24]
    load32 $r2, 4096
    bne $r1, $r2, diag_dma_dual_invalid
    lwi $r1, [$r3 + 0x18]
    load32 $r2, 0x22000008
    bne $r1, $r2, diag_dma_dual_invalid
diag_dma_dual_launch:
    /* Preserve RX-loop registers beyond the documented diag clobber set. */
    swi $r2, [$fp + OTP_SAVE_R2]
    swi $r4, [$fp + OTP_SAVE_R4]
    swi $r5, [$fp + OTP_SAVE_R5]
    load32 $r2, 0x89020068
    lwi $r4, [$r2]
    movi $r1, 1
    dsb
    swi $r1, [$r0 + 0x08]
    swi $r1, [$r3 + 0x08]
    dsb
    /* Both must still be active after the paired launch (<1 us apart). */
    lwi $r1, [$r0 + 0x08]
    andi $r1, $r1, 1
    beqz $r1, diag_dma_dual_timeout
    lwi $r1, [$r3 + 0x08]
    andi $r1, $r1, 1
    beqz $r1, diag_dma_dual_timeout
    load32 $r5, 32768
diag_dma_dual_poll:
    lwi $r1, [$r0 + 0x08]
    lwi $r5, [$r3 + 0x08]
    or $r1, $r1, $r5
    andi $r1, $r1, 1
    lwi $r5, [$r2]
    sub $r5, $r5, $r4
    beqz $r1, diag_dma_dual_complete
    slti $r1, $r5, 1200
    beqz $r1, diag_dma_dual_timeout
    j diag_dma_dual_poll
diag_dma_dual_complete:
    load32 $r1, 0x80000000
    or $r5, $r5, $r1
    swi $r5, [$fp + DIAG_RESULT]
    movi $r1, 0
    j diag_dma_dual_restore
diag_dma_dual_timeout:
    movi $r1, 4
diag_dma_dual_restore:
    lwi $r2, [$fp + OTP_SAVE_R2]
    lwi $r4, [$fp + OTP_SAVE_R4]
    lwi $r5, [$fp + OTP_SAVE_R5]
    lwi $r0, [$fp + DIAG_REQ_SEQ]
    j diag_reply_status
diag_dma_dual_invalid:
    movi $r1, 3
    lwi $r0, [$fp + DIAG_REQ_SEQ]
    j diag_reply_status
/* Command 5 arms only from a disarmed state. 20ms..1s, in N9 timer
 * microseconds; authorization, RF containment and baseline checks live on
 * host side. Arm is ACKed before the host enables generator GO. */
diag_tx_wd_arm:
    lwi $r1, [$fp + TX_WD_STATE]
    bnez $r1, diag_tx_wd_invalid
    lwi $r1, [$fp + DIAG_ARG]
    load32 $r3, 20000
    slt $r0, $r1, $r3
    bnez $r0, diag_tx_wd_invalid
    load32 $r3, 1000000
    slt $r0, $r3, $r1
    bnez $r0, diag_tx_wd_invalid
    load32 $r0, 0x89020068
    lwi $r0, [$r0]
    swi $r0, [$fp + TX_WD_ARMED_AT]
    swi $r1, [$fp + TX_WD_DURATION_US]
    movi $r1, 1
    swi $r1, [$fp + TX_WD_STATE]
    dsb
    lwi $r0, [$fp + DIAG_REQ_SEQ]
    movi $r1, 0
    j diag_reply_status
diag_tx_wd_invalid:
    movi $r1, 3
    lwi $r0, [$fp + DIAG_REQ_SEQ]
    j diag_reply_status
/* Disarm performs RF shutdown first, regardless of the previous state.
 * The host must already have restored its complete baseline. */
diag_tx_wd_disarm:
    /* Disarm only after the host has explicitly stopped the generator.
     * If either GO or tone path remains enabled, force it OFF and refuse
     * the request; the watchdog remains armed/expired until a safe stop. */
    load32 $r0, 0x8308fc00
    lwi $r1, [$r0]
    load32 $r3, 0x40000000
    and $r1, $r1, $r3
    bnez $r1, diag_tx_wd_disarm_unsafe
    load32 $r0, 0x8308fc20
    lwi $r1, [$r0]
    bnez $r1, diag_tx_wd_disarm_unsafe
    load32 $r0, 0x8309fc00
    lwi $r1, [$r0]
    and $r1, $r1, $r3
    bnez $r1, diag_tx_wd_disarm_unsafe
    load32 $r0, 0x8309fc20
    lwi $r1, [$r0]
    bnez $r1, diag_tx_wd_disarm_unsafe
    /* Exact ROM TX-active path state sets D008 bit6; refuse disarm if any
     * logical path 0..3 still has that bit asserted. */
    movi $r3, 0x40
    load32 $r0, 0x8300d008
    lwi $r1, [$r0]
    and $r1, $r1, $r3
    bnez $r1, diag_tx_wd_disarm_unsafe
    load32 $r0, 0x8301d008
    lwi $r1, [$r0]
    and $r1, $r1, $r3
    bnez $r1, diag_tx_wd_disarm_unsafe
    load32 $r0, 0x8302d008
    lwi $r1, [$r0]
    and $r1, $r1, $r3
    bnez $r1, diag_tx_wd_disarm_unsafe
    load32 $r0, 0x8303d008
    lwi $r1, [$r0]
    and $r1, $r1, $r3
    bnez $r1, diag_tx_wd_disarm_unsafe
    lwi $r1, [$fp + TX_WD_STATE]
    movi $r3, 1
    bne $r1, $r3, diag_tx_wd_disarm_unsafe
    movi $r1, 0
    swi $r1, [$fp + TX_WD_STATE]
    dsb
    lwi $r0, [$fp + DIAG_REQ_SEQ]
    j diag_reply_status
diag_tx_wd_disarm_unsafe:
    tx_shutdown
    movi $r1, 4    /* unsafe/expired state, fail closed */
    lwi $r0, [$fp + DIAG_REQ_SEQ]
    j diag_reply_status
diag_tx_wd_status:
    lwi $r1, [$fp + TX_WD_STATE]
    j diag_reply_value
/* Read-only native eFuse via validated immutable Fenvi ROM.
 * No writes to OTP, no reference to ROM write routine at 0x80d12e.
 * Allowed: ROM banks0..2 and aligned 0..1008 within each bank.
 * The main command ABI advertises this only after runtime ROM guard checks.
 */
diag_rom_efuse_probe:
    /* ARG = (ROM eFuse bank index << 16) | within-bank byte offset.
     * Three ROM-confirmed banks, each 0x400 physical bytes (10-bit AIN).
     * No write/program command can be invoked from this code path.
     */
    lwi $r1, [$fp + DIAG_ARG]
    srli $r3, $r1, 16
    movi $r0, 3
    slt $r0, $r3, $r0
    beqz $r0, diag_rom_invalid
    slli $r0, $r1, 16
    srli $r0, $r0, 16
    andi $r3, $r0, 15
    bnez $r3, diag_rom_invalid
    movi $r3, 1024
    slt $r3, $r0, $r3
    beqz $r3, diag_rom_invalid
    load32 $r1, 0x02003000
    bne $gp, $r1, diag_rom_unsupported
    load32 $r3, 0x020077e0
    lwi $r1, [$r3]
    load32 $r3, 0x0080df2a
    bne $r1, $r3, diag_rom_unsupported
    load32 $r3, 0x020077f0
    lwi $r1, [$r3]
    load32 $r3, 0x0080dd72
    bne $r1, $r3, diag_rom_unsupported
    load32 $r3, 0x0080cec0
    lwi $r1, [$r3]
    load32 $r3, 0xa48042fc
    bne $r1, $r3, diag_rom_unsupported
    swi $r2, [$fp + OTP_SAVE_R2]
    swi $r4, [$fp + OTP_SAVE_R4]
    swi $r5, [$fp + OTP_SAVE_R5]
    swi $lp, [$fp + OTP_SAVE_LP]
    movi $r0, 0
    swi $r0, [$fp + OTP_VALID]
    swi $r0, [$fp + OTP_DATA]
    swi $r0, [$fp + OTP_DATA + 4]
    swi $r0, [$fp + OTP_DATA + 8]
    swi $r0, [$fp + OTP_DATA + 12]
    lwi $r0, [$fp + DIAG_ARG]
    srli $r4, $r0, 16      /* ROM bank index 0..2 */
    slli $r0, $r0, 16
    srli $r0, $r0, 16      /* within-bank byte offset 0..0x3f0 */
    addi $r1, $fp, OTP_DATA
    addi $r2, $fp, OTP_VALID
    movi $r3, 1            /* read mode: NEVER OTP programming */
    load32 $r5, 0x0080cec0 /* actual Fenvi ROM sys_efuse READ */
    jral5 $r5
    swi $r0, [$fp + OTP_ROM_RC]
    lwi $r2, [$fp + OTP_SAVE_R2]
    lwi $r4, [$fp + OTP_SAVE_R4]
    lwi $r5, [$fp + OTP_SAVE_R5]
    lwi $lp, [$fp + OTP_SAVE_LP]
    lwi $r1, [$fp + OTP_ROM_RC]
    bnez $r1, diag_rom_fail
    lwi $r1, [$fp + OTP_VALID]
    swi $r1, [$fp + DIAG_RESULT]
    movi $r1, 0
    lwi $r0, [$fp + DIAG_REQ_SEQ]
    j diag_reply_status
diag_rom_invalid:
    movi $r1, 3
    lwi $r0, [$fp + DIAG_REQ_SEQ]
    j diag_reply_status
diag_rom_unsupported:
    movi $r1, 2
    lwi $r0, [$fp + DIAG_REQ_SEQ]
    j diag_reply_status
diag_rom_fail:
    movi $r1, 4
    lwi $r0, [$fp + DIAG_REQ_SEQ]
    j diag_reply_status
diag_version:
    load32 $r1, 0x4d543031    /* 'MT01' ABI signature */
    swi $r1, [$fp + DIAG_RESULT]
    movi $r1, 0
    j diag_reply_status
diag_caps:
    movi $r1, 2045         /* bit 1024: watchdog covers logical TX paths0..3 */
    load32 $r3, 0x02003000
    bne $gp, $r3, diag_caps_publish
    load32 $r3, 0x020077e0
    lwi $r3, [$r3]
    load32 $r0, 0x0080df2a
    bne $r3, $r0, diag_caps_publish
    load32 $r3, 0x020077f0
    lwi $r3, [$r3]
    load32 $r0, 0x0080dd72
    bne $r3, $r0, diag_caps_publish
    load32 $r3, 0x0080cec0
    lwi $r3, [$r3]
    load32 $r0, 0xa48042fc /* verified first ROM eFuse-read instruction pair */
    bne $r3, $r0, diag_caps_publish
    movi $r1, 2047         /* bit 1024: watchdog covers logical TX paths0..3 */
diag_caps_publish:
    swi $r1, [$fp + DIAG_RESULT]
    movi $r1, 0
    lwi $r0, [$fp + DIAG_REQ_SEQ]
    j diag_reply_status
diag_stat:
    lwi $r1, [$fp + DIAG_ARG]
    beqz $r1, diag_stat_seq
    movi $r3, 1
    beq $r1, $r3, diag_stat_gaps
    movi $r3, 2
    beq $r1, $r3, diag_stat_time
    movi $r1, 3            /* invalid stat index */
    j diag_reply_status
diag_stat_seq:
    lwi $r1, [$fp + 40]
    j diag_reply_value
diag_stat_gaps:
    lwi $r1, [$fp + 104]
    j diag_reply_value
diag_stat_time:
    load32 $r1, 0x89020068 /* already used by existing I/Q code */
    lwi $r1, [$r1]
diag_reply_value:
    swi $r1, [$fp + DIAG_RESULT]
    movi $r1, 0
diag_reply_status:
    swi $r1, [$fp + DIAG_STATUS]
    dsb
    swi $r0, [$fp + DIAG_DONE_SEQ]
    dsb
diag_exit:
    ret5
    .size _start, .-_start
