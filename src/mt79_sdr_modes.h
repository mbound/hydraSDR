/* SPDX-License-Identifier: MIT
 * Native MT79xx SDR receiver-format contract, independent of BirriaSDR.
 */
#ifndef MT79_SDR_MODES_H
#define MT79_SDR_MODES_H
#include <stdint.h>

enum mt79_sdr_rx_mode {
    MT79_SDR_RX0 = 1,
    MT79_SDR_RX1 = 2,
    MT79_SDR_DUAL = 3
};
enum mt79_sdr_capture_layout {
    MT79_CAPTURE_DENSE_QI14 = 1,
    MT79_CAPTURE_RAW2_INTERLEAVED_QI14 = 2
};
struct mt79_sdr_mode_plan {
    enum mt79_sdr_rx_mode mode;
    enum mt79_sdr_capture_layout layout;
    uint32_t rate_per_channel;
    uint32_t channel_count;
    uint32_t dfe_source_word; /* low 8 bits of 0x830AD440; bit31 active */
    uint32_t dfe_mux_word;    /* lower 28 bits of 0x830AD448 */
    uint32_t expected_complex_per_channel_per_usb_packet;
    uint64_t expected_usb_payload_bytes_per_sec;
    uint8_t rf0_companion_activation;
    uint8_t phase_calibration_required;
};
/* Returns 0 or -1 for unsupported mode/rate pair; never silently aliases
 * SINGLE mode onto a RAW2 source.
 */
int mt79_sdr_plan(enum mt79_sdr_rx_mode mode, uint32_t rate,
                  struct mt79_sdr_mode_plan *out);
#endif
