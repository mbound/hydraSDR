/* SPDX-License-Identifier: MIT
 * Device-owned, read-only diagnostics. No second USB handle or firmware mode switch.
 */
#include "internal.h"
#include <string.h>

static const struct hydrasdr_diag_register mt7961_diag[] = {
    {"dfe_source",  0x830ad440u, "DFE mode and active source"},
    {"dfe_mux",     0x830ad448u, "DFE tap / capture mux"},
    {"phy0_gain0",  0x830000c4u, "PHY0 gain register 0"},
    {"phy0_gain1",  0x83000108u, "PHY0 gain register 1"},
    {"phy0_gain2",  0x83000150u, "PHY0 gain register 2"},
    {"phy1_gain0",  0x830100c4u, "PHY1 gain register 0"},
    {"phy1_gain1",  0x83010108u, "PHY1 gain register 1"},
    {"phy1_gain2",  0x83010150u, "PHY1 gain register 2"},
};

/* Stable symbolic IDs, never arbitrary caller-controlled address access. */
size_t hydrasdr_diag_count(void) {
    return sizeof(mt7961_diag) / sizeof(mt7961_diag[0]);
}
const struct hydrasdr_diag_register *hydrasdr_diag_info(size_t id) {
    return id < hydrasdr_diag_count() ? &mt7961_diag[id] : NULL;
}
int hydrasdr_diag_read(hydrasdr_dev_t *dev, size_t id, uint32_t *value) {
    if (!dev || !value || id >= hydrasdr_diag_count())
        return HYDRASDR_ERROR_INVALID;
    /* Strict idle-only until the asynchronous streaming API offers a formally
     * scheduled control transaction / sample-boundary discontinuity event.
     */
    if (atomic_load(&dev->run_state) != BS_IDLE)
        return HYDRASDR_ERROR_BUSY;
    pthread_mutex_lock(&dev->control_mutex);
    int rc;
    if (atomic_load(&dev->run_state) != BS_IDLE)
        rc = HYDRASDR_ERROR_BUSY;
    else
        rc = bs_read32(dev, BS_MMIO, mt7961_diag[id].addr, value);
    pthread_mutex_unlock(&dev->control_mutex);
    return rc;
}
