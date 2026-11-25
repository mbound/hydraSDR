/* SPDX-License-Identifier: MIT
 * No USB hardware needed: verifies allowlist, lock and idle-only boundaries.
 */
#include "internal.h"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdio.h>
#include <string.h>

static int seen=0;
static uint32_t last_address=0;
static int fake_read(void *ctx,enum bs_space space,uint32_t addr,uint32_t *val) {
    (void)ctx;
    assert(space==BS_MMIO);
    ++seen;
    last_address=addr;
    *val=addr^0x5a5a5a5au;
    return 0;
}
int main(void) {
    hydrasdr_dev_t dev;
    memset(&dev,0,sizeof(dev));
    atomic_init(&dev.run_state,BS_IDLE);
    assert(pthread_mutex_init(&dev.control_mutex,NULL)==0);
    dev.io.read32=fake_read;
    assert(hydrasdr_diag_count()==8);
    for(size_t i=0;i<hydrasdr_diag_count();i++) {
        const struct hydrasdr_diag_register *reg=hydrasdr_diag_info(i);
        uint32_t val=0;
        assert(reg && reg->name && reg->description);
        assert(hydrasdr_diag_read(&dev,i,&val)==0);
        assert(last_address==reg->addr && val==(reg->addr^0x5a5a5a5au));
    }
    assert(seen==8);
    assert(hydrasdr_diag_info(8)==NULL);
    uint32_t v=0;
    assert(hydrasdr_diag_read(&dev,8,&v)==HYDRASDR_ERROR_INVALID);
    assert(hydrasdr_diag_read(&dev,0,NULL)==HYDRASDR_ERROR_INVALID);
    atomic_store(&dev.run_state,BS_RUNNING);
    assert(hydrasdr_diag_read(&dev,0,&v)==HYDRASDR_ERROR_BUSY);
    assert(seen==8);
    atomic_store(&dev.run_state,BS_IDLE);
    assert(pthread_mutex_destroy(&dev.control_mutex)==0);
    puts("mt79 diagnostic single-owner tests passed");
    return 0;
}
