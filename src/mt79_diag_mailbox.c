/* SPDX-License-Identifier: MIT
 * MT79xx research-N9 diagnostic mailbox over the existing USB MMIO handle.
 * No stock firmware, no kernel module and no second libusb device owner.
 * This is not an EEPROM/OTP read until the N9 image advertises that capability.
 */
#include "internal.h"

#define DIAG_MB 0x02038360u
#define DIAG_REQ_SEQ (DIAG_MB+112u)
#define DIAG_CMD (DIAG_MB+116u)
#define DIAG_ARG (DIAG_MB+120u)
#define DIAG_DONE_SEQ (DIAG_MB+124u)
#define DIAG_STATUS (DIAG_MB+128u)
#define DIAG_RESULT (DIAG_MB+132u)
#define DIAG_OTP_DATA (DIAG_MB+144u)
#define DIAG_OTP_VALID (DIAG_MB+160u)

int mt_diag_transact_locked(hydrasdr_dev_t *dev,uint32_t command,uint32_t arg,
                    uint32_t *value)
{
    /* Exact write ordering is the protocol; request sequence is written last.
     * All these registers are in the existing, tested N9 mailbox RAM range.
     */
    uint32_t seq=++dev->diag_sequence;
    uint32_t done=0,status=UINT32_MAX,result=0;
    if(!seq)seq=++dev->diag_sequence;
    BS_TRY(bs_write32(dev,BS_MMIO,DIAG_ARG,arg));
    BS_TRY(bs_write32(dev,BS_MMIO,DIAG_CMD,command));
    BS_TRY(bs_write32(dev,BS_MMIO,DIAG_REQ_SEQ,seq));
    uint64_t deadline=bs_now_ns()+UINT64_C(50000000);
    do {
        BS_TRY(bs_read32(dev,BS_MMIO,DIAG_DONE_SEQ,&done));
        if(done==seq) {
            BS_TRY(bs_read32(dev,BS_MMIO,DIAG_STATUS,&status));
            BS_TRY(bs_read32(dev,BS_MMIO,DIAG_RESULT,&result));
            /* Reject stale/non-atomic publication during a concurrent reset. */
            BS_TRY(bs_read32(dev,BS_MMIO,DIAG_DONE_SEQ,&done));
            if(done!=seq)return HYDRASDR_ERROR_PROTOCOL;
            if(status==2)return HYDRASDR_ERROR_UNSUPPORTED;
            if(status==3)return HYDRASDR_ERROR_INVALID;
            if(status)return HYDRASDR_ERROR_PROTOCOL;
            *value=result;
            return 0;
        }
        bs_sleep_us(250);
    } while(bs_now_ns()<deadline);
    return HYDRASDR_ERROR_TIMEOUT; /* older research image has no dispatcher */
}
int hydrasdr_custom_diag_query(hydrasdr_dev_t *dev,uint32_t command,
                                uint32_t argument,uint32_t *value)
{
    if(!dev || !value || (command!=HYDRASDR_DIAG_FW_ABI &&
       command!=HYDRASDR_DIAG_FW_STAT &&
       command!=HYDRASDR_DIAG_FW_EFUSE &&
       command!=HYDRASDR_DIAG_FW_CAPS))
        return HYDRASDR_ERROR_INVALID;
    pthread_mutex_lock(&dev->control_mutex);
    int rc=0;
    if(atomic_load(&dev->run_state)!=BS_RUNNING || !dev->control_ready) {
        rc=HYDRASDR_ERROR_BUSY;
        goto done;
    }
    if(!dev->diag_abi_verified) {
        uint32_t abi=0;
        rc=mt_diag_transact_locked(dev,HYDRASDR_DIAG_FW_ABI,0,&abi);
        if(rc)goto done;
        if(abi!=HYDRASDR_DIAG_ABI_MAGIC) {
            rc=HYDRASDR_ERROR_PROTOCOL;goto done;
        }
        dev->diag_abi_verified=true;
    }
    if(command==HYDRASDR_DIAG_FW_ABI) {
        *value=HYDRASDR_DIAG_ABI_MAGIC;
        goto done;
    }
    if(command==HYDRASDR_DIAG_FW_EFUSE) {
        uint32_t capabilities=0;
        rc=mt_diag_transact_locked(dev,HYDRASDR_DIAG_FW_CAPS,0,&capabilities);
        if(rc)goto done;
        if(!(capabilities&HYDRASDR_DIAG_CAP_EFUSE)) {
            rc=HYDRASDR_ERROR_UNSUPPORTED;goto done;
        }
    }
    rc=mt_diag_transact_locked(dev,command,argument,value);
done:
    pthread_mutex_unlock(&dev->control_mutex);
    return rc;
}

int hydrasdr_custom_efuse_read(hydrasdr_dev_t *dev,unsigned bank,
                                 unsigned offset,struct hydrasdr_efuse_block *out)
{
    if(!dev || !out || bank>=3 || offset>=1024 || (offset&15))
        return HYDRASDR_ERROR_INVALID;
    struct hydrasdr_efuse_block result={0};
    result.bank=(uint16_t)bank;
    result.offset=(uint16_t)offset;
    pthread_mutex_lock(&dev->control_mutex);
    int rc=0;
    uint32_t v=0;
    if(atomic_load(&dev->run_state)!=BS_RUNNING || !dev->control_ready) {
        rc=HYDRASDR_ERROR_BUSY;goto done;
    }
    if(!dev->diag_abi_verified) {
        rc=mt_diag_transact_locked(dev,HYDRASDR_DIAG_FW_ABI,0,&v);
        if(rc)goto done;
        if(v!=HYDRASDR_DIAG_ABI_MAGIC){rc=HYDRASDR_ERROR_PROTOCOL;goto done;}
        dev->diag_abi_verified=true;
    }
    rc=mt_diag_transact_locked(dev,HYDRASDR_DIAG_FW_CAPS,0,&v);
    if(rc)goto done;
    if(!(v&HYDRASDR_DIAG_CAP_EFUSE)){
        rc=HYDRASDR_ERROR_UNSUPPORTED;goto done;
    }
    rc=mt_diag_transact_locked(dev,HYDRASDR_DIAG_FW_EFUSE,
                ((uint32_t)bank<<16)|(uint32_t)offset,&v);
    if(rc)goto done;
    result.raw_validity=v;
    for(unsigned i=0;i<4;i++) {
        uint32_t word=0;
        rc=bs_read32(dev,BS_MMIO,DIAG_OTP_DATA+4*i,&word);
        if(rc)goto done;
        result.data[i*4]=(uint8_t)word;
        result.data[i*4+1]=(uint8_t)(word>>8);
        result.data[i*4+2]=(uint8_t)(word>>16);
        result.data[i*4+3]=(uint8_t)(word>>24);
    }
    uint32_t done_seq=0;
    rc=bs_read32(dev,BS_MMIO,DIAG_DONE_SEQ,&done_seq);
    if(rc)goto done;
    if(done_seq!=dev->diag_sequence) {
        rc=HYDRASDR_ERROR_PROTOCOL;goto done;
    }
    *out=result;
done:
    pthread_mutex_unlock(&dev->control_mutex);
    return rc;
}
