/* SPDX-License-Identifier: MIT */
#include "hydra_measurement.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#define CHECK(x) do {if(!(x)){fprintf(stderr,"line %d: %s\n",__LINE__,#x);exit(1);}}while(0)
int main(void) {
    struct hydra_iq_accumulator a,b;struct hydra_iq_result r;
    CHECK(hydra_iq_init(&a,0,0)==HYDRASDR_ERROR_INVALID);
    CHECK(!hydra_iq_init(&a,2,0));
    /* lane0 +/-4096 complex tone = -6.0206 dBFS; lane1 DC 2048. */
    int16_t iq[]={4096,0,2048,0,0,4096,2048,0,-4096,0,2048,0,0,-4096,2048,0};
    CHECK(!hydra_iq_update(&a,iq,3,0)); /* split on odd lane boundary */
    CHECK(!hydra_iq_update(&a,iq+6,5,3));
    CHECK(!hydra_iq_result(&a,0,&r));
    CHECK(r.count==4 && r.mean_i==0 && r.mean_q==0 && !r.clipped);
    CHECK(fabs(r.power_dbfs+6.020599913)<1e-8 && r.ac_dbfs==r.power_dbfs);
    CHECK(!hydra_iq_result(&a,1,&r));CHECK(r.count==4 && r.mean_i==2048 && r.ac_power==0 && isinf(r.ac_dbfs));
    b=a;CHECK(hydra_iq_update(&a,iq,1,9)==HYDRASDR_ERROR_PROTOCOL);CHECK(!memcmp(&a,&b,sizeof(a)));
    int16_t invalid[]={1,2,9000,0};CHECK(hydra_iq_update(&a,invalid,2,8)==HYDRASDR_ERROR_PROTOCOL);CHECK(!memcmp(&a,&b,sizeof(a)));
    CHECK(!hydra_iq_init(&a,1,0));
    int16_t rails[]={-8192,-8192,8191,8191,0,0};CHECK(!hydra_iq_update(&a,rails,3,0));
    CHECK(!hydra_iq_result(&a,0,&r));CHECK(r.clipped==2 && fabs(r.peak_dbfs-3.0102999566)<1e-8);
    CHECK(!hydra_iq_init(&a,2,1));CHECK(!hydra_iq_update(&a,iq,1,1));
    CHECK(hydra_iq_result(&a,0,&r)==HYDRASDR_ERROR_STATE);CHECK(!hydra_iq_result(&a,1,&r));
    CHECK(!hydra_iq_init(&a,1,UINT64_MAX));CHECK(hydra_iq_update(&a,iq,1,UINT64_MAX)==HYDRASDR_ERROR_INVALID);
    puts("IQ power, DC, lane fragmentation, rails, discontinuity and overflow passed");
}
