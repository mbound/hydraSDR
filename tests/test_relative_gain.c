/* SPDX-License-Identifier: MIT
 * Source-qualified relative gain model, no hardware or private OTP needed.
 */
#include "hydrasdr.h"
#include "hydra_fenvi_2437_relative_gain.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(expr) do { if(!(expr)){fprintf(stderr,"%s:%d FAILED %s\n",__FILE__,__LINE__,#expr);exit(1);} }while(0)

int main(void)
{
    const struct hydrasdr_relative_gain_profile *p=&hydra_fenvi_2437_provisional;
    struct hydrasdr_gain zero={0,0,0}, top={5,8,12}, invalid={1,4,1};
    int value=-1;
    CHECK(hydrasdr_relative_gain_estimate(p,&zero,&value)==0 && value==0);
    CHECK(hydrasdr_relative_gain_estimate(p,&top,&value)==0 && value==593);
    CHECK(hydrasdr_relative_gain_estimate(p,&invalid,&value)==HYDRASDR_ERROR_UNSUPPORTED);
    CHECK(hydrasdr_relative_gain_estimate(p,&zero,NULL)==HYDRASDR_ERROR_INVALID);
    CHECK(hydrasdr_relative_gain_estimate(NULL,&zero,&value)==HYDRASDR_ERROR_INVALID);
    CHECK(hydrasdr_relative_gain_context_match(p,2437000000ULL,10000000u,HYDRASDR_RX_SINGLE0,0)==0);
    CHECK(hydrasdr_relative_gain_context_match(p,2437000000ULL,40000000u,HYDRASDR_RX_SINGLE0,0)==HYDRASDR_ERROR_UNSUPPORTED);
    CHECK(hydrasdr_relative_gain_context_match(p,5200000000ULL,10000000u,HYDRASDR_RX_SINGLE0,0)==HYDRASDR_ERROR_UNSUPPORTED);
    CHECK(hydrasdr_relative_gain_context_match(p,2437000000ULL,10000000u,HYDRASDR_RX_DUAL,0)==HYDRASDR_ERROR_UNSUPPORTED);
    CHECK(hydrasdr_relative_gain_context_match(p,2437000000ULL,10000000u,HYDRASDR_RX_SINGLE0,1)==HYDRASDR_ERROR_UNSUPPORTED);
    struct hydrasdr_gain selected={255,255,255};
    CHECK(hydrasdr_relative_gain_select(p,0,NULL,&selected,&value)==0);
    CHECK(selected.lna==0 && selected.lpf==0 && selected.digital==0 && value==0);
    CHECK(hydrasdr_relative_gain_select(p,593,NULL,&selected,&value)==0);
    CHECK(selected.lna==5 && selected.lpf==8 && selected.digital==12 && value==593);
    CHECK(hydrasdr_relative_gain_select(p,-1,NULL,&selected,&value)==HYDRASDR_ERROR_UNSUPPORTED);
    CHECK(hydrasdr_relative_gain_select(p,594,NULL,&selected,&value)==HYDRASDR_ERROR_UNSUPPORTED);
    struct hydrasdr_gain cap={3,4,4};
    CHECK(hydrasdr_relative_gain_select(p,280,&cap,&selected,&value)==0);
    CHECK(selected.lna<=cap.lna && selected.lpf<=cap.lpf && selected.digital<=cap.digital);
    CHECK(abs(value-280)<=10);
    CHECK(hydrasdr_relative_gain_select(p,400,&cap,&selected,&value)==HYDRASDR_ERROR_UNSUPPORTED);
    cap=(struct hydrasdr_gain){6,4,0};
    CHECK(hydrasdr_relative_gain_select(p,100,&cap,&selected,&value)==HYDRASDR_ERROR_INVALID);
    struct hydrasdr_relative_gain_profile bad=*p;
    bad.source_basis=0;
    CHECK(hydrasdr_relative_gain_estimate(&bad,&zero,&value)==HYDRASDR_ERROR_INVALID);
    bad=*p;bad.lna_mask|=1u<<6;
    CHECK(hydrasdr_relative_gain_estimate(&bad,&zero,&value)==HYDRASDR_ERROR_INVALID);
    bad=*p;bad.lpf_tenths[8]=-1;
    CHECK(hydrasdr_relative_gain_estimate(&bad,&zero,&value)==HYDRASDR_ERROR_INVALID);
    bad=*p;bad.digital_tenths[2]=2000;
    CHECK(hydrasdr_relative_gain_estimate(&bad,&zero,&value)==HYDRASDR_ERROR_INVALID);
    bad=*p;bad.provenance="";
    CHECK(hydrasdr_relative_gain_estimate(&bad,&zero,&value)==HYDRASDR_ERROR_INVALID);
    printf("provisional relative gain table/selection/context guards passed; no calibrated dBm claim\n");
    return 0;
}
