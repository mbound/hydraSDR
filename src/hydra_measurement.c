/* SPDX-License-Identifier: MIT */
#include "hydra_measurement.h"
#include <math.h>
#include <string.h>
int hydra_iq_init(struct hydra_iq_accumulator *a,unsigned channels,uint64_t first)
{
    if(!a || (channels!=1 && channels!=2)) return HYDRASDR_ERROR_INVALID;
    memset(a,0,sizeof(*a)); a->channels=channels; a->next_word=first; return 0;
}
int hydra_iq_update(struct hydra_iq_accumulator *a,const int16_t *iq,
                    uint32_t n,uint64_t first)
{
    if(!a || !iq || !n || (a->channels!=1 && a->channels!=2))
        return HYDRASDR_ERROR_INVALID;
    if(first!=a->next_word) return HYDRASDR_ERROR_PROTOCOL;
    if(UINT64_MAX-first<n || a->words>UINT32_MAX-n) return HYDRASDR_ERROR_INVALID;
    struct hydra_iq_accumulator b=*a;
    for(uint32_t k=0;k<n;k++) {
        int32_t i=iq[2*(size_t)k],q=iq[2*(size_t)k+1];
        if(i < -8192 || i>8191 || q < -8192 || q>8191)
            return HYDRASDR_ERROR_PROTOCOL;
        unsigned l=(unsigned)((first+k)%a->channels);
        uint32_t power=(uint32_t)(i*i+q*q);
        b.lane[l].count++; b.lane[l].sum_i+=i; b.lane[l].sum_q+=q;
        b.lane[l].sum_power+=power;
        b.lane[l].clipped+=(i==-8192 || i==8191 || q==-8192 || q==8191);
        if(power>b.lane[l].peak_power) b.lane[l].peak_power=power;
    }
    b.next_word+=n; b.words+=n; *a=b; return 0;
}
static double dbfs(double p) {return p>0 ? 10.0*log10(p/67108864.0) : -INFINITY;}
int hydra_iq_result(const struct hydra_iq_accumulator *a,unsigned l,struct hydra_iq_result *out)
{
    if(!a || !out || (a->channels!=1 && a->channels!=2) || l>=a->channels)
        return HYDRASDR_ERROR_INVALID;
    if(!a->lane[l].count) return HYDRASDR_ERROR_STATE;
    struct hydra_iq_result r={0};
    r.count=a->lane[l].count; r.clipped=a->lane[l].clipped;
    r.mean_i=(double)a->lane[l].sum_i/r.count;
    r.mean_q=(double)a->lane[l].sum_q/r.count;
    r.mean_power=(double)a->lane[l].sum_power/r.count;
    r.ac_power=fmax(0,r.mean_power-r.mean_i*r.mean_i-r.mean_q*r.mean_q);
    r.peak_power=a->lane[l].peak_power;
    r.power_dbfs=dbfs(r.mean_power); r.ac_dbfs=dbfs(r.ac_power); r.peak_dbfs=dbfs(r.peak_power);
    *out=r; return 0;
}
