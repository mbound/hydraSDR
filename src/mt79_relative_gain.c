/* SPDX-License-Identifier: MIT
 * Board-qualified EMPIRICAL receive-level gain profile lookup and code chooser.
 *
 * Deliberately separate from hydrasdr_{set,get}_tuner_gain (calibrated dB).
 * A relative change derived from ambient input is NOT absolute gain, RSSI,
 * RF output power, dBm, TSSI, EVM or a user OTP/EEPROM calibration.
 * No hardware writes, no RF SPI operations and no USB I/O here.
 */
#include "hydrasdr.h"
#include <limits.h>
#include <stddef.h>

#define LNA_COUNT 6u
#define LPF_COUNT 9u
#define DIG_COUNT 13u
#define MASK_N(n) ((1u<<(n))-1u)

static int valid(const struct hydrasdr_relative_gain_profile *p)
{
    if (!p || p->version != HYDRASDR_REL_GAIN_PROFILE_VERSION ||
        p->source_basis != HYDRASDR_REL_GAIN_AMBIENT_PROVISIONAL ||
        !p->provenance || !p->provenance[0] ||
        !p->frequency_hz || !p->sample_rate ||
        p->rx_mode != HYDRASDR_RX_SINGLE0 || p->rx_chain != 0 ||
        (p->lna_mask & ~MASK_N(LNA_COUNT)) ||
        (p->lpf_mask & ~MASK_N(LPF_COUNT)) ||
        (p->digital_mask & ~MASK_N(DIG_COUNT)) ||
        !(p->lna_mask & 1u) || !(p->lpf_mask & 1u) ||
        !(p->digital_mask & 1u) ||
        p->lna_tenths[0] || p->lpf_tenths[0] || p->digital_tenths[0])
        return HYDRASDR_ERROR_INVALID;

    /* Finite, physically plausible monotonically nondecreasing stage maps;
     * reject corrupted or mis-scaled profiles, not just malformed masks. */
    const int16_t *coeffs[3]={p->lna_tenths,p->lpf_tenths,p->digital_tenths};
    const unsigned n[3]={LNA_COUNT,LPF_COUNT,DIG_COUNT};
    const unsigned masks[3]={p->lna_mask,p->lpf_mask,p->digital_mask};
    for(unsigned stage=0;stage<3;stage++){
        int preceding=0;
        for(unsigned i=0;i<n[stage];i++){
            if(!(masks[stage]&(1u<<i)))continue;
            if(coeffs[stage][i]<preceding || coeffs[stage][i]>1200)
                return HYDRASDR_ERROR_INVALID;
            preceding=coeffs[stage][i];
        }
    }
    return 0;
}

static int code_valid(const struct hydrasdr_relative_gain_profile *p,
                      const struct hydrasdr_gain *g)
{
    if(!g || g->lna>=LNA_COUNT || g->lpf>=LPF_COUNT ||
       g->digital>=DIG_COUNT)
        return 0;
    return (p->lna_mask&(1u<<g->lna)) &&
           (p->lpf_mask&(1u<<g->lpf)) &&
           (p->digital_mask&(1u<<g->digital));
}

/* Source-qualified relative dB above LNA0/LPF0/DIG0. No unit-conversion
 * input for absolute dBm exists here intentionally.
 */
int hydrasdr_relative_gain_estimate(const struct hydrasdr_relative_gain_profile *p,
                                     const struct hydrasdr_gain *gain,
                                     int *tenths_db)
{
    if(!tenths_db || !gain)return HYDRASDR_ERROR_INVALID;
    int rc=valid(p);
    if(rc)return rc;
    if(!code_valid(p,gain))return HYDRASDR_ERROR_UNSUPPORTED;
    *tenths_db=(int)p->lna_tenths[gain->lna]+
               (int)p->lpf_tenths[gain->lpf]+
               (int)p->digital_tenths[gain->digital];
    return 0;
}

/* Quantize a requested *relative* gain to a set of empirically probed
 * raw gain codes. Caller applies codes via hydrasdr_set_gain_codes(), which
 * retains ownership/rollback guards. No implicit in-device calibration.
 *
 * max_codes is an optional operator-set safety ceiling; decreasing it
 * is recommended when strong receive signals clip on the actual board.
 * The chooser cannot predict ADC overload from code values alone.
 */
int hydrasdr_relative_gain_select(const struct hydrasdr_relative_gain_profile *p,
                                   int requested_tenths_db,
                                   const struct hydrasdr_gain *max_codes,
                                   struct hydrasdr_gain *chosen,
                                   int *achieved_tenths_db)
{
    if(!chosen || !achieved_tenths_db)return HYDRASDR_ERROR_INVALID;
    int rc=valid(p);
    if(rc)return rc;
    if(requested_tenths_db<0)return HYDRASDR_ERROR_UNSUPPORTED;
    struct hydrasdr_gain max={5,8,12};
    if(max_codes){
        if(max_codes->lna>max.lna || max_codes->lpf>max.lpf ||
           max_codes->digital>max.digital)
            return HYDRASDR_ERROR_INVALID;
        max=*max_codes;
    }
    int best_distance=INT_MAX, best_gain=-1, min_gain=INT_MAX, max_gain=-1;
    struct hydrasdr_gain winner={0,0,0};
    for(unsigned lna=0;lna<=max.lna;lna++)
      for(unsigned lpf=0;lpf<=max.lpf;lpf++)
        for(unsigned digital=0;digital<=max.digital;digital++){
          struct hydrasdr_gain candidate={(uint8_t)lna,(uint8_t)lpf,(uint8_t)digital};
          if(!code_valid(p,&candidate))continue;
          int estimate=p->lna_tenths[lna]+p->lpf_tenths[lpf]+p->digital_tenths[digital];
          if(estimate<min_gain)min_gain=estimate;
          if(estimate>max_gain)max_gain=estimate;
          int distance=estimate-requested_tenths_db;
          if(distance<0)distance=-distance;
          /* Prefer the larger LNA and smaller digital step on exact ties,
           * preserving analog-before-digital preference from experiments.
           * This is not an AGC or overload-avoidance algorithm. */
          if(distance<best_distance ||
             (distance==best_distance &&
              (digital<winner.digital ||
               (digital==winner.digital && lna>winner.lna)))){
              best_distance=distance;best_gain=estimate;winner=candidate;
          }
        }
    if(requested_tenths_db<min_gain || requested_tenths_db>max_gain ||
       best_distance==INT_MAX)return HYDRASDR_ERROR_UNSUPPORTED;
    *chosen=winner;
    *achieved_tenths_db=best_gain;
    return 0;
}

/* A gain profile is conditionally valid: reject different frequency, sample
 * rates, chain layout or unpublished special modes rather than assume a
 * 2437MHz/10Msps measurement applies at 5GHz or in 2RX.
 */
int hydrasdr_relative_gain_context_match(const struct hydrasdr_relative_gain_profile *p,
                                           uint64_t frequency_hz, uint32_t sample_rate,
                                           enum hydrasdr_rx_mode mode, unsigned rx_chain)
{
    int rc=valid(p);
    if(rc)return rc;
    return p->frequency_hz==frequency_hz && p->sample_rate==sample_rate &&
           p->rx_mode==mode && p->rx_chain==rx_chain ? 0
                                                       : HYDRASDR_ERROR_UNSUPPORTED;
}
