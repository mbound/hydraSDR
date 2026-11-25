/* SPDX-License-Identifier: MIT
 * No-hardware provisional gain code selection demonstration.
 */
#include "hydrasdr.h"
#include "hydra_fenvi_2437_relative_gain.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int main(int argc,char **argv)
{
    if(argc!=2){
        fprintf(stderr,"Usage: %s RELATIVE_GAIN_DB\n",argv[0]);
        fprintf(stderr,"Scope: Fenvi source-qualified 2437 MHz / 10 MSps SINGLE0; NOT calibrated absolute RF dB or dBm.\n");
        return 2;
    }
    errno=0;
    char *end=NULL;
    double request=strtod(argv[1],&end);
    if(errno || !end || end==argv[1] || *end || !isfinite(request) ||
       request<0 || request>100){
        fputs("Bad relative dB target\n",stderr);
        return 2;
    }
    struct hydrasdr_gain codes={0};
    int achieved=0;
    int rc=hydrasdr_relative_gain_select(&hydra_fenvi_2437_provisional,
             (int)lround(request*10),NULL,&codes,&achieved);
    if(rc){
       fprintf(stderr,"Requested gain is outside measured model (rc=%d)\n",rc);
       return 3;
    }
    printf("PROVISIONAL_RELATIVE_GAIN requested=%.1f dB estimated=%.1f dB raw_codes=LNA%u LPF%u DIG%u\n",
           request,achieved/10.0,codes.lna,codes.lpf,codes.digital);
    printf("No hardware changed. To apply, use hydrasdr_set_tuner_gain_mode(dev,1) and hydrasdr_set_gain_codes(dev,&codes).\n");
    printf("Conditions: exact Fenvi board, 2437MHz 10Msps SINGLE0. Not absolute input power, not RSSI dBm, not portable calibration.\n");
    return 0;
}
