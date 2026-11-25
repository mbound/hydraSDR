#!/usr/bin/env python3
"""Empirical *relative received-power* gain steps; never absolute RF input dBm.
This method uses high-level IQ blocks, NOT FCS-matched Wi-Fi beacons, so the
estimates remain provisional until a stable external stimulus is isolated.
"""
from __future__ import annotations
import argparse, json, math, re, statistics
from pathlib import Path
PAT=re.compile(r"^GAIN_STAGE (\d+) LNA=(\d+) LPF=(\d+) DIG=(\d+) BLOCKS=(\d+) SAMPLES=(\d+) P50=([-\d.]+) P95=([-\d.]+) P99=([-\d.]+) P999=([-\d.]+) P9999=([-\d.]+) CLIPPED=(\d+) MAX_ABS=(\d+)$")
def decode(path):
    lines=path.read_text().splitlines()
    assert any(x.startswith('SUMMARY rc=0 stream=0 cancel=0 close=0 stats=0') for x in lines),path
    rows={}
    for line in lines:
        m=PAT.fullmatch(line)
        if not m: continue
        z=m.groups();i=int(z[0])
        assert i not in rows
        rows[i]={'i':i,'codes':tuple(map(int,z[1:4])),'blocks':int(z[4]),
                 'samples':int(z[5]),'p50':float(z[6]),'p95':float(z[7]),
                 'p99':float(z[8]),'p999':float(z[9]),'p9999':float(z[10]),
                 'clipped':int(z[11]),'max_abs':int(z[12])}
    return rows
def baseline(rows,refs,i,metric='p99'):
    before=[j for j in refs if j<=i]
    after=[j for j in refs if j>=i]
    a=before[-1] if before else after[0]
    b=after[0] if after else before[-1]
    if a==b:return rows[a][metric]
    w=(i-a)/(b-a)
    return (1-w)*rows[a][metric]+w*rows[b][metric]
def calculate(logs,profiles):
    refs=[p['index'] for p in profiles if p['stage']=='ref']
    assert len(refs)>=4 and refs[0]==0 and refs[-1]==len(profiles)-1
    allrows=[decode(p) for p in logs]
    values={}
    rec=[]
    for run,rows in enumerate(allrows):
        assert set(rows)==set(range(len(profiles))),(logs[run],len(rows),len(profiles))
        for p in profiles:
            i=p['index'];z=rows[i];assert z['codes']==(p['lna'],p['lpf'],p['digital'])
            if p['stage']=='ref': continue
            component=p['stage'];code=p[component]
            amp=z['p99']-baseline(rows,refs,i)
            sigma=z['p999']-baseline(rows,refs,i,'p999')
            assert z['blocks']>1500 and z['samples']>300000
            rec.append(dict(run=run,index=i,component=component,code=code,
                            reference_codes=[3,4,0],
                            difference_db=round(amp,3),
                            difference_db_p999=round(sigma,3),
                            p99_dbfs=z['p99'],
                            noise_p50_dbfs=z['p50'],
                            signal_over_median_db=round(z['p99']-z['p50'],2),
                            clipped=z['clipped']))
            values.setdefault((component,code),[]).append(amp)
    comps={}
    for key,vals in values.items():
        med=statistics.median(vals)
        diff=(max(vals)-min(vals)) if len(vals)>1 else None
        comps.setdefault(key[0],{})[key[1]]=dict(relative_to_reference_db=round(med,2),
           repeat_span_db=round(diff,2) if diff is not None else None,n=len(vals),
           values_db=[round(x,2) for x in vals])
    for component,codes in comps.items():
        if 0 not in codes:raise ValueError('missing 0 for '+component)
        zero=codes[0]['relative_to_reference_db']
        for code,details in codes.items():
            details['above_code0_db']=round(details['relative_to_reference_db']-zero,2)
    return comps,rec,allrows
def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--profiles',type=Path,required=True)
    ap.add_argument('--log',type=Path,action='append',required=True)
    ap.add_argument('--out',type=Path,required=True)
    a=ap.parse_args()
    profiles=json.loads(a.profiles.read_text())['profiles']
    c,rec,rows=calculate(a.log,profiles)
    bad=[r for r in rec if r['clipped'] or r['signal_over_median_db']<8.0]
    result={'schema':'fenvi-mt7972-relative-gain-provisional-v1','chipset':'MT7921AUN/MT7961+MT7972',
     'frequency_hz':2437000000,'rate_sps':10000000,'rx_mode':'SINGLE0','gain_force':'host',
     'reference':{'lna':3,'lpf':4,'digital':0},'signal_measurement':'unidentified ambient RF signal; IQ-block 99th percentile power vs bracketing reference (P99)',
     'calibrated_input_dbm':False,'source_power_dbm_known':False,
     'error_estimate':'repeat_span_db describes run-to-run differences only; excludes source fading and unidentified RF signals',
     'per_stage_db_above_code0':c,'observations':rec,
     'hardware_runs':len(rows),'source_files':[p.name for p in a.log],
     'all_streams_successful':True,'flagged_low_margin_or_clipped':bad,
     'warning':'Not traceable absolute RF gain or dBm. No FCS/preamble isolation; no verified per-frequency/chain transferability. Do not set CAP_GAIN_DB as calibrated.'}
    a.out.write_text(json.dumps(result,indent=2,sort_keys=True)+'\n')
    print('runs',len(rows),'observations',len(rec),'flagged',len(bad))
    for component in ['lna','lpf','digital']:
        print('===',component)
        for code,data in sorted(c[component].items()):
            print(code,'above0',data['above_code0_db'],'db','repeat_span',data['repeat_span_db'],'db')
if __name__=='__main__':main()
