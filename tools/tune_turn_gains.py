#!/usr/bin/env python3
"""Bounded, supervised PI pattern search using the existing 90-degree LIO trial.

Never releases STOP. Aborts on any failed trial. Parameters remain RAM-only.
Score (lower is better): motion seconds + start-delay seconds +
0.5 * summed settled angle errors (degrees) + excess angular travel (degrees).
The latter uses 0.2s samples and a 0.2 degree noise deadband.
A candidate must improve mean score >=10% in discovery + repeated validation.
This is a local search on this surface, not global PID identification.
"""
import argparse
import datetime
import json
import math
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / 'src/mrbobus_bringup/config/tuning.yaml'
MARKER = '\n# Algorithmic turn calibration candidate (RAM only).\n'


def score(samples):
    valid = [s for s in samples if 'lio_base' in s]
    origin = valid[0]['lio_base']['yaw']
    def angle(s):
        d = s['lio_base']['yaw'] - origin
        return math.degrees(math.atan2(math.sin(d), math.cos(d)))
    phases = []
    for name, target, direction in [('p_probe_1', 90., 1), ('p_probe_-1', 0., -1)]:
        indices = [i for i,s in enumerate(valid) if s['phase'] == name]
        if not indices: raise ValueError('Missing motion phase')
        first,last = indices[0],indices[-1]
        move = valid[first:last+1]
        end = last + 1
        while end < len(valid) and valid[end]['phase'] == 'settle': end += 1
        settled = valid[max(last,end-1)]
        duration = move[-1]['monotonic']-move[0]['monotonic']
        start = next((s['monotonic']-move[0]['monotonic'] for s in move
                      if direction*(angle(s)-angle(move[0])) >= 2), duration)
        error = abs(angle(settled)-target)
        if duration >= 29 or error > 5: raise ValueError(f'Target not settled: {error:.2f} deg')
        previous = move[0]; wrong = 0.
        for s in valid[first:end]:
            if s['monotonic']-previous['monotonic'] >= .2:
                wrong += max(0., -direction*(angle(s)-angle(previous))-.2)
                previous = s
        phases.append(dict(duration=duration,start_delay=start,error_deg=error,
                           opposite_travel_deg=wrong,settled_deg=angle(settled)))
    cost = sum(p['duration']+p['start_delay']+.5*p['error_deg']+p['opposite_travel_deg'] for p in phases)
    return dict(score=cost,phases=phases)


def profile(p,i):
    assert .8 <= p <= 1.2 and .25 <= i <= .75
    return f'auto_turn:\n  rear_p: {p}\n  rear_i: {i}\n  front_p: {p*.53:.6f}\n  front_i: {i*.53:.6f}\n'


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--host',default='taras@192.168.67.149')
    ap.add_argument('--score',type=Path)
    a=ap.parse_args()
    if a.score:
        print(json.dumps(score(json.loads(a.score.read_text())),indent=2)); return
    original=CONFIG.read_text(); base=original.split(MARKER)[0].rstrip()+'\n'
    out=ROOT/'.local/calibration'/datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    out.mkdir(parents=True); (out/'tuning.before.yaml').write_text(original)
    results=[]
    def trial(p,i,label):
        CONFIG.write_text(base+MARKER+profile(p,i))
        command=[sys.executable,str(ROOT/'tools/run_drive_trial.py'),'--host',a.host,
                 '--profile','auto_turn','--p-probe','--probe-angle','90','--current-limit','20','--require-lio']
        print(f'Candidate {label}: P={p}, I={i}',flush=True)
        before=set((ROOT/'.local/trials').iterdir())
        subprocess.run(command,check=True)
        created=set((ROOT/'.local/trials').iterdir())-before
        if len(created)!=1: raise RuntimeError('Ambiguous trial artifact')
        path=created.pop()
        result=dict(p=p,i=i,label=label,trial=str(path),**score(json.loads((path/'samples.json').read_text())))
        results.append(result); (out/'results.json').write_text(json.dumps(results,indent=2))
        print('Scored: '+json.dumps(result),flush=True)
        return result
    try:
        baseline=trial(1.,.5,'baseline')
        for p,i in [(1.,.25),(1.,.75),(.8,.5),(1.2,.5)]: trial(p,i,'neighbor')
        best=min(results,key=lambda r:r['score'])
        repeat=trial(best['p'],best['i'],'winner-repeat')
        control=trial(1.,.5,'baseline-repeat')
        improvement=1-(best['score']+repeat['score'])/(baseline['score']+control['score'])
        chosen=best if improvement>=.1 else baseline
        CONFIG.write_text(base+MARKER+profile(chosen['p'],chosen['i']))
        (out/'selection.json').write_text(json.dumps(dict(chosen=chosen,improvement=improvement,
            accepted=improvement>=.1,criterion=__doc__),indent=2))
        print(f'Selected P={chosen["p"]}, I={chosen["i"]}; improvement={improvement:.1%}; {out}',flush=True)
    except BaseException:
        CONFIG.write_text(original)
        raise


if __name__=='__main__': main()
