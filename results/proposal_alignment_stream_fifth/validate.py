"""Locked 10-test synthetic protocol; legacy checkpoint, no tuning."""
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import torch

from ccgins.learning import load
from ccgins.metrics import summarize
from ccgins.pipeline import Config, run
from ccgins.scenarios import TEST_SEEDS, OUTAGES, make_scenario
from ccgins.stream_replay import save_artifact, replay

OUT = Path(__file__).parent

def main():
    torch.set_num_threads(1)
    net = load('results/motionnet.pt')
    import yaml
    cfg = Config(**yaml.safe_load(Path('configs/proposal_alignment_full.yaml').read_text())['filter'])
    rows=[]
    for seed in TEST_SEEDS:
        sc=make_scenario(seed, outages=OUTAGES)
        sc.gnss.valid[sc.gnss.idx == 0]=False
        path=OUT/f'seed_{seed}.npz'
        save_artifact(path, sc, cfg)
        print(f"seed {seed}: artifact saved",flush=True)
        states=[]
        def collect(event):
            if event['event']=='imu' and round(event['epoch']/sc.truth.dt)%10==0 and event['epoch']>0:
                states.append(event['state'])
        nav,timing=replay(path,net,collect)
        print(f"seed {seed}: replay complete {timing}",flush=True)
        batch=run(sc.truth,sc.imu,sc.gnss,sc.imu_spec,sc.gnss_spec,cfg,model=net,log_covariance=True)
        log=SimpleNamespace(t=np.array([s.t for s in states]),p_est=np.array([s.position for s in states]),
            p_true=batch.p_true,P_pos=np.array([s.position_covariance for s in states]),runtime_per_step_us=0)
        diffs={}
        for name,actual,expected in [('position',log.p_est,batch.p_est),
                ('velocity',[s.velocity for s in states],batch.v_est),
                ('covariance',[s.covariance for s in states],batch.extra['covariance']),
                ('position_covariance',log.P_pos,batch.P_pos)]:
            diffs[name+'_max_abs']=float(np.max(np.abs(np.asarray(actual)-expected)))
        row=dict(seed=seed,stream=summarize(log,OUTAGES,t_start=30.),batch=summarize(batch,OUTAGES,t_start=30.),
                 differences=diffs,timing=timing,artifact_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        rows.append(row)
        (OUT/'metrics.json').write_text(json.dumps(dict(checkpoint_sha256=hashlib.sha256(Path('results/motionnet.pt').read_bytes()).hexdigest(),
            config=vars(cfg),source='synthetic IMU and synthetic GNSS',runs=rows),indent=2)+'\n')
        print(json.dumps(row),flush=True)

if __name__=='__main__': main()
