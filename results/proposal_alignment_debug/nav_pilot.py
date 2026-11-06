"""Validation-only monitor/domain diagnostic; never selects primary settings."""
from concurrent.futures import ProcessPoolExecutor
import json
import numpy as np
import torch
import yaml
from ccgins import learning as L
from ccgins.scenarios import make_scenario
from ccgins.pipeline import Config,run
from ccgins.metrics import summarize
from ccgins.alignment_next import diagnostic


def pilot(job):
 arch,monitor=job; torch.set_num_threads(1)
 c=yaml.safe_load(open('configs/proposal_alignment_next.yaml'))
 sc=make_scenario(501,duration=600,outages=c['outages'])
 net=L.load(f'results/proposal_alignment_debug/{arch}_smooth.pt')
 original=L.predict; live=[]; calls=0
 def capture(*args,**kwargs):
  nonlocal calls
  mu,var=original(*args,**kwargs)
  # Dual invokes A then B; no monitor invokes only A.
  if monitor!='dual' or calls%2==0: live.append((mu[0],var[0]))
  calls+=1
  return mu,var
 L.predict=capture
 log=run(sc.truth,sc.imu,sc.gnss,sc.imu_spec,sc.gnss_spec,Config(**{**c['filter'],'monitor':monitor}),model=net)
 L.predict=original
 idx=np.arange(100,len(sc.truth.t),10)
 mu=np.array([x[0] for x in live]); v=np.array([x[1] for x in live]); e=mu-sc.truth.v_body[idx-1]
 nll=.5*(np.log(2*np.pi*v)+e**2/v)
 return dict(pilot='validation only; not selected',seed=501,architecture=arch,monitor=monitor,
  navigation=summarize(log,c['outages'],t_start=30),alarm_time=log.alarm_time,
  final_kappa=log.kappa_hist[-1].tolist(),live_network_nll=float(nll.mean()),
  live_nll_channels=nll.mean(0).tolist(),reference_network=diagnostic(net,sc))

if __name__=='__main__':
 with ProcessPoolExecutor(max_workers=2) as pool:
  rows=list(pool.map(pilot,[(a,m) for a in ['legacy_cnn','proposal_tcn'] for m in ['dual','none']]))
 open('results/proposal_alignment_debug/nav_pilot.json','w').write(json.dumps(rows,indent=2)+'\n')
 print(json.dumps(rows,indent=2))
