import json, numpy as np, torch
from multiprocessing import Pool
from pathlib import Path
from ccgins import learning
from ccgins.metrics import summarize
from ccgins.pipeline import Config, run
from ccgins.scenarios import OUTAGES, VAL_SEEDS, make_scenario
NEFF=tuple(json.loads(Path("results/motionnet_val.json").read_text())["neff_corr_scale"])
def f(seed):
    torch.set_num_threads(1)
    net=learning.load("results/motionnet.pt"); sc=make_scenario(seed,outages=OUTAGES)
    idx=np.arange(100,len(sc.truth.t),10)
    with torch.no_grad(): mu,var=learning.predict(net,sc.imu6(),idx)
    out={}
    for b in (0.5,2,5,10):
        cfg=Config(filter="inekf",pseudo="learned_vel",corr_scale=NEFF,aci=True,aci_beta=b,init_seed=seed)
        m=summarize(run(sc.truth,sc.imu,sc.gnss,sc.imu_spec,sc.gnss_spec,cfg,pred=(idx,mu,var)),OUTAGES)
        out[b]=(m["rmse_h"],m["max_err_outage_120s"],m["coverage95_h_outage"],m["coverage95_h"])
    return out
with Pool(4) as p: r=p.map(f,VAL_SEEDS)
for b in r[0]: print(b,np.mean([x[b] for x in r],0).round(3))
