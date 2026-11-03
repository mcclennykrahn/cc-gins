import json, sys, numpy as np, torch
from multiprocessing import Pool
from pathlib import Path
from ccgins import learning
from ccgins.metrics import summarize
from ccgins.pipeline import Config, run
from ccgins.scenarios import OUTAGES, VAL_SEEDS, make_scenario
from ccgins.sim import ImuSpec
NEFF=tuple(json.loads(Path("results/motionnet_val.json").read_text())["neff_corr_scale"])
SH=ImuSpec(vib_accel_per_mps=0.03, vib_gyro_per_mps=0.003, accel_bias_sigma=8.0)
M={"nhc":dict(pseudo="nhc"),
   "lv+aci fast":dict(pseudo="learned_vel",corr_scale=NEFF,aci=True,aci_gamma=0.05,aci_window=100),
   "hyb+aci fast":dict(pseudo="hybrid",corr_scale=NEFF,aci=True,aci_gamma=0.05,aci_window=100),
   "hyb+aci vfast":dict(pseudo="hybrid",corr_scale=NEFF,aci=True,aci_gamma=0.1,aci_window=50),
   "hyb+aci k3":dict(pseudo="hybrid",corr_scale=NEFF,aci=True,hybrid_max_kappa=3),
   "hyb+aci fast k3":dict(pseudo="hybrid",corr_scale=NEFF,aci=True,aci_gamma=0.05,aci_window=100,hybrid_max_kappa=3),
   "hyb+aci fast k10":dict(pseudo="hybrid",corr_scale=NEFF,aci=True,aci_gamma=0.05,aci_window=100,hybrid_max_kappa=10),
}
def f(a):
    seed,shift=a
    torch.set_num_threads(1)
    net=learning.load("results/motionnet.pt"); sc=make_scenario(seed,outages=OUTAGES,imu_spec=SH if shift else None)
    idx=np.arange(100,len(sc.truth.t),10)
    with torch.no_grad(): mu,var=learning.predict(net,sc.imu6(),idx)
    out={}
    for k,kw in M.items():
        log=run(sc.truth,sc.imu,sc.gnss,sc.imu_spec,sc.gnss_spec,Config(filter="inekf",init_seed=seed,**kw),pred=(idx,mu,var))
        m=summarize(log,OUTAGES); out[k]=(m["rmse_h"],m["max_err_outage_60s"],m["max_err_outage_120s"],m["coverage95_h_outage"],log.extra.get("fallback_frac",0))
    return out
with Pool(8) as p: r=p.map(f,[(s,sh) for sh in (True,False) for s in VAL_SEEDS])
for sh,rr in (("shift",r[:4]),("nominal",r[4:])):
    print(sh)
    for k in M: print("  %-14s"%k, np.mean([x[k] for x in rr],0).round(3))
