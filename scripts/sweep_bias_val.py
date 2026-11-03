"""VAL-seed sweep for the v3 bias monitor (hybrid_bias_z). Selection uses VAL seeds only."""
import json
from multiprocessing import Pool
from pathlib import Path

import numpy as np
import torch

from ccgins import learning
from ccgins.metrics import summarize
from ccgins.pipeline import Config, run
from ccgins.scenarios import OUTAGES, VAL_SEEDS, make_scenario
from ccgins.sim import ImuSpec

NEFF = tuple(json.loads(Path("results/motionnet_val.json").read_text())["neff_corr_scale"])
SH = ImuSpec(vib_accel_per_mps=0.03, vib_gyro_per_mps=0.003, accel_bias_sigma=8.0)
V2 = dict(pseudo="hybrid", corr_scale=NEFF, aci=True, aci_gamma=0.05, aci_window=100, hybrid_max_kappa=10.0)
M = {"nhc": dict(pseudo="nhc"), "v2": V2}
for zb in (2.0, 3.0, 4.0, 6.0):
    M[f"v2+bias z{zb:g}"] = dict(V2, hybrid_bias_z=zb)
    M[f"v2+bias z{zb:g} latch"] = dict(V2, hybrid_bias_z=zb, hybrid_bias_latch=True)


def f(a):
    seed, shift = a
    torch.set_num_threads(1)
    net = learning.load("results/motionnet.pt")
    sc = make_scenario(seed, outages=OUTAGES, imu_spec=SH if shift else None)
    idx = np.arange(100, len(sc.truth.t), 10)
    with torch.no_grad():
        mu, var = learning.predict(net, sc.imu6(), idx)
    out = {}
    for k, kw in M.items():
        log = run(sc.truth, sc.imu, sc.gnss, sc.imu_spec, sc.gnss_spec,
                  Config(filter="inekf", init_seed=seed, **kw), pred=(idx, mu, var))
        m = summarize(log, OUTAGES)
        out[k] = (m["rmse_h"], m["max_err_outage_60s"], m["max_err_outage_120s"],
                  m["coverage95_h_outage"], log.extra.get("fallback_frac", 0))
    return out


if __name__ == "__main__":
    with Pool(8) as p:
        r = p.map(f, [(s, sh) for sh in (True, False) for s in VAL_SEEDS])
    n = len(VAL_SEEDS)
    print("cols: rmse_h, max60, max120, cov_outage, fallback_frac (mean over VAL seeds)")
    for sh, rr in (("shift", r[:n]), ("nominal", r[n:])):
        print(sh)
        for k in M:
            print("  %-16s" % k, np.mean([x[k] for x in rr], 0).round(3))
