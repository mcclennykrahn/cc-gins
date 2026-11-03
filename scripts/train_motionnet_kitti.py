"""Fine-tune the sim-trained MotionNet on KITTI train drives; validate both on the KITTI val drive.

Outputs results/kitti/motionnet_kitti.pt and results/kitti/motionnet_val.json with per-model
RMSE, raw-variance 95% coverage and the correlation inflation factor n_eff (used by experiments).
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import torch
from scipy.stats import chi2

from ccgins import learning
from ccgins.kitti_scenarios import TRAIN_DRIVES, VAL_DRIVES, drive_segments, make_kitti_scenario

ap = argparse.ArgumentParser()
ap.add_argument("--epochs", type=int, default=10)
ap.add_argument("--aug-seeds", type=int, default=4, help="IMU-error realizations per segment")
a = ap.parse_args()
out = Path("results/kitti")
out.mkdir(parents=True, exist_ok=True)

imus, vbs = [], []
dur = 0.0
for d in TRAIN_DRIVES:
    for tr in drive_segments(d, 30.0):
        dur += tr.t[-1]
        for s in range(a.aug_seeds):
            sc = make_kitti_scenario(tr, seed=1000 + s)
            imus.append(sc.imu6())
            vbs.append(tr.v_body)
print(f"train: {len(imus)} sequences ({dur:.0f} s of driving x {a.aug_seeds} IMU-error seeds)")
sim_net = learning.load("results/motionnet.pt")
t0 = time.time()
ft = learning.train_motionnet(imus, vbs, epochs=a.epochs, lr=1e-3, init=sim_net)
print(f"fine-tuned in {time.time() - t0:.0f}s")
learning.save(ft, out / "motionnet_kitti.pt")


def validate(net):
    errs, zs = [], []
    for d in VAL_DRIVES:
        for tr in drive_segments(d, 30.0):
            for s in range(3):
                sc = make_kitti_scenario(tr, seed=2000 + s)
                idx = np.arange(learning.WINDOW, len(tr.t), 10)
                with torch.no_grad():
                    mu, var = learning.predict(net, sc.imu6(), idx)
                e = mu - tr.v_body[idx - 1]
                errs.append(e)
                zs.append(e / np.sqrt(var))
    neff = np.mean([learning.correlation_factor(z) for z in zs], axis=0)
    E, Z = np.concatenate(errs), np.concatenate(zs)
    return {"val_rmse_fwd_lat_up": np.sqrt((E**2).mean(0)).round(4).tolist(),
            "val_coverage95_raw_variance": (Z**2 <= chi2.ppf(0.95, 1)).mean(0).round(4).tolist(),
            "val_mean_normalized_sq_err": (Z**2).mean(0).round(3).tolist(),
            "neff_corr_scale": neff.round(2).tolist()}


rep = {"sim_only": validate(sim_net), "kitti_finetuned": validate(ft),
       "train_drives": TRAIN_DRIVES, "val_drives": VAL_DRIVES, "train_seconds": round(dur, 1)}
print(json.dumps(rep, indent=1))
(out / "motionnet_val.json").write_text(json.dumps(rep, indent=1))
