"""Fine-tune MotionNet on REAL i2Nav MEMS data (awesome-gins-datasets), 2-fold time split.

Fold A: train on the 2nd half of the drive, test on the 1st half; fold B: the reverse (10 s guard).
Training sensors: ICM20602 + ADIS16460 only (ADIS16465, i300 are unseen sensors at test time).
Within the training half, the last VAL_S seconds are held out as validation (-> n_eff, coverage).
Augmentation: constant random turn-on biases added to the raw IMU (gyro 0.3 deg/s, accel 20 mg 1-sigma),
since sensors differ mainly in turn-on bias. Init: the sim-trained MotionNet.

Outputs results/i2nav/motionnet_fold{A,B}.pt, results/i2nav/motionnet_val.json.
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import torch
from scipy.stats import chi2

from ccgins import learning
from ccgins import i2nav_scenarios as I

ap = argparse.ArgumentParser()
ap.add_argument("--epochs", type=int, default=12)
ap.add_argument("--aug", type=int, default=4)
ap.add_argument("--stride", type=int, default=5)
a = ap.parse_args()
torch.set_num_threads(8)
OUT = Path("results/i2nav")
OUT.mkdir(parents=True, exist_ok=True)
VAL_S = 150.0
DEG = np.pi / 180


def windows_eval(net, seq):
    imu6 = np.concatenate([seq.truth.gyro, seq.truth.accel], 1)
    idx = np.arange(learning.WINDOW, len(seq.truth.t), 10)
    mu, var = learning.predict(net, imu6, idx)
    e = mu - seq.truth.v_body[idx - 1]
    return e, e / np.sqrt(var)


def stats(net, seqs):
    E, Z, ne = [], [], []
    for s in seqs:
        e, z = windows_eval(net, s)
        E.append(e); Z.append(z); ne.append(learning.correlation_factor(z))
    E, Z = np.concatenate(E), np.concatenate(Z)
    return {"rmse_fwd_lat_up": np.sqrt((E**2).mean(0)).round(4).tolist(),
            "coverage95_raw_variance": (Z**2 <= chi2.ppf(0.95, 1)).mean(0).round(4).tolist(),
            "neff_corr_scale": np.mean(ne, 0).round(2).tolist()}


sim_net = learning.load("results/motionnet.pt")
kitti_net = learning.load("results/kitti/motionnet_kitti.pt")
rep = {"train_imus": I.TRAIN_IMUS, "val_seconds": VAL_S, "aug": a.aug, "epochs": a.epochs}
for fold in "AB":
    (t0, t1), (s0, s1) = I.fold_ranges(fold)
    imus, vbs = [], []
    rng = np.random.default_rng(7)
    for name in I.TRAIN_IMUS:
        seq = I.segment(name, t0, t1 - VAL_S)
        imu6 = np.concatenate([seq.truth.gyro, seq.truth.accel], 1)
        for k in range(a.aug):
            off = np.zeros(6) if k == 0 else np.r_[rng.normal(0, 0.3 * DEG, 3), rng.normal(0, 20e-3 * 9.81, 3)]
            imus.append(imu6 + off)
            vbs.append(seq.truth.v_body)
    tic = time.time()
    net = learning.train_motionnet(imus, vbs, epochs=a.epochs, lr=1e-3, init=sim_net, stride=a.stride)
    print(f"fold {fold}: fine-tuned in {time.time() - tic:.0f}s", flush=True)
    learning.save(net, OUT / f"motionnet_fold{fold}.pt")
    val = [I.segment(n, t1 - VAL_S, t1) for n in I.TRAIN_IMUS]
    test = {n: I.segment(n, s0, s1) for n in I.IMUS}
    rep[fold] = {"train_range_s": [t0, t1 - VAL_S], "val_range_s": [t1 - VAL_S, t1], "test_range_s": [s0, s1]}
    for mname, m in (("ft", net), ("sim", sim_net), ("kitti", kitti_net)):
        r = {"val": stats(m, val)}
        r["test_per_imu"] = {n: stats(m, [s])["rmse_fwd_lat_up"] for n, s in test.items()}
        rep[fold][mname] = r
    print(json.dumps(rep[fold], indent=1), flush=True)
(OUT / "motionnet_val.json").write_text(json.dumps(rep, indent=1))
