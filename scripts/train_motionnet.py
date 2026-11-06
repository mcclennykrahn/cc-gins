"""Train MotionNet on simulated trajectories and report validation accuracy / calibration."""
import argparse
import json
import time
from pathlib import Path

import numpy as np
from scipy.stats import chi2

from ccgins import learning
from ccgins.scenarios import TRAIN_SEEDS, VAL_SEEDS, make_scenario

ap = argparse.ArgumentParser()
ap.add_argument("--architecture", choices=["legacy_cnn", "proposal_tcn"], default="legacy_cnn")
ap.add_argument("--variance-activation", choices=["hard", "smooth"], default="smooth")
ap.add_argument("--epochs", type=int, default=8)
ap.add_argument("--n-train", type=int, default=len(TRAIN_SEEDS))
ap.add_argument("--out", default="results/exploratory_motionnet.pt")
args = ap.parse_args()
Path(args.out).parent.mkdir(parents=True, exist_ok=True)

t0 = time.time()
imus, vbs, attitudes, biases = [], [], [], []
for s in TRAIN_SEEDS[: args.n_train]:
    sc = make_scenario(s)
    imus.append(sc.imu6())
    vbs.append(sc.truth.v_body)
    attitudes.append(learning.roll_pitch(sc.truth.R))
    biases.append(np.concatenate([sc.imu.gyro_bias, sc.imu.accel_bias], axis=1))
print(f"generated {len(imus)} training trajectories in {time.time() - t0:.1f}s")
t0 = time.time()
net = learning.train_motionnet(imus, vbs, epochs=args.epochs, architecture=args.architecture, variance_activation=args.variance_activation,
    attitude_list=attitudes if args.architecture == "proposal_tcn" else None,
    bias_list=biases if args.architecture == "proposal_tcn" else None)
print(f"trained in {time.time() - t0:.1f}s")
learning.save(net, args.out)

# validation
errs, z2 = [], []
for s in VAL_SEEDS:
    sc = make_scenario(s)
    idx = np.arange(learning.WINDOW, len(sc.truth.t), 10)
    mu, var = learning.predict(net, sc.imu6(), idx, attitude=learning.roll_pitch(sc.truth.R),
        bias=np.concatenate([sc.imu.gyro_bias, sc.imu.accel_bias], axis=1))
    y = sc.truth.v_body[idx - 1]
    errs.append(mu - y)
    z2.append((mu - y) ** 2 / var)
zs = [np.sign(e) * np.sqrt(q) for e, q in zip(errs, z2)]
neff = np.mean([learning.correlation_factor(z) for z in zs], axis=0)
errs, z2 = np.concatenate(errs), np.concatenate(z2)
rep = {
    "architecture": args.architecture,
    "variance_activation": net.variance_activation,
    "val_rmse_fwd_lat_up": np.sqrt((errs**2).mean(0)).round(4).tolist(),
    "val_coverage95_raw_variance": (z2 <= chi2.ppf(0.95, 1)).mean(0).round(4).tolist(),
    "val_mean_normalized_sq_err": z2.mean(0).round(3).tolist(),
    "neff_corr_scale": neff.round(2).tolist(),
}
print(json.dumps(rep, indent=1))
Path(args.out).with_suffix(".val.json").write_text(json.dumps(rep, indent=1))
