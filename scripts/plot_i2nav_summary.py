"""Per-IMU bar chart: 120 s outage max error and in-outage 95% coverage (from results/i2nav/outage_metrics.json)."""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

OUT = Path("results/i2nav")
d = json.loads((OUT / "outage_metrics.json").read_text())
M = ["InEKF+NHC(未标定安装角)", "InEKF+NHC+安装角标定", "InEKF+NHC+安装角+相关σ",
     "v3+安装角标定(本文)[FT]", "v3+安装角+相关σ(本文)[FT]"]
LAB = ["NHC raw", "NHC+mount", "NHC+mount+corrσ", "v3+mount", "v3+mount+corrσ"]
imus = d["imus"]
x = np.arange(len(imus)); w = 0.16
fig, ax = plt.subplots(1, 2, figsize=(12, 4.2))
for i, m in enumerate(M):
    e = [d["aggregate_per_imu"][u][m]["max_err_outage_120s"]["mean"] for u in imus]
    c = [d["aggregate_per_imu"][u][m]["coverage95_h_outage"]["mean"] for u in imus]
    ax[0].bar(x + (i - 2) * w, e, w, label=LAB[i]); ax[1].bar(x + (i - 2) * w, c, w, label=LAB[i])
for a, t in zip(ax, ["120 s outage: mean max horizontal error [m]", "95% coverage inside outages"]):
    a.set_xticks(x); a.set_xticklabels([u + ("\n(train IMU)" if u in d["train_imus"] else "\n(unseen IMU)") for u in imus])
    a.set_title(t); a.grid(axis="y", alpha=.3)
ax[1].axhline(0.95, color="k", ls="--", lw=1); ax[0].legend(fontsize=8)
fig.suptitle("i2Nav awesome-gins (real MEMS + real RTK), held-out time halves, 24 outages per IMU per duration")
fig.tight_layout(); fig.savefig(OUT / "per_imu_summary.png", dpi=120)
print("saved", OUT / "per_imu_summary.png")
