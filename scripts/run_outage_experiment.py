"""GNSS-outage experiment: compare baselines vs learned pseudo-measurements + ACI.

Outputs results/outage_metrics.json, results/outage_table.md, results/outage_error.png,
results/outage_consistency.png.
"""
import argparse
import json
import os
from multiprocessing import Pool
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")

import numpy as np

from ccgins import learning
from ccgins.metrics import summarize
from ccgins.pipeline import Config, run
from ccgins.sim import ImuSpec
from ccgins.scenarios import OUTAGES, TEST_SEEDS, make_scenario

NEFF = tuple(json.loads(Path("results/motionnet_val.json").read_text())["neff_corr_scale"])
LV = dict(filter="inekf", pseudo="learned_vel")
# v2 (selected on VAL seeds, see scripts/sweep_hybrid_val.py): hybrid pseudo-measurement,
# faster ACI (gamma=0.05, window=100 obs = 10 s) and fallback to NHC-only when kappa_fwd > 10
V2 = dict(pseudo="hybrid", corr_scale=NEFF, aci=True, aci_gamma=0.05, aci_window=100, hybrid_max_kappa=10.0)
# v3 (selected on VAL seeds, see scripts/sweep_bias_val.py): v2 + latched residual-bias monitor
V3 = dict(V2, hybrid_bias_z=4.0, hybrid_bias_latch=True)
METHODS = {
    "ESKF (纯惯导外推)": dict(filter="eskf", pseudo="none"),
    "ESKF+NHC": dict(filter="eskf", pseudo="nhc"),
    "InEKF+NHC": dict(filter="inekf", pseudo="nhc"),
    "InEKF+学习NHC": dict(filter="inekf", pseudo="learned_nhc"),
    "InEKF+学习速度(白噪声假设)": dict(LV),
    "InEKF+学习速度+相关膨胀": dict(LV, corr_scale=NEFF),
    "InEKF+学习速度+相关膨胀+ACI(β=0)": dict(LV, corr_scale=NEFF, aci=True, aci_beta=0.0),
    "ESKF+学习速度+相关膨胀+ACI": dict(LV, filter="eskf", corr_scale=NEFF, aci=True),
    "InEKF+学习速度+相关膨胀+ACI(v1)": dict(LV, corr_scale=NEFF, aci=True),
    "InEKF+混合(NHC侧/垂+学习前向)+相关膨胀": dict(filter="inekf", pseudo="hybrid", corr_scale=NEFF),
    "InEKF+混合+相关膨胀+快速ACI+κ回退(v2)": dict(filter="inekf", **V2),
    "InEKF+v2+偏差监测锁存回退(v3,本文)": dict(filter="inekf", **V3),
}
SHIFT_SPEC = ImuSpec(vib_accel_per_mps=0.03, vib_gyro_per_mps=0.003, accel_bias_sigma=8.0)
SHIFT = os.environ.get("CCGINS_SHIFT", "0") == "1"
MODEL = "results/motionnet.pt"


def _init(shift):
    global SHIFT
    SHIFT = shift


def one_seed(seed):
    import torch
    torch.set_num_threads(1)
    net = learning.load(MODEL)
    sc = make_scenario(seed, outages=OUTAGES, imu_spec=SHIFT_SPEC if SHIFT else None)
    idx = np.arange(learning.WINDOW, len(sc.truth.t), 10)
    with torch.no_grad():
        mu, var = learning.predict(net, sc.imu6(), idx)
    res, traces = {}, {}
    for name, kw in METHODS.items():
        cfg = Config(init_seed=seed, label=name, **kw)
        log = run(sc.truth, sc.imu, sc.gnss, sc.imu_spec, sc.gnss_spec, cfg, pred=(idx, mu, var))
        res[name] = summarize(log, OUTAGES)
        res[name]["fallback_frac"] = float(log.extra.get("fallback_frac", 0.0))
        if seed == TEST_SEEDS[0]:
            traces[name] = dict(t=log.t, eh=np.linalg.norm(log.p_est[:, :2] - log.p_true[:, :2], axis=1),
                                sig=np.sqrt(np.trace(log.P_pos[:, :2, :2], axis1=1, axis2=2)),
                                kappa=log.kappa_hist)
    print(f"seed {seed} done", flush=True)
    return seed, res, traces


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=len(TEST_SEEDS))
    ap.add_argument("--workers", type=int, default=5)
    ap.add_argument("--shift", action="store_true", help="test under IMU/vibration distribution shift")
    a = ap.parse_args()
    if a.shift:
        os.environ["CCGINS_SHIFT"] = "1"
    tag = "_shift" if a.shift else ""
    seeds = TEST_SEEDS[: a.seeds]
    with Pool(a.workers, initializer=_init, initargs=(a.shift,)) as pool:
        out = pool.map(one_seed, seeds)
    per_seed = {s: r for s, r, _ in out}
    traces = next(tr for s, _, tr in out if tr)
    keys = list(next(iter(per_seed.values()))[next(iter(METHODS))].keys())
    agg = {}
    for name in METHODS:
        agg[name] = {}
        for k in keys:
            v = np.array([per_seed[s][name][k] for s in seeds])
            agg[name][k] = {"mean": float(v.mean()), "std": float(v.std())}
    Path("results").mkdir(exist_ok=True)
    Path(f"results/outage{tag}_metrics.json").write_text(json.dumps(
        {"seeds": seeds, "outages": OUTAGES, "aggregate": agg,
         "per_seed": {str(s): per_seed[s] for s in seeds}}, indent=1, ensure_ascii=False))

    cols = [("rmse_h", "水平RMSE[m]"), ("max_err_outage_30s", "30s中断最大误差[m]"),
            ("max_err_outage_60s", "60s中断最大误差[m]"), ("max_err_outage_120s", "120s中断最大误差[m]"),
            ("coverage95_h", "95%覆盖率(全程)"), ("coverage95_h_outage", "95%覆盖率(中断内)"),
            ("anees_pos", "ANEES(位置)"), ("runtime_us", "耗时[µs/步]")]
    lines = ["| 方法 | " + " | ".join(c for _, c in cols) + " |", "|" + "---|" * (len(cols) + 1)]
    for name in METHODS:
        cells = []
        for k, _ in cols:
            m, s = agg[name][k]["mean"], agg[name][k]["std"]
            cells.append(f"{m:.3f}" if k.startswith("coverage") else
                         f"{m:.0f}" if k == "runtime_us" else f"{m:.2f}±{s:.2f}")
        lines.append(f"| {name} | " + " | ".join(cells) + " |")
    md = "\n".join(lines)
    Path(f"results/outage{tag}_table.md").write_text(md + f"\n\n种子: {seeds}; 中断: {OUTAGES}\n")
    print(md)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.sans-serif"] = ["Noto Sans CJK SC", "WenQuanYi Zen Hei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    show = ["ESKF+NHC", "InEKF+学习速度(白噪声假设)", "InEKF+学习速度+相关膨胀+ACI(v1)", "InEKF+混合+相关膨胀+快速ACI+κ回退(v2)", "InEKF+v2+偏差监测锁存回退(v3,本文)"]
    fig, ax = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    for name in show:
        tr = traces[name]
        l, = ax[0].plot(tr["t"], tr["eh"], label=name, lw=1)
        ax[1].plot(tr["t"], tr["eh"] / np.maximum(tr["sig"], 1e-6), color=l.get_color(), lw=1)
    for s, d in OUTAGES:
        for x in ax:
            x.axvspan(s, s + d, color="grey", alpha=0.2)
    ax[0].set_ylabel("horizontal error [m]")
    ax[0].set_yscale("log")
    ax[0].legend(fontsize=8)
    ax[1].axhline(2.45, ls="--", color="k", lw=0.8)
    ax[1].set_ylabel("error / sigma_h")
    ax[1].set_xlabel("time [s] (grey = GNSS outage)")
    ax[0].set_title(f"seed {TEST_SEEDS[0]}")
    fig.tight_layout()
    fig.savefig(f"results/outage{tag}_error.png", dpi=120)

    fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))
    names = list(METHODS)
    y = np.arange(len(names))
    ax[0].barh(y, [agg[n]["max_err_outage_120s"]["mean"] for n in names],
               xerr=[agg[n]["max_err_outage_120s"]["std"] for n in names])
    ax[0].set_yticks(y, names, fontsize=8)
    ax[0].set_xlabel("max horizontal error in 120 s outage [m]")
    ax[1].barh(y, [agg[n]["coverage95_h_outage"]["mean"] for n in names])
    ax[1].axvline(0.95, ls="--", color="k")
    ax[1].set_yticks(y, [""] * len(names))
    ax[1].set_xlabel("95% coverage inside outages")
    fig.tight_layout()
    fig.savefig(f"results/outage{tag}_consistency.png", dpi=120)


if __name__ == "__main__":
    main()
