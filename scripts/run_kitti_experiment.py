"""KITTI-raw real-motion experiment.

Ground truth: OXTS RT3003 (pose + velocity) of held-out TEST drives; IMU: OXTS vehicle-frame
ax/ay/az, wx/wy/wz at 100 Hz + injected low-cost-MEMS errors (bias, bias RW, white noise; no
synthetic vibration); GNSS: synthetic 1 Hz from ground truth (1 m / 2 m white + 0.5 m GM multipath,
0.5/0/1.2 m lever arm), outages 30/60/120 s where the segment is long enough.
Learned models: "FT" = sim model fine-tuned on KITTI TRAIN drives; "sim" = zero-shot sim model
(a real sim-to-real domain shift). n_eff for each model from the KITTI VAL drive.

Outputs results/kitti/outage_{metrics.json,table.md,error.png}.
"""
import argparse
import json
import os
from multiprocessing import Pool
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
import numpy as np

from ccgins import learning
from ccgins.kitti_scenarios import TEST_DRIVES, drive_segments, make_kitti_scenario, outage_schedule
from ccgins.metrics import summarize
from ccgins.pipeline import Config, run

OUT = Path("results/kitti")
VAL = json.loads((OUT / "motionnet_val.json").read_text())
NEFF = {"ft": tuple(VAL["kitti_finetuned"]["neff_corr_scale"]), "sim": tuple(VAL["sim_only"]["neff_corr_scale"])}
MODELS = {"ft": str(OUT / "motionnet_kitti.pt"), "sim": "results/motionnet.pt"}
V2 = dict(filter="inekf", pseudo="hybrid", aci=True, aci_gamma=0.05, aci_window=100, hybrid_max_kappa=10.0)
V3 = dict(V2, hybrid_bias_z=4.0, hybrid_bias_latch=True)
LV = dict(filter="inekf", pseudo="learned_vel")
# (label, model or None, config without corr_scale, use corr_scale)
METHODS = [
    ("ESKF (纯惯导外推)", None, dict(filter="eskf", pseudo="none"), False),
    ("ESKF+NHC", None, dict(filter="eskf", pseudo="nhc"), False),
    ("InEKF+NHC", None, dict(filter="inekf", pseudo="nhc"), False),
    ("InEKF+学习速度(白噪声假设)[FT]", "ft", LV, False),
    ("InEKF+学习速度+相关膨胀[FT]", "ft", LV, True),
    ("InEKF+学习速度+相关膨胀+ACI(v1)[FT]", "ft", dict(LV, aci=True), True),
    ("InEKF+混合+相关膨胀[FT]", "ft", dict(filter="inekf", pseudo="hybrid"), True),
    ("v2[FT]", "ft", V2, True),
    ("v3(本文)[FT]", "ft", V3, True),
    ("InEKF+学习速度+相关膨胀+ACI(v1)[sim零样本]", "sim", dict(LV, aci=True), True),
    ("v2[sim零样本]", "sim", V2, True),
    ("v3(本文)[sim零样本]", "sim", V3, True),
]
_nets = {}


def job(a):
    import torch
    torch.set_num_threads(1)
    seg_id, drive, k, seed = a
    tr = drive_segments(drive, 100.0)[k]
    outs = outage_schedule(tr.t[-1])
    sc = make_kitti_scenario(tr, seed=seed, outages=outs)
    idx = np.arange(learning.WINDOW, len(tr.t), 10)
    preds = {}
    for m, path in MODELS.items():
        if m not in _nets:
            _nets[m] = learning.load(path)
        with torch.no_grad():
            mu, var = learning.predict(_nets[m], sc.imu6(), idx)
        preds[m] = (idx, mu, var)
    res, trace = {}, None
    for name, m, kw, corr in METHODS:
        cfg = Config(init_seed=seed, label=name, corr_scale=NEFF[m] if corr else None, **kw)
        log = run(tr, sc.imu, sc.gnss, sc.imu_spec, sc.gnss_spec, cfg, pred=preds[m] if m else None)
        r = summarize(log, outs, t_start=30.0)
        r["fallback_frac"] = float(log.extra.get("fallback_frac", 0.0))
        res[name] = r
        if seed == 0 and drive == "2011_09_30_drive_0028":
            trace = trace or {"outages": outs}
            trace[name] = (log.t, np.linalg.norm(log.p_est[:, :2] - log.p_true[:, :2], axis=1))
    print(f"{drive}#{k} seed {seed} outages {outs} done", flush=True)
    return seg_id, seed, outs, res, trace


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--workers", type=int, default=8)
    a = ap.parse_args()
    segs = []
    for d in TEST_DRIVES:
        for k, tr in enumerate(drive_segments(d, 100.0)):
            segs.append((f"{d}#{k}", d, k, round(float(tr.t[-1]), 1)))
    jobs = [(sid, d, k, s) for sid, d, k, _ in segs for s in range(a.seeds)]
    with Pool(a.workers) as p:
        out = p.map(job, jobs)
    trace = next(t for *_, t in out if t)
    names = [m[0] for m in METHODS]
    keys = ["rmse_h", "max_err_outage_30s", "max_err_outage_60s", "max_err_outage_120s",
            "end_err_outage_30s", "end_err_outage_60s", "end_err_outage_120s",
            "coverage95_h", "coverage95_h_outage", "anees_pos", "fallback_frac"]
    agg = {}
    for n in names:
        agg[n] = {}
        for k in keys:
            v = np.array([r[n][k] for *_, r, _ in out if k in r[n]])
            agg[n][k] = {"mean": float(v.mean()), "std": float(v.std()), "n": int(len(v))} if len(v) else None
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "outage_metrics.json").write_text(json.dumps({
        "segments": segs, "seeds": list(range(a.seeds)),
        "schedules": {sid: o for sid, _, o, _, _ in out},
        "neff": NEFF, "aggregate": agg,
        "per_run": [{"segment": sid, "seed": s, "res": r} for sid, s, _, r, _ in out]},
        indent=1, ensure_ascii=False))
    cols = [("rmse_h", "水平RMSE[m]"), ("max_err_outage_30s", "30s中断最大误差[m]"),
            ("max_err_outage_60s", "60s中断最大误差[m]"), ("max_err_outage_120s", "120s中断最大误差[m]"),
            ("coverage95_h_outage", "95%覆盖率(中断内)"), ("fallback_frac", "前向回退比例")]
    lines = ["| 方法 | " + " | ".join(c for _, c in cols) + " |", "|" + "---|" * (len(cols) + 1)]
    for n in names:
        cells = []
        for k, _ in cols:
            x = agg[n][k]
            if x is None:
                cells.append("-")
            elif k in ("coverage95_h_outage", "fallback_frac"):
                cells.append(f"{x['mean']:.3f}")
            else:
                cells.append(f"{x['mean']:.2f}±{x['std']:.2f}")
        lines.append(f"| {n} | " + " | ".join(cells) + " |")
    nrun = {d: agg[names[0]][f"max_err_outage_{d}s"]["n"] for d in (30, 60, 120)}
    md = "\n".join(lines) + (f"\n\n测试段: {[(s, L) for s, _, _, L in segs]}; 每段 {a.seeds} 个噪声种子; "
                             f"各中断时长样本数: {nrun}\n")
    (OUT / "outage_table.md").write_text(md)
    print(md)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.sans-serif"] = ["Noto Sans CJK SC", "WenQuanYi Zen Hei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    fig, ax = plt.subplots(figsize=(11, 4.5))
    for n in ["ESKF (纯惯导外推)", "InEKF+NHC", "InEKF+学习速度+相关膨胀[FT]", "v3(本文)[FT]",
              "InEKF+学习速度+相关膨胀+ACI(v1)[sim零样本]", "v3(本文)[sim零样本]"]:
        t, e = trace[n]
        ax.plot(t, e, lw=1, label=n)
    for s, d in trace["outages"]:
        ax.axvspan(s, s + d, color="grey", alpha=0.2)
    ax.set_yscale("log")
    ax.set_xlabel("time [s] (grey = GNSS outage)")
    ax.set_ylabel("horizontal error [m]")
    ax.set_title("KITTI 2011_09_30_drive_0028 (held-out), seed 0")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(OUT / "outage_error.png", dpi=120)


if __name__ == "__main__":
    main()
