"""REAL-data GNSS-outage experiment on i2Nav-WHU awesome-gins-datasets (4 real MEMS IMUs, real RTK).

Per fold (A/B time split), per IMU, per run: test half (~800 s), real 1 Hz RTK with injected outages
(30/120/60/30/120/60 s, first outage at 50/70/90 s), init seeds 0/1 (random initial-state errors).
Earth rotation + local gravity enabled in the mechanization. Mounting angles calibrated per IMU on
the TRAINING half with the RTK/INS filter only (no truth). n_eff per model from the validation slice.

Outputs results/i2nav/outage_{metrics.json,table.md,paired.md,error.png}.
"""
import argparse
import json
import os
from multiprocessing import Pool
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
import numpy as np

from ccgins import i2nav_scenarios as I
from ccgins import learning
from ccgins.metrics import summarize
from ccgins.pipeline import Config, run

OUT = Path("results/i2nav")
VAL = json.loads((OUT / "motionnet_val.json").read_text())
V2 = dict(filter="inekf", pseudo="hybrid", aci=True, aci_gamma=0.05, aci_window=100, hybrid_max_kappa=10.0)
V3 = dict(V2, hybrid_bias_z=4.0, hybrid_bias_latch=True)
LV = dict(filter="inekf", pseudo="learned_vel")
BASE = "InEKF+NHC+安装角标定"
BASE2 = "InEKF+NHC+安装角+相关σ"
# (label, model or None, cfg, use corr_scale, use mount calibration; "eff" = mount + correlation-aware NHC sigma)
METHODS = [
    ("ESKF (纯惯导外推)", None, dict(filter="eskf", pseudo="none"), False, False),
    ("ESKF+NHC(未标定安装角)", None, dict(filter="eskf", pseudo="nhc"), False, False),
    ("InEKF+NHC(未标定安装角)", None, dict(filter="inekf", pseudo="nhc"), False, False),
    (BASE, None, dict(filter="inekf", pseudo="nhc"), False, True),
    (BASE2, None, dict(filter="inekf", pseudo="nhc"), False, "eff"),
    ("InEKF+学习速度(白噪声假设)[FT]", "ft", LV, False, False),
    ("InEKF+学习速度+相关膨胀[FT]", "ft", LV, True, False),
    ("InEKF+学习速度+相关膨胀+ACI(v1)[FT]", "ft", dict(LV, aci=True), True, False),
    ("v2+安装角标定[FT]", "ft", V2, True, True),
    ("v3(未标定安装角)[FT]", "ft", V3, True, False),
    ("v3+安装角标定(本文)[FT]", "ft", V3, True, True),
    ("v3+安装角+相关σ(本文)[FT]", "ft", V3, True, "eff"),
    ("InEKF+学习速度+相关膨胀+ACI(v1)[sim零样本]", "sim", dict(LV, aci=True), True, False),
    ("v3+安装角标定[sim零样本]", "sim", V3, True, True),
    ("v3+安装角标定[KITTI模型零样本]", "kitti", V3, True, True),
]
MODEL_PATH = {"sim": "results/motionnet.pt", "kitti": "results/kitti/motionnet_kitti.pt"}
FIRSTS = (50.0, 70.0, 90.0)
INIT_SEEDS = (0, 1)


def calib_job(a):
    name, fold = a
    tr, _ = I.fold_ranges(fold)
    m, sig, sig_eff, neff = I.calibrate_mount(I.segment(name, *tr), return_eff=True)
    return (name, fold), {"pitch_deg": float(np.degrees(m[0])), "yaw_deg": float(np.degrees(m[1])),
                          "mount_py": list(m), "resid_sigma_lat_vert": list(sig), "neff_lat_vert": list(neff),
                          "nhc_sigma_eff": sig_eff}


def job(a):
    import torch
    torch.set_num_threads(1)
    name, fold, first, seed, calib = a
    _, te = I.fold_ranges(fold)
    seq = I.segment(name, *te)
    I.enable_earth(seq)
    outs = I.outage_schedule(seq.truth.t[-1], first=first)
    sc = I.make_scenario(seq, seed, outs)
    idx = np.arange(learning.WINDOW, len(seq.truth.t), 10)
    preds = {}
    for m in ("ft", "sim", "kitti"):
        net = learning.load(OUT / f"motionnet_fold{fold}.pt" if m == "ft" else MODEL_PATH[m])
        with torch.no_grad():
            mu, var = learning.predict(net, sc.imu6(), idx)
        preds[m] = (idx, mu, var)
    res, trace = {}, None
    for lab, m, kw, corr, mount in METHODS:
        cfg = Config(init_seed=seed, label=lab, **I.REAL_CFG, **kw,
                     corr_scale=tuple(VAL[fold][m]["val"]["neff_corr_scale"]) if corr else None,
                     mount_py=tuple(calib[f"{name}/{fold}"]["mount_py"]) if mount else None,
                     nhc_sigma=calib[f"{name}/{fold}"]["nhc_sigma_eff"] if mount == "eff" else 0.1)
        log = run(seq.truth, sc.imu, sc.gnss, sc.imu_spec, sc.gnss_spec, cfg, pred=preds[m] if m else None)
        r = summarize(log, outs, t_start=30.0)
        r["fallback_frac"] = float(log.extra.get("fallback_frac", 0.0))
        r["per_outage_max"] = per_outage_max(log, outs)
        res[lab] = r
        if name == "ADIS16465" and fold == "A" and first == FIRSTS[0] and seed == 0:
            trace = trace or {"outages": outs}
            trace[lab] = (log.t.tolist(), np.linalg.norm(log.p_est[:, :2] - log.p_true[:, :2], axis=1).tolist())
    print(f"{name} fold {fold} first {first} seed {seed} done", flush=True)
    return name, fold, first, seed, outs, res, trace


def per_outage_max(log, outs):
    e = np.linalg.norm(log.p_est[:, :2] - log.p_true[:, :2], axis=1)
    return [[float(d), float(e[(log.t >= s) & (log.t <= s + d)].max())] for s, d in outs]


def paired(out, a, b, dur):
    """per-outage paired difference of max horizontal error: method a - baseline b."""
    da, db = [], []
    for *_, res, _ in out:
        for (d1, x), (d2, y) in zip(res[a]["per_outage_max"], res[b]["per_outage_max"]):
            if d1 == dur:
                da.append(x); db.append(y)
    da, db = np.array(da), np.array(db)
    diff = da - db
    rng = np.random.default_rng(0)
    bs = [rng.choice(diff, len(diff)).mean() for _ in range(2000)]
    from scipy.stats import wilcoxon
    p = float(wilcoxon(diff).pvalue) if np.any(diff != 0) else 1.0
    return {"n": int(len(diff)), "mean_a": float(da.mean()), "mean_b": float(db.mean()),
            "median_a": float(np.median(da)), "median_b": float(np.median(db)),
            "mean_diff": float(diff.mean()), "ci95": [float(np.percentile(bs, 2.5)), float(np.percentile(bs, 97.5))],
            "rel_change": float(diff.mean() / db.mean()), "win_rate": float((diff < 0).mean()), "wilcoxon_p": p}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=8)
    a = ap.parse_args()
    with Pool(a.workers) as p:
        calib = {f"{n}/{f}": c for (n, f), c in p.map(calib_job, [(n, f) for n in I.IMUS for f in "AB"])}
    (OUT / "mount_calibration.json").write_text(json.dumps(calib, indent=1))
    print(json.dumps(calib, indent=1), flush=True)
    jobs = [(n, f, fi, s, calib) for f in "AB" for n in I.IMUS for fi in FIRSTS for s in INIT_SEEDS]
    with Pool(a.workers) as p:
        out = p.map(job, jobs)
    trace = next(t for *_, t in out if t)
    names = [m[0] for m in METHODS]
    keys = ["rmse_h", "max_err_outage_30s", "max_err_outage_60s", "max_err_outage_120s",
            "end_err_outage_30s", "end_err_outage_60s", "end_err_outage_120s",
            "coverage95_h", "coverage95_h_outage", "anees_pos", "fallback_frac"]

    def agg_of(rows):
        ag = {}
        for n in names:
            ag[n] = {}
            for k in keys:
                if k.startswith("max_err_outage_"):  # all outages of that duration (summarize keeps only the last)
                    d = float(k.split("_")[-1][:-1])
                    v = np.array([x for r in rows for dd, x in r[n]["per_outage_max"] if dd == d])
                else:
                    v = np.array([r[n][k] for r in rows if k in r[n]])
                ag[n][k] = {"mean": float(v.mean()), "std": float(v.std()), "n": int(len(v))} if len(v) else None
        return ag

    agg = agg_of([o[5] for o in out])
    agg_imu = {n: agg_of([o[5] for o in out if o[0] == n]) for n in I.IMUS}
    pairs = {}
    for lab in names:
        if lab == BASE:
            continue
        pairs[lab] = {d: paired(out, lab, BASE, d) for d in (30.0, 60.0, 120.0)}
    pairs2 = {lab: {d: paired(out, lab, BASE2, d) for d in (30.0, 60.0, 120.0)}
              for lab in ("v3+安装角+相关σ(本文)[FT]", "v3+安装角标定(本文)[FT]", "InEKF+学习速度+相关膨胀[FT]")}
    pairs_imu = {}
    for n in I.IMUS:
        sub = [o for o in out if o[0] == n]
        pairs_imu[n] = {lab: {d: paired(sub, lab, BASE, d) for d in (60.0, 120.0)}
                        for lab in ("v3+安装角标定(本文)[FT]", "v3+安装角+相关σ(本文)[FT]", BASE2,
                                    "InEKF+NHC(未标定安装角)", "ESKF (纯惯导外推)")}
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "outage_metrics.json").write_text(json.dumps({
        "imus": I.IMUS, "train_imus": I.TRAIN_IMUS, "firsts": FIRSTS, "init_seeds": INIT_SEEDS,
        "mount_calibration": calib, "aggregate": agg, "aggregate_per_imu": agg_imu,
        "paired_vs_base": {k: {str(d): v for d, v in x.items()} for k, x in pairs.items()},
        "paired_vs_base2": {k: {str(d): v for d, v in x.items()} for k, x in pairs2.items()},
        "paired_vs_base_per_imu": {n: {k: {str(d): v for d, v in x.items()} for k, x in y.items()}
                                   for n, y in pairs_imu.items()},
        "per_run": [{"imu": n, "fold": f, "first": fi, "seed": s, "outages": o,
                     "res": {k: {kk: vv for kk, vv in v.items()} for k, v in r.items()}}
                    for n, f, fi, s, o, r, _ in out]}, indent=1, ensure_ascii=False))

    cols = [("rmse_h", "水平RMSE[m]"), ("max_err_outage_30s", "30s中断最大误差[m]"),
            ("max_err_outage_60s", "60s中断最大误差[m]"), ("max_err_outage_120s", "120s中断最大误差[m]"),
            ("coverage95_h_outage", "95%覆盖率(中断内)"), ("fallback_frac", "前向回退比例")]

    def table(ag):
        lines = ["| 方法 | " + " | ".join(c for _, c in cols) + " |", "|" + "---|" * (len(cols) + 1)]
        for n in names:
            cells = []
            for k, _ in cols:
                x = ag[n][k]
                if x is None:
                    cells.append("-")
                elif k in ("coverage95_h_outage", "fallback_frac"):
                    cells.append(f"{x['mean']:.3f}")
                else:
                    cells.append(f"{x['mean']:.2f}±{x['std']:.2f}")
            lines.append(f"| {n} | " + " | ".join(cells) + " |")
        return "\n".join(lines)

    nrun = len(out)
    md = (f"## 全部 4 个 IMU（{nrun} 次运行 = 2 折 × 4 IMU × 3 中断起点 × 2 初始误差种子）\n\n" + table(agg) + "\n")
    for n in I.IMUS:
        md += f"\n## {n}{'（训练用 IMU）' if n in I.TRAIN_IMUS else '（未见过的 IMU）'}\n\n" + table(agg_imu[n]) + "\n"
    (OUT / "outage_table.md").write_text(md)
    pl = ["| 方法 (相对 " + BASE + ") | 时长 | n | 方法均值[m] | 基线均值[m] | 差值均值[m] (95% CI) | 相对变化 | 胜率 | Wilcoxon p |",
          "|---|---|---|---|---|---|---|---|---|"]
    for lab, x in pairs.items():
        for d, s in x.items():
            pl.append(f"| {lab} | {d:.0f}s | {s['n']} | {s['mean_a']:.2f} | {s['mean_b']:.2f} | {s['mean_diff']:+.2f} "
                      f"[{s['ci95'][0]:+.2f}, {s['ci95'][1]:+.2f}] | {100 * s['rel_change']:+.1f}% | {s['win_rate']:.2f} | {s['wilcoxon_p']:.2g} |")
    pl += ["", f"### 相对更强基线 {BASE2}", "", "| 方法 | 时长 | n | 方法均值[m] | 基线均值[m] | 差值均值[m] (95% CI) | 相对变化 | 胜率 | Wilcoxon p |",
           "|---|---|---|---|---|---|---|---|---|"]
    for lab, x in pairs2.items():
        for d, s in x.items():
            pl.append(f"| {lab} | {d:.0f}s | {s['n']} | {s['mean_a']:.2f} | {s['mean_b']:.2f} | {s['mean_diff']:+.2f} "
                      f"[{s['ci95'][0]:+.2f}, {s['ci95'][1]:+.2f}] | {100 * s['rel_change']:+.1f}% | {s['win_rate']:.2f} | {s['wilcoxon_p']:.2g} |")
    pl += ["", "### 按 IMU（相对 " + BASE + "）", "", "| IMU | 方法 | 时长 | n | 差值均值[m] (95% CI) | 相对变化 | 胜率 | p |", "|---|---|---|---|---|---|---|---|"]
    for n, y in pairs_imu.items():
        for lab, x in y.items():
            for d, s in x.items():
                pl.append(f"| {n} | {lab} | {d:.0f}s | {s['n']} | {s['mean_diff']:+.2f} [{s['ci95'][0]:+.2f}, {s['ci95'][1]:+.2f}] "
                          f"| {100 * s['rel_change']:+.1f}% | {s['win_rate']:.2f} | {s['wilcoxon_p']:.2g} |")
    (OUT / "outage_paired.md").write_text("\n".join(pl) + "\n")
    print(md)
    print("\n".join(pl))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.sans-serif"] = ["Noto Sans CJK SC", "WenQuanYi Zen Hei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    fig, ax = plt.subplots(figsize=(11, 4.5))
    st = {BASE: dict(ls="--", lw=1.6, color="k"), "InEKF+NHC(未标定安装角)": dict(ls=":", lw=1.4, color="grey")}
    st[BASE2] = dict(ls="-.", lw=1.4, color="C2")
    for n in ["ESKF (纯惯导外推)", "InEKF+NHC(未标定安装角)", BASE, BASE2, "InEKF+学习速度+相关膨胀[FT]",
              "v3+安装角标定(本文)[FT]", "v3+安装角+相关σ(本文)[FT]"]:
        t, e = trace[n]
        ax.plot(t, e, label=n, **st.get(n, dict(lw=1)))
    for s, d in trace["outages"]:
        ax.axvspan(s, s + d, color="grey", alpha=0.2)
    ax.set_yscale("log")
    ax.set_xlabel("time [s] (grey = injected GNSS outage on real RTK)")
    ax.set_ylabel("horizontal error [m]")
    ax.set_title("i2Nav awesome-gins, ADIS16465 (unseen IMU), fold A test half, seed 0")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(OUT / "outage_error.png", dpi=120)


if __name__ == "__main__":
    main()
