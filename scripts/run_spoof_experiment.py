"""Slow-drift spoofing experiment: single-channel (NIS) CUSUM vs dual-channel (DR) CUSUM.

1. Thresholds h: calibrated on clean CALIB_SEEDS as 1.5 x max CUSUM statistic (no alarms).
2. Test on TEST_SEEDS: false alarms on clean runs; detection delay and max horizontal error
   after spoof onset for ramp spoofing at several drift rates (alarm -> GNSS excluded).
Outputs results/spoof_metrics.json, results/spoof_table.md, results/spoof_stats.png.
"""
import argparse
import json
import os
from multiprocessing import Pool
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
import numpy as np

from ccgins import learning
from ccgins.pipeline import Config, run
from ccgins.scenarios import CALIB_SEEDS, TEST_SEEDS, make_scenario

NEFF = tuple(json.loads(Path("results/motionnet_val.json").read_text())["neff_corr_scale"])
BASE = dict(filter="inekf", pseudo="learned_vel", corr_scale=NEFF, aci=True)
SPOOF_START = 300.0
RATES = [0.2, 0.5, 1.0]
_net = None


def _pred(sc):
    import torch
    global _net
    torch.set_num_threads(1)
    if _net is None:
        _net = learning.load("results/motionnet.pt")
    idx = np.arange(learning.WINDOW, len(sc.truth.t), 10)
    with torch.no_grad():
        mu, var = learning.predict(_net, sc.imu6(), idx)
    return idx, mu, var


def _run(sc, pred, seed, **kw):
    cfg = Config(init_seed=seed, **BASE, **kw)
    return run(sc.truth, sc.imu, sc.gnss, sc.imu_spec, sc.gnss_spec, cfg, pred=pred)


def calib(seed):
    sc = make_scenario(seed)
    pred = _pred(sc)
    out = {}
    for mon in ("single", "dual"):
        log = _run(sc, pred, seed, monitor=mon, cusum_h=1e12)
        out[mon] = float(log.stat_g[log.gnss_t > 30].max())
    return out


def test(args):
    seed, H = args
    res = {"clean": {}, "spoof": {}}
    sc = make_scenario(seed)
    pred = _pred(sc)
    for mon in ("single", "dual"):
        log = _run(sc, pred, seed, monitor=mon, cusum_h=H[mon])
        res["clean"][mon] = log.alarm_time
    trace = None
    for rate in RATES:
        sc = make_scenario(seed, spoof_start=SPOOF_START, spoof_rate=rate)
        pred = _pred(sc)
        r = {}
        for mon in ("none", "single", "dual"):
            log = _run(sc, pred, seed, monitor=mon, cusum_h=H.get(mon, 1.0))
            eh = np.linalg.norm(log.p_est[:, :2] - log.p_true[:, :2], axis=1)
            m = log.t >= SPOOF_START
            r[mon] = {"alarm_time": log.alarm_time, "max_err_after": float(eh[m].max()),
                      "end_err": float(eh[-1])}
            if seed == TEST_SEEDS[0] and rate == 0.5 and mon != "none":
                trace = trace or {}
                trace[mon] = (log.gnss_t.tolist(), log.stat_q.tolist(), log.stat_g.tolist())
            if seed == TEST_SEEDS[0] and rate == 0.5:
                trace = trace or {}
                trace["err_" + mon] = (log.t.tolist(), eh.tolist())
        res["spoof"][str(rate)] = r
    print(f"seed {seed} done", flush=True)
    return seed, res, trace


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=len(TEST_SEEDS))
    ap.add_argument("--workers", type=int, default=5)
    a = ap.parse_args()
    with Pool(a.workers) as pool:
        cal = pool.map(calib, CALIB_SEEDS)
        H = {m: 1.5 * max(c[m] for c in cal) for m in ("single", "dual")}
        print("calibrated h:", H, flush=True)
        out = pool.map(test, [(s, H) for s in TEST_SEEDS[: a.seeds]])
    seeds = [s for s, _, _ in out]
    per = {s: r for s, r, _ in out}
    trace = next(t for _, _, t in out if t)
    summ = {"h": H, "calib_max_g": cal, "false_alarms_clean": {}, "by_rate": {}}
    for mon in ("single", "dual"):
        summ["false_alarms_clean"][mon] = int(sum(per[s]["clean"][mon] is not None for s in seeds))
    lines = ["| 漂移速率[m/s] | 方法 | 检出率 | 平均检测延迟[s] | 欺骗前误报 | 欺骗后最大水平误差[m] |",
             "|---|---|---|---|---|---|"]
    for rate in RATES:
        summ["by_rate"][str(rate)] = {}
        for mon in ("none", "single", "dual"):
            rs = [per[s]["spoof"][str(rate)][mon] for s in seeds]
            at = [r["alarm_time"] for r in rs]
            early = sum(t is not None and t < SPOOF_START for t in at)
            det = [t - SPOOF_START for t in at if t is not None and t >= SPOOF_START]
            me = np.array([r["max_err_after"] for r in rs])
            d = {"detection_rate": len(det) / len(rs), "mean_delay_s": float(np.mean(det)) if det else None,
                 "pre_spoof_alarms": early, "max_err_after_mean": float(me.mean()),
                 "max_err_after_std": float(me.std())}
            summ["by_rate"][str(rate)][mon] = d
            name = {"none": "无监测", "single": "单通道 NIS-CUSUM", "dual": "双通道 DR-CUSUM(本文)"}[mon]
            lines.append(f"| {rate} | {name} | " + ("-" if mon == "none" else f"{d['detection_rate']:.0%}") +
                         " | " + (f"{d['mean_delay_s']:.1f}" if d["mean_delay_s"] is not None else "-") +
                         f" | {early if mon != 'none' else '-'} | {d['max_err_after_mean']:.2f}±{d['max_err_after_std']:.2f} |")
    md = "\n".join(lines)
    fa = summ["false_alarms_clean"]
    md += (f"\n\n干净数据误报(测试种子 {len(seeds)} 条×600 s): 单通道 {fa['single']}, 双通道 {fa['dual']}; "
           f"阈值 h(由 CALIB 种子 1.5×最大值标定): 单通道 {H['single']:.1f}, 双通道 {H['dual']:.1f}\n")
    print(md)
    Path("results/spoof_metrics.json").write_text(json.dumps({"seeds": seeds, "summary": summ,
                                                             "per_seed": {str(s): per[s] for s in seeds}},
                                                            indent=1, ensure_ascii=False, default=lambda o: o.item()))
    Path("results/spoof_table.md").write_text(md)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for mon, c in (("single", "C0"), ("dual", "C3")):
        t, q, g = trace[mon]
        ax[0].plot(t, q, c, lw=0.7, label=mon)
        ax[1].plot(t, g, c, lw=1, label=mon)
        ax[1].axhline(H[mon], color=c, ls="--", lw=0.8)
    for mon, c in (("none", "k"), ("single", "C0"), ("dual", "C3")):
        t, e = trace["err_" + mon]
        ax[2].plot(t, e, c, lw=1, label=mon)
    for x in ax:
        x.axvline(SPOOF_START, color="grey", ls=":")
    ax[0].set_yscale("log")
    ax[0].set_ylabel("test statistic q")
    ax[1].set_yscale("symlog", linthresh=1)
    ax[1].set_ylim(bottom=0)
    ax[1].set_ylabel("CUSUM g (dashed: h)")
    ax[2].set_ylabel("horizontal error [m]")
    ax[2].set_xlabel(f"time [s]; ramp spoof 0.5 m/s from {SPOOF_START:.0f} s, seed {TEST_SEEDS[0]}")
    for x in ax:
        x.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig("results/spoof_stats.png", dpi=120)


if __name__ == "__main__":
    main()
