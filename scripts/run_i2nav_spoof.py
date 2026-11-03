"""REAL-data slow-drift spoofing on i2Nav awesome-gins (real RTK + additive ramp), NIS-CUSUM vs DR-CUSUM.

Aiding: v3 + mounting calibration + correlation-aware NHC sigma with the fold's fine-tuned MotionNet.
1. Thresholds h = 1.5 x max CUSUM statistic on CLEAN runs over each fold's TRAINING half
   (4 IMUs x 2 folds x init seeds 0,1); one h per monitor.
2. Test halves: clean runs (false alarms), then ramp spoofing starting 300 s into the half at
   0.2 / 0.5 / 1.0 m/s (45 deg), alarm -> GNSS excluded. Detection delay and max error after onset.
Outputs results/i2nav/spoof_{metrics.json,table.md}.
"""
import json
import os
from multiprocessing import Pool
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
import numpy as np

from ccgins import i2nav_scenarios as I
from ccgins import learning
from ccgins.pipeline import Config, run

OUT = Path("results/i2nav")
VAL = json.loads((OUT / "motionnet_val.json").read_text())
CAL = json.loads((OUT / "mount_calibration.json").read_text())
V3 = dict(filter="inekf", pseudo="hybrid", aci=True, aci_gamma=0.05, aci_window=100, hybrid_max_kappa=10.0,
          hybrid_bias_z=4.0, hybrid_bias_latch=True)
SPOOF_START = 300.0
RATES = [0.2, 0.5, 1.0]
SEEDS = (0, 1)


def _setup(name, fold, half, seed, **spoof):
    import torch
    torch.set_num_threads(1)
    tr, te = I.fold_ranges(fold)
    seq = I.segment(name, *(tr if half == "train" else te))
    I.enable_earth(seq)
    sc = I.make_scenario(seq, seed, **spoof)
    net = learning.load(OUT / f"motionnet_fold{fold}.pt")
    idx = np.arange(learning.WINDOW, len(seq.truth.t), 10)
    with torch.no_grad():
        mu, var = learning.predict(net, sc.imu6(), idx)
    return seq, sc, (idx, mu, var)


def _run(name, fold, seq, sc, pred, seed, **kw):
    cfg = Config(init_seed=seed, **I.REAL_CFG, **V3, corr_scale=tuple(VAL[fold]["ft"]["val"]["neff_corr_scale"]),
                 mount_py=tuple(CAL[f"{name}/{fold}"]["mount_py"]), nhc_sigma=CAL[f"{name}/{fold}"]["nhc_sigma_eff"], **kw)
    return run(seq.truth, sc.imu, sc.gnss, sc.imu_spec, sc.gnss_spec, cfg, pred=pred)


def calib(a):
    name, fold, seed = a
    seq, sc, pred = _setup(name, fold, "train", seed)
    out = {}
    for mon in ("single", "dual"):
        log = _run(name, fold, seq, sc, pred, seed, monitor=mon, cusum_h=1e12)
        out[mon] = float(log.stat_g[log.gnss_t > 30].max())
    return out


def test(a):
    name, fold, seed, H = a
    res = {"clean": {}, "spoof": {}}
    seq, sc, pred = _setup(name, fold, "test", seed)
    for mon in ("single", "dual"):
        res["clean"][mon] = _run(name, fold, seq, sc, pred, seed, monitor=mon, cusum_h=H[mon]).alarm_time
    for rate in RATES:
        seq, sc, pred = _setup(name, fold, "test", seed, spoof_start=SPOOF_START, spoof_rate=rate)
        r = {}
        for mon in ("none", "single", "dual"):
            log = _run(name, fold, seq, sc, pred, seed, monitor=mon, cusum_h=H.get(mon, 1.0))
            eh = np.linalg.norm(log.p_est[:, :2] - log.p_true[:, :2], axis=1)
            m = log.t >= SPOOF_START
            r[mon] = {"alarm_time": log.alarm_time, "max_err_after": float(eh[m].max()), "end_err": float(eh[-1])}
        res["spoof"][str(rate)] = r
    print(f"{name} {fold} seed {seed} done", flush=True)
    return f"{name}/{fold}/{seed}", res


def main():
    runs = [(n, f, s) for n in I.IMUS for f in "AB" for s in SEEDS]
    with Pool(8) as pool:
        cal = pool.map(calib, runs)
        H = {m: 1.5 * max(c[m] for c in cal) for m in ("single", "dual")}
        print("calibrated h:", H, flush=True)
        out = dict(pool.map(test, [(n, f, s, H) for n, f, s in runs]))
    keys = list(out)
    summ = {"h": H, "calib_max_g": cal, "false_alarms_clean": {}, "by_rate": {}}
    for mon in ("single", "dual"):
        summ["false_alarms_clean"][mon] = int(sum(out[k]["clean"][mon] is not None for k in keys))
    lines = ["| 漂移速率[m/s] | 方法 | 检出率 | 平均检测延迟[s] | 欺骗前误报 | 欺骗后最大水平误差[m] |",
             "|---|---|---|---|---|---|"]
    for rate in RATES:
        summ["by_rate"][str(rate)] = {}
        for mon in ("none", "single", "dual"):
            rs = [out[k]["spoof"][str(rate)][mon] for k in keys]
            at = [r["alarm_time"] for r in rs]
            early = sum(t is not None and t < SPOOF_START for t in at)
            det = [t - SPOOF_START for t in at if t is not None and t >= SPOOF_START]
            me = np.array([r["max_err_after"] for r in rs])
            d = {"detection_rate": len(det) / len(rs), "mean_delay_s": float(np.mean(det)) if det else None,
                 "pre_spoof_alarms": early, "max_err_after_mean": float(me.mean()), "max_err_after_std": float(me.std())}
            summ["by_rate"][str(rate)][mon] = d
            nm = {"none": "无监测", "single": "单通道 NIS-CUSUM", "dual": "双通道 DR-CUSUM(本文)"}[mon]
            lines.append(f"| {rate} | {nm} | " + ("-" if mon == "none" else f"{d['detection_rate']:.0%}") + " | "
                         + (f"{d['mean_delay_s']:.1f}" if d["mean_delay_s"] is not None else "-")
                         + f" | {early if mon != 'none' else '-'} | {d['max_err_after_mean']:.2f}±{d['max_err_after_std']:.2f} |")
    fa = summ["false_alarms_clean"]
    md = "\n".join(lines) + (f"\n\n真实 RTK 干净测试半段误报({len(keys)} 条, 各约 800 s): 单通道 {fa['single']}, 双通道 {fa['dual']}; "
                             f"阈值 h(训练半段干净运行 1.5×最大值): 单通道 {H['single']:.1f}, 双通道 {H['dual']:.1f}\n")
    print(md)
    (OUT / "spoof_table.md").write_text(md)
    (OUT / "spoof_metrics.json").write_text(json.dumps({"summary": summ, "per_run": out}, indent=1, ensure_ascii=False))


if __name__ == "__main__":
    main()
