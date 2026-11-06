"""Paired real i2Nav evaluation; no reference-derived pseudo measurements."""
import argparse
import hashlib
import json
from multiprocessing import Pool
from pathlib import Path

import numpy as np

from . import i2nav_scenarios as I
from .metrics import summarize
from .pipeline import Config, run
from .zupt import AdaptiveZeroVelocity


def job(args):
    name, fold, kind, case, profile = args
    train, test = I.fold_ranges(fold)
    cal_seq = I.segment(name, *train)
    detector = AdaptiveZeroVelocity.calibrate(cal_seq.truth.gyro, cal_seq.truth.accel,
                                              I.filter_spec(name), dt=cal_seq.truth.dt,
                                              accel_std_cap=.25 if profile=='conservative' else 3.,
                                              gyro_std_cap=.01 if profile=='conservative' else .05)
    # Previously published calibration uses only training-half RTK/INS, no truth.
    mounts = json.loads(Path('results/i2nav/mount_calibration.json').read_text())
    mount = mounts[f'{name}/{fold}']
    seq = I.segment(name, *test)
    outages = [] if case == 'clean' else I.outage_schedule(seq.truth.t[-1], first=60.)
    sc = I.make_scenario(seq, 0, outages)
    rows = {}
    for enabled in (False, True):
        cfg = Config(filter=kind, pseudo='nhc', init_seed=0, **I.REAL_CFG,
                     **I.earth_config(seq), mount_py=tuple(mount['mount_py']),
                     nhc_sigma=mount['nhc_sigma_eff'], zupt=enabled,
                     zupt_require_gnss_stop=profile=='guarded',
                     zupt_accel_var=detector.accel_var, zupt_gyro_var=detector.gyro_var)
        log = run(sc.truth, sc.imu, sc.gnss, sc.imu_spec, sc.gnss_spec, cfg)
        metrics = summarize(log, outages, t_start=30.)
        metrics.pop('runtime_us')
        err = np.linalg.norm(log.p_est[:, :2]-log.p_true[:, :2], axis=1)
        metrics['outages'] = [dict(start=s, duration=d,
            max_h=float(err[(log.t >= s) & (log.t < s+d)].max()),
            end_h=float(err[(log.t >= s) & (log.t < s+d)][-1])) for s, d in outages]
        motion = log.extra['motion']
        tt = np.array(motion['zupt_times'])
        speeds = np.linalg.norm(seq.truth.v, axis=1)
        ii = np.rint(tt/seq.truth.dt).astype(int)
        metrics['motion'] = motion
        # Reference used exclusively for scoring, never detector/calibration inputs.
        metrics['accepted_while_reference_speed_gt_0_5'] = int((speeds[ii] > .5).sum())
        metrics['reference_stationary_s'] = float((speeds < .1).sum()*seq.truth.dt)
        rows['adaptive' if enabled else 'baseline'] = metrics
    result = dict(imu=name, fold=fold, filter=kind, case=case, seed=0, profile=profile,
                  duration_s=float(seq.truth.t[-1]), train_range=train, test_range=test,
                  thresholds=dict(accel_var=detector.accel_var, gyro_var=detector.gyro_var),
                  mount=mount, methods=rows)
    print(f'{name} {fold} {kind} {case}: ' + ' -> '.join(f"{r['rmse_h']:.3f}" for r in rows.values()), flush=True)
    return result


def aggregate(rows):
    return {method: {key: float(np.mean([r['methods'][method][key] for r in rows]))
                     for key in ('rmse_h', 'coverage95_h', 'anees_pos')}
            for method in ('baseline', 'adaptive')}


def write_report(rows, out):
    out = Path(out); out.mkdir(parents=True, exist_ok=True)
    hashes = {n: hashlib.sha256((I.DATA/f'{n}.npz').read_bytes()).hexdigest() for n in I.IMUS}
    result = dict(protocol='4 IMUs x 2 disjoint time folds x 2 filters x clean/outage x 2 methods; seed 0',
                  data_sha256=hashes, mounting_calibration_sha256=hashlib.sha256(
                      Path('results/i2nav/mount_calibration.json').read_bytes()).hexdigest(),
                  profile=rows[0]['profile'], aggregate=aggregate(rows), runs=rows)
    (out/'metrics.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    lines = ['# Adaptive ZUPT on real i2Nav IMU + RTK', '',
        f"{len(rows)} paired cases / {2*len(rows)} trajectory runs. All cases retained. Profile: {rows[0]['profile']}.",
        'Vehicle/guarded caps: accel std 3 m/s², gyro std 0.05 rad/s. Conservative profile: 0.25 m/s² and 0.01 rad/s.',
        'Guarded profile (default): two consecutive accepted RTK displacement upper-speed bounds <0.25 m/s (distance + 2 sigma, epochs <=2 s apart) confirm rest. Any IMU dwell reset or moving RTK fix (even filter-rejected) revokes confirmation. New stops during outages cannot be confirmed. Other profiles omit this independent stop guard.',
        'Baseline: ESKF/InEKF + mounting-calibrated NHC with correlation-inflated sigma; Earth compensation and real receiver epoch sigma (0.05 m floor).',
        'Same seed 0, IMU, RTK, geometry, initialization, and outages within each pair. First 30 s excluded from global metrics.',
        'Calibration: lower-decile one-second IMU variance on the opposite time half, 4x margin, sensor noise density floor and physical caps. Existing training-half RTK/INS mounting calibration reused.',
        'Causal 0.5 s trailing window, 1 s dwell; speed <0.5 m/s, raw gyro mean <0.02 rad/s and gravity magnitude within 0.25 m/s². ZUPT sigma 0.05 m/s at 10 Hz, chi-square 0.999 gate; never forced.',
        'IMU rates indexed k cover [t[k], t[k+1]]; detector sees only propagated samples. No test truth or future samples used by detection.',
        'Clean and fixed outage schedule (first=60 s, 30/120/60/30/120/60 s, 40 s gaps) run on both test halves (~803 s each).', '',
        '| IMU | Fold | Filter | Case | RMSE baseline → adaptive (m) | Coverage baseline → adaptive | ANEES baseline → adaptive | ZUPT accepted | Speed >0.5 m/s |',
        '|---|---|---|---|---:|---:|---:|---:|---:|']
    for r in rows:
        b, a = (r['methods'][m] for m in ('baseline', 'adaptive'))
        lines.append(f"| {r['imu']} | {r['fold']} | {r['filter']} | {r['case']} | {b['rmse_h']:.4f} → {a['rmse_h']:.4f} | {b['coverage95_h']:.3f} → {a['coverage95_h']:.3f} | {b['anees_pos']:.3f} → {a['anees_pos']:.3f} | {a['motion']['zupt_accepted']} | {a['accepted_while_reference_speed_gt_0_5']} |")
    lines += ['', '## Paired outage maxima (all events, including regressions)', '',
              '| IMU | Duration | n | Mean maximum baseline → adaptive (m) | Mean endpoint baseline → adaptive (m) | Improved / tied / regressed |',
              '|---|---:|---:|---:|---:|---:|']
    for name in [*I.IMUS, 'ALL']:
        sub = [r for r in rows if r['case']=='outage' and (name=='ALL' or r['imu']==name)]
        for d in (30, 60, 120):
            pairs = [(b, a) for r in sub for b, a in zip(r['methods']['baseline']['outages'], r['methods']['adaptive']['outages']) if b['duration']==d]
            bm, am = [x['max_h'] for x,y in pairs], [y['max_h'] for x,y in pairs]
            be, ae = [x['end_h'] for x,y in pairs], [y['end_h'] for x,y in pairs]
            diff = np.array(am)-bm
            lines.append(f"| {name} | {d}s | {len(pairs)} | {np.mean(bm):.4f} → {np.mean(am):.4f} | {np.mean(be):.4f} → {np.mean(ae):.4f} | {(diff < -1e-6).sum()} / {(abs(diff)<=1e-6).sum()} / {(diff > 1e-6).sum()} |")
    diffs = np.array([r['methods']['adaptive']['rmse_h']-r['methods']['baseline']['rmse_h'] for r in rows])
    accepted = sum(r['methods']['adaptive']['motion']['zupt_accepted'] for r in rows)
    false = sum(r['methods']['adaptive']['accepted_while_reference_speed_gt_0_5'] for r in rows)
    lines += ['', f'Global RMSE improved / tied / regressed: {(diffs < -1e-6).sum()} / {(abs(diffs)<=1e-6).sum()} / {(diffs > 1e-6).sum()}. Accepted ZUPTs: {accepted}; accepted at reference speed >0.5 m/s: {false}.', '',
        '## Limits', '',
        'This is one shared drive, four co-mounted IMUs, two correlated time folds and one initial seed. Event counts are descriptive; no independence or statistical significance claim.',
        'The filter-speed veto can suppress real stops after large outage drift, and can admit smooth motion if the filter is already wrong. Zero observed false-motion accepts is not a safety guarantee; stops below 0.5 m/s include creep. Raw gyro mean conservatively rejects sensors with large turn-on bias.',
        'The vehicle caps were revised after the conservative pilot rejected the calibration-half vibration floor (0.40–0.72 m/s² accel std). This is exploratory development, not a blind confirmatory evaluation. The pilot is retained in results/adaptive_motion_conservative. The unguarded vehicle profile is retained in results/adaptive_motion_vehicle; its large regressions motivated the guarded default, so this evaluation must not be treated as held-out validation. Calibration is frozen after the training half; caps can reject highly vibrating stationary vehicles. No online bias/threshold learning from accepted ZUPTs, avoiding self-reinforcing detection.',
        'Odometry is integrated in the same pipeline with explicit mounting rotation and IMU-to-wheel lever arm, Earth-rate correction, timestamp validation, validity mask and NIS gating. i2Nav has no wheel-speed stream: no odometry accuracy claim or synthetic wheel experiment is reported. Slip, scale, timing offsets and geometry uncertainty remain unmodelled; caller must calibrate geometry and exclude slipping epochs.',
        'Reference is used for initial-state construction (existing protocol) and scoring only; this is not unaided cold-start navigation.', '',
        f"Reproduce: `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -m ccgins.motion_experiment --workers 4 --profile {rows[0]['profile']} --out {out}`", '']
    (out/'report.md').write_text('\n'.join(lines))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--workers', type=int, default=4)
    ap.add_argument('--out', default='results/adaptive_motion')
    ap.add_argument('--profile', choices=('guarded', 'vehicle', 'conservative'), default='guarded')
    a = ap.parse_args()
    jobs = [(n, f, k, c, a.profile) for n in I.IMUS for f in 'AB' for k in ('eskf', 'inekf') for c in ('clean', 'outage')]
    with Pool(a.workers) as pool:
        rows = pool.map(job, jobs)
    write_report(rows, a.out)


if __name__ == '__main__':
    main()
