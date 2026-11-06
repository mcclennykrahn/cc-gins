"""Replay frozen published configurations, without changing model/scoring defaults.

Protocol: all 64 guarded motion paths (4 IMUs x 2 folds x 2 filters x
clean/outage x baseline/ZUPT), plus all 24 NO-NHC stream_motion paths.
Compare every 10th IMU epoch, all position/velocity components and all 225
covariance entries, atol=1e-12, rtol=0. No fitting or case exclusion.
Synthetic wheel remains reference-derived, NOT real odometry validation.
"""
import argparse
import hashlib
import json
from multiprocessing import Pool
from pathlib import Path

import numpy as np
from . import i2nav_scenarios as I
from .metrics import summarize
from .parity import replay
from .pipeline import Config, run, mount_matrix
from .zupt import OdometryData

SOURCES = ('results/adaptive_motion/metrics.json', 'results/stream_motion/metrics.json')


def job(args):
    path, r, method = args
    seq = I.segment(r['imu'], *r['test_range'])
    outages = [] if r.get('case') == 'clean' else I.outage_schedule(seq.truth.t[-1], first=60.)
    sc = I.make_scenario(seq, 0, outages)
    motion = 'adaptive_motion' in path
    det = r['thresholds'] if motion else r['calibration']
    cfg = Config(filter=r['filter'] if motion else 'inekf',
                 pseudo='nhc' if motion else 'none', monitor='none',
                 zupt=method != 'baseline', **I.REAL_CFG, **I.earth_config(seq),
                 zupt_accel_var=det['accel_var'], zupt_gyro_var=det['gyro_var'])
    odo = None
    if motion:
        cfg.mount_py = tuple(r['mount']['mount_py'])
        cfg.nhc_sigma = r['mount']['nhc_sigma_eff']
    elif method != 'guarded':
        mount = json.loads(Path('results/i2nav/mount_calibration.json').read_text())[f"{r['imu']}/{r['fold']}"]
        C = mount_matrix(*mount['mount_py'])
        ii = np.arange(10, len(seq.truth.t), 10)
        vb = np.einsum('nji,nj->ni', seq.truth.R[ii], seq.truth.v[ii])
        forward = (vb @ C.T)[:, 0]
        tt = seq.truth.t[ii]
        sigma = .01 + .02*np.abs(forward)
        slipping = (tt % 90 >= 40) & (tt % 90 < 45)
        speed = 1.01*forward + np.random.default_rng(2026).normal(size=len(ii))*sigma + slipping*.1*forward
        odo = OdometryData(tt, speed, sigma, C, np.zeros(3), ~slipping)
        cfg.zupt_odometry_stop = method == 'synthetic_wheel_stop'
    log = run(sc.truth, sc.imu, sc.gnss, sc.imu_spec, sc.gnss_spec, cfg,
              odometry=odo, log_covariance=True)
    parity = replay(sc, cfg, log, odo)
    # Batch behavior must stay identical to the previously published metrics.
    scores = summarize(log, outages, t_start=30.)
    score_diff = {k: float(scores[k]-r['methods'][method][k])
                  for k in ('rmse_h', 'coverage95_h', 'anees_pos')}
    assert all(abs(d) < 1e-12 for d in score_diff.values()), score_diff
    result = dict(source=path, imu=r['imu'], fold=r['fold'], filter=cfg.filter,
                  case=r.get('case', 'outage'), method=method,
                  config=vars(cfg), stream_batch=parity, published_batch_score_diff=score_diff)
    print(r['imu'], r['fold'], cfg.filter, result['case'], method, parity, flush=True)
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--workers', type=int, default=4)
    ap.add_argument('--out', default='results/stream_batch_parity')
    a = ap.parse_args()
    jobs = [(path, r, m) for path in SOURCES
            for r in json.loads(Path(path).read_text())['runs'] for m in r['methods']]
    with Pool(a.workers) as pool:
        rows = pool.map(job, jobs)
    maxima = {k: max(r['stream_batch'][k] for r in rows)
              for k in rows[0]['stream_batch'] if k.endswith('_max_abs')}
    hashes = {p: hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in SOURCES}
    hashes.update({str(I.DATA/f'{n}.npz'): hashlib.sha256((I.DATA/f'{n}.npz').read_bytes()).hexdigest() for n in I.IMUS})
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    (out/'metrics.json').write_text(json.dumps(dict(protocol=__doc__, input_sha256=hashes,
        maxima=maxima, runs=rows), indent=2, allow_nan=False)+'\n')
    lines = ['# Stream / batch consistency on frozen published configurations', '',
        __doc__, '',
        f'{len(rows)} complete batch + stream pairs; all cases retained. Every logged epoch checked. All motion counters/times equal. Published batch RMSE, coverage and ANEES unchanged to 1e-12.', '',
        '| Maximum absolute discrepancy | Value |', '|---|---:|']
    lines += [f'| {k} | {v:.16g} |' for k, v in maxima.items()]
    lines += ['', '## Diagnosis and scope', '',
        'Streaming previously silently omitted fixed mounted NHC, making motion-report configurations inequivalent. It now applies the same fixed mounted NHC after ZUPT/wheel and before GNSS, with the same WINDOW warmup and pseudo cadence. Unsupported learned/ACI/monitor/adaptive-R/range configurations raise an error.',
        'Batch propagates with nominal truth.dt. Timestamp subtraction in streaming introduced tiny interval errors on real resampled grids; these can accumulate during outages. from_truth now uses nominal dt for whole fixed-grid intervals within 1e-9 s; off-grid GNSS subintervals still use their actual durations. Batch estimator behavior and default stop confirmation are unchanged.',
        'Full covariance uses each filter’s internal error coordinates (ESKF and InEKF differ); compared only within the same filter. Velocity/position diagonal and cross blocks are included, together with nav-frame position covariance.',
        'Intentional differences: streaming applies start-epoch GNSS while batch starts at the next IMU epoch. Replay explicitly skips valid index-0 GNSS, counts recorded per case. Off-grid GNSS splits propagation in streaming; batch uses indices. Replay checks alignment and never snaps timestamps. Wheel data must be queued before its propagated grid epoch; late GNSS cannot be reconstructed. from_truth supplies fixed dt; a manually constructed navigator without dt uses actual timestamp differences.', '',
        '## Scientific limits', '',
        'This is numerical equivalence, not covariance calibration. The NO-NHC synthetic-wheel coverage/ANEES regressions remain: reference-derived speed is an oracle input, not measured odometry accuracy. Previously isolated wheel-stop RMSE 6.4053 → 6.4273 m was worse. No new independent-drive or real-wheel evidence is supplied, and no stop-assist generalization claim is made. Existing adaptive_motion NHC baselines and stream_motion reports are retained.', '',
        'Reproduce: `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -m ccgins.parity_experiment --workers 4`', '']
    (out/'report.md').write_text('\n'.join(lines))


if __name__ == '__main__':
    main()
