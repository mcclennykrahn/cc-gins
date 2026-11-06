"""Exploratory SYNTHETIC reference-derived wheel speed, NOT real wheel validation."""
import argparse
import hashlib
import json
from dataclasses import replace
from multiprocessing import Pool
from pathlib import Path
import numpy as np
from . import i2nav_scenarios as I
from .pipeline import Config, run, mount_matrix
from .zupt import AdaptiveZeroVelocity, OdometryData
from .metrics import summarize
from .stream import StreamingNavigator


def job(args):
    name, fold = args
    train, test = I.fold_ranges(fold)
    cal = I.segment(name, *train)
    det = AdaptiveZeroVelocity.calibrate(cal.truth.gyro, cal.truth.accel, I.filter_spec(name), dt=cal.truth.dt)
    seq = I.segment(name, *test)
    outs = I.outage_schedule(seq.truth.t[-1], first=60.)
    sc = I.make_scenario(seq, 0, outs)
    mount = json.loads(Path('results/i2nav/mount_calibration.json').read_text())[f'{name}/{fold}']
    C = mount_matrix(*mount['mount_py'])
    ii = np.arange(10, len(seq.truth.t), 10)
    # Synthetic wheel at IMU centre: no invented physical wheel lever geometry.
    vb = np.einsum('nji,nj->ni', seq.truth.R[ii], seq.truth.v[ii])
    forward = (vb @ C.T)[:, 0]
    tt = seq.truth.t[ii]
    rng = np.random.default_rng(2026)
    sigma = .01 + .02*np.abs(forward)
    slipping = (tt % 90 >= 40) & (tt % 90 < 45)
    speed = 1.01*forward + rng.normal(size=len(ii))*sigma + slipping*.1*forward
    odo = OdometryData(tt, speed, sigma, C, np.zeros(3), ~slipping)
    cfg = Config(filter='inekf', pseudo='none', monitor='none', zupt=True,
                 **I.REAL_CFG, **I.earth_config(seq),
                 zupt_accel_var=det.accel_var, zupt_gyro_var=det.gyro_var)
    rows = {}
    logs = {}
    for method in ('guarded', 'synthetic_wheel', 'synthetic_wheel_stop'):
        c = replace(cfg, zupt_odometry_stop=method=='synthetic_wheel_stop')
        log = run(sc.truth, sc.imu, sc.gnss, sc.imu_spec, sc.gnss_spec, c,
                  odometry=None if method=='guarded' else odo)
        logs[method] = log
        m = summarize(log, outs, t_start=30.)
        m.pop('runtime_us')
        eh = np.linalg.norm(log.p_est[:, :2]-log.p_true[:, :2], axis=1)
        m['outages'] = [dict(start=s, duration=d, max_h=float(eh[(log.t>=s)&(log.t<s+d)].max()),
                            end_h=float(eh[(log.t>=s)&(log.t<s+d)][-1])) for s,d in outs]
        m['motion'] = log.extra['motion']
        zt = np.array(m['motion']['zupt_times'])
        zidx = np.rint(zt/seq.truth.dt).astype(int)
        m['zupt_reference_speed_gt_0_5'] = int((np.linalg.norm(seq.truth.v[zidx], axis=1)>.5).sum())
        m['outage_zupts'] = int(sum(((zt>=s)&(zt<s+d)).sum() for s,d in outs))
        rows[method] = m
    # Replay every epoch with the same sigma floors and ordering as batch.
    nav = StreamingNavigator.from_truth(sc.truth, sc.imu_spec, sc.gnss_spec, replace(cfg, zupt_odometry_stop=True))
    for j,t in enumerate(sc.gnss.t):
        if sc.gnss.valid[j] and sc.gnss.idx[j] > 0:
            nav.push_gnss(t, sc.gnss.pos[j], np.maximum(sc.gnss.sigma_ep[j], cfg.gnss_sigma_floor)**2)
    for j,t in enumerate(odo.t):
        nav.push_odometry(t, odo.speed[j], odo.sigma[j], C, odo.lever_b, odo.valid[j])
    positions=[]; velocities=[]; covs=[]
    # Batch logs only propagated epochs at multiples of ten.
    wanted = set(np.rint(logs['synthetic_wheel_stop'].t/seq.truth.dt).astype(int))
    for k,t in enumerate(seq.truth.t):
        nav.push_imu(t, sc.imu.accel[k], sc.imu.gyro[k])
        if k in wanted:
            state = nav.get_state()
            positions.append(state.position); velocities.append(state.velocity); covs.append(state.position_covariance)
    log = logs['synthetic_wheel_stop']
    parity = dict(position_max_abs=float(np.max(np.abs(positions-log.p_est))),
                  velocity_max_abs=float(np.max(np.abs(velocities-log.v_est))),
                  position_cov_max_abs=float(np.max(np.abs(covs-log.P_pos))),
                  motion_equal=nav.motion==log.extra['motion'])
    print(name, fold, {k:round(v['rmse_h'],3) for k,v in rows.items()}, parity, flush=True)
    return dict(imu=name, fold=fold, train_range=train, test_range=test,
                calibration=dict(accel_var=det.accel_var, gyro_var=det.gyro_var),
                methods=rows, stream_batch=parity)


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--workers', type=int, default=4)
    ap.add_argument('--out', default='results/stream_motion')
    ap.add_argument('--report-only', action='store_true'); a=ap.parse_args()
    if a.report_only:
        write_report(json.loads((Path(a.out)/'metrics.json').read_text())['runs'], a.out)
        return
    with Pool(a.workers) as pool:
        rows=pool.map(job, [(n,f) for n in I.IMUS for f in 'AB'])
    write_report(rows, a.out)


def write_report(rows, out):
    out=Path(out); out.mkdir(parents=True, exist_ok=True)
    for r in rows:
        parity = r['stream_batch']
        assert parity['motion_equal'] and all(parity[k] < 1e-5 for k in
            ('position_max_abs', 'velocity_max_abs', 'position_cov_max_abs')), parity
        seq = I.segment(r['imu'], *r['test_range'])
        stopped = np.linalg.norm(seq.truth.v, axis=1) < .1
        edges = np.diff(np.r_[False, stopped, False].astype(int))
        events = []
        for begin, end in zip(np.flatnonzero(edges==1), np.flatnonzero(edges==-1)):
            t0 = float(seq.truth.t[begin]); t1 = float(seq.truth.t[end-1])
            if t1-t0 < 1.: continue
            outage = next((o for o in r['methods']['guarded']['outages'] if o['start'] < t0 < o['start']+o['duration']), None)
            if outage is None: continue
            last = min(t1, outage['start']+outage['duration'])
            events.append(dict(start=t0, end=t1, outage_start=outage['start'],
                accepted_during_stop={m: int(((np.array(v['motion']['zupt_times'])>=t0)&
                                             (np.array(v['motion']['zupt_times'])<last)).sum())
                                      for m,v in r['methods'].items()}))
        r['reference_new_outage_stops'] = events
    agg={m:{k:float(np.mean([r['methods'][m][k] for r in rows])) for k in ('rmse_h','coverage95_h','anees_pos','outage_zupts')} for m in rows[0]['methods']}
    data=dict(protocol='SYNTHETIC wheel: 4 co-mounted IMUs x 2 time folds; InEKF seed 0; 3 batch methods + 1 stream replay per case',
              mount_calibration_sha256=hashlib.sha256(Path('results/i2nav/mount_calibration.json').read_bytes()).hexdigest(),
              data_sha256={n:hashlib.sha256((I.DATA/f'{n}.npz').read_bytes()).hexdigest() for n in I.IMUS}, aggregate=agg, runs=rows)
    (out/'metrics.json').write_text(json.dumps(data, indent=2, allow_nan=False)+'\n')
    lines=['# SYNTHETIC wheel speed: exploratory outage-stop experiment','',
        'i2Nav has NO real wheel-speed log. Reference/body velocity is intentionally used as a synthetic measurement here; this is an oracle-assisted mechanism study, not measured odometry accuracy or held-out validation.',
        '8 paired cases, 24 batch trajectories + 8 full stream replays. Four co-mounted IMUs, folds A/B, InEKF, initial seed 0. All cases retained. Same drive/folds as prior development; no significance claim.',
        'No NHC/learned pseudo-measurements: streaming supports IMU/GNSS/motion only. Guarded baseline is therefore a new matched baseline, not the earlier NHC baseline.',
        'Frozen opposite-half IMU variance calibration; existing opposite-half mounting calibration. Synthetic wheel point is the IMU centre (explicit zero lever), vehicle forward axis from calibrated mounting. 10 Hz signed reference/body forward speed, 1% scale error, independent Gaussian sigma=0.01+0.02|v_forward| m/s; +10% slip for seconds 40–45 of each 90 s period, explicitly marked invalid and excluded. Synthetic RNG seed 2026. Assumed known slip is optimistic; no slip estimator.',
        'Methods: guarded ZUPT; guarded ZUPT + synthetic wheel; guarded ZUPT + synthetic wheel + opt-in wheel-zero stop confirmation. The latter two isolate the incremental benefit of stop confirmation from continuous speed aiding.',
        'Wheel stop guard requires two accepted |speed|+3sigma<0.1 m/s observations <=0.2 s apart; expires after 0.2 s. ZUPT still requires causal IMU quiet/dwell, gravity/rotation and filter-speed veto, and is never forced. Invalid observations are not updates; previous evidence expires naturally. Forward zero does not exclude lateral slip.',
        'Fixed six-outage schedule first=60 s, durations 30/120/60/30/120/60 s with 40 s gaps. Real RTK receiver sigma with 0.05 m floor; Earth compensation. Global scoring excludes first 30 s. Reference also initializes the existing protocol and scores trajectories.', '',
        '| IMU/fold | Guarded RMSE | + synthetic wheel | + wheel stop | Outage ZUPTs (baseline / wheel / stop) |', '|---|---:|---:|---:|---:|']
    for r in rows:
        ms=list(r['methods'].values())
        lines.append(f"| {r['imu']}/{r['fold']} | "+' | '.join(f"{m['rmse_h']:.4f}" for m in ms)+' | '+' / '.join(str(m['outage_zupts']) for m in ms)+' |')
    lines+=['','Aggregate (arithmetic mean across cases):', '', '| Method | RMSE m | Coverage95 | ANEES/3 |', '|---|---:|---:|---:|']
    for m,v in agg.items(): lines.append(f"| {m} | {v['rmse_h']:.4f} | {v['coverage95_h']:.4f} | {v['anees_pos']:.4f} |")
    lines+=['','## All paired outage events, including regressions','','| Comparison | n | Mean max before → after m | Mean end before → after m | Max improved / tied / regressed |','|---|---:|---:|---:|---:|']
    for before,after in [('guarded','synthetic_wheel_stop'),('synthetic_wheel','synthetic_wheel_stop')]:
        pairs=[(b,c) for r in rows for b,c in zip(r['methods'][before]['outages'],r['methods'][after]['outages'])]
        bm=np.array([b['max_h'] for b,c in pairs]); am=np.array([c['max_h'] for b,c in pairs]); d=am-bm
        lines.append(f'| {before} → {after} | {len(pairs)} | {bm.mean():.4f} → {am.mean():.4f} | {np.mean([b["end_h"] for b,c in pairs]):.4f} → {np.mean([c["end_h"] for b,c in pairs]):.4f} | {(d< -1e-6).sum()} / {(abs(d)<=1e-6).sum()} / {(d>1e-6).sum()} |')
    for before,after in [('guarded','synthetic_wheel_stop'),('synthetic_wheel','synthetic_wheel_stop')]:
        diff=np.array([r['methods'][after]['rmse_h']-r['methods'][before]['rmse_h'] for r in rows])
        lines += ['', f"Whole-run RMSE {before} → {after}: improved / tied / regressed = {(diff < -1e-6).sum()} / {(abs(diff)<=1e-6).sum()} / {(diff > 1e-6).sum()}."]
    lines += ['', 'Accepted ZUPTs total / inside outages: ' + ', '.join(
        f"{m}={sum(r['methods'][m]['motion']['zupt_accepted'] for r in rows)} / {sum(r['methods'][m]['outage_zupts'] for r in rows)}" for m in agg) + '.']
    events=[e for r in rows for e in r['reference_new_outage_stops']]
    lines += ['', f"Reference-defined new outage stops (speed norm <0.1 m/s for >=1 s, onset strictly inside outage): {len(events)} co-mounted sensor/fold episodes (correlated; three physical stop onsets viewed by four sensors). Counts with at least one accepted ZUPT before stop/outage ends: " + ', '.join(f"{m}={sum(e['accepted_during_stop'][m]>0 for e in events)}" for m in agg) + '.',
              'Accepted ZUPTs at reference speed >0.5 m/s: ' + ', '.join(f"{m}={sum(r['methods'][m]['zupt_reference_speed_gt_0_5'] for r in rows)}" for m in agg) + '. Creep and near-zero motion are not excluded by this diagnostic.']
    lines+=['',f'Stream/batch maximum absolute position discrepancy: {max(r["stream_batch"]["position_max_abs"] for r in rows):.3g} m. Full per-case position/velocity/covariance discrepancies and motion counts are in metrics.json.',
        '', 'Limits: synthetic speed directly imports reference information. No real wheel accuracy, slip robustness, timing/geometry uncertainty or independent-drive generalization is measured. Large outage drift can keep the filter-speed veto active and prevent real stops. Raw gyro bias can also veto rest. Continuous wheel aiding and stop confirmation effects must be read separately. No tuning on these outcomes.', '',
        'Reproduce: `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -m ccgins.stream_motion_experiment --workers 4`','']
    (out/'report.md').write_text('\n'.join(lines))

if __name__=='__main__': main()
