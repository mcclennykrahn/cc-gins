"""Sensor-only GNSS course feasibility; never substitutes course for body attitude.

A fixed trailing least-squares antenna velocity tests whether GNSS course alone
can support the existing truth-relative yaw prior. No filter updates are changed.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from ccgins.cli import _build_inputs, load_config
from ccgins.timing import at_or_after


EPOCHS = [5., 10., 20., 30., 59., 60., 65., 89., 90., 95., 119.,
          120., 125., 179., 180.]
TURN_LIMIT_DEG = 3.


def causal_course(t, position, valid, now, window=5., min_fixes=4,
                  min_speed=3., max_age=1.01):
    """Return a causal antenna-course candidate or an explicit refusal.

    Assumes forward motion if interpreted as body heading. Speed gating cannot
    verify that assumption or reject turning. This is an audit diagnostic, not
    a validated alignment acceptance gate. Inputs are ENU fixes in seconds/metres.
    """
    t, position, valid = np.asarray(t), np.asarray(position), np.asarray(valid)
    if t.ndim != 1 or position.shape != (len(t), 3) or valid.shape != t.shape:
        raise ValueError('expected t[N], position[N,3], valid[N]')
    if not np.isfinite([now, window, min_speed, max_age]).all() or window <= 0 or min_speed <= 0 or max_age < 0 or min_fixes < 2:
        raise ValueError('invalid course protocol')
    if not np.isfinite(t).all() or np.any(np.diff(t) <= 0):
        raise ValueError('fix times must be finite and strictly increasing')
    mask = valid.astype(bool) & at_or_after(t, now-window) & (t <= now)
    ts, ps = t[mask], position[mask]
    row = dict(n_fixes=len(ts), status='insufficient_fixes', course_rad=None,
               first_fix_s=float(ts[0]) if len(ts) else None,
               speed_mps=None, velocity_enu=None, last_fix_age_s=None)
    if not len(ts):
        return row
    row['last_fix_age_s'] = float(max(0., now-ts[-1]))
    if not at_or_after(max_age, now-ts[-1]):
        row['status'] = 'stale'
        return row
    if len(ts) < min_fixes:
        return row
    if not np.isfinite(ps).all():
        row['status'] = 'invalid_position'
        return row
    tc = ts-ts.mean()
    velocity = tc @ (ps-ps.mean(axis=0)) / (tc @ tc)
    speed = float(np.linalg.norm(velocity[:2]))
    row.update(speed_mps=speed, velocity_enu=velocity.tolist())
    if speed < min_speed:
        row['status'] = 'low_speed'
        return row
    row.update(status='candidate', course_rad=float(np.arctan2(velocity[1], velocity[0])))
    return row


def wrap_angle(x):
    return float(np.arctan2(np.sin(x), np.cos(x)))


def measured_rotation(imu, first_fix_s, now):
    """Integrated body rotation magnitude over complete, causal IMU steps."""
    first = np.searchsorted(imu.t, first_fix_s)
    end = np.searchsorted(imu.t, now, side='right') - 1
    return float(np.degrees(imu.dt * np.linalg.norm(imu.gyro[first:end], axis=1).sum()))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--config', default='configs/example_sim.yaml')
    ap.add_argument('--seeds', type=int, nargs='+', default=[900, 103001, 103002])
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    raw = load_config(args.config)
    result = dict(candidate_id='scheme_causal_course', production_changed=False,
                  scope='Antenna course candidate, not full roll/pitch/yaw alignment; no tail validation.',
                  config=raw, window_s=5., min_speed_mps=3., min_fixes=4,
                  max_age_s=1.01, integrated_gyro_limit_deg=TURN_LIMIT_DEG, rows=[])
    for seed in args.seeds:
        truth, imu, gnss, _, gs = _build_inputs(raw, seed_override=seed)
        antenna = truth.p[gnss.idx] + np.einsum('nij,j->ni', truth.R[gnss.idx], gs.lever_arm)
        rows = []
        for now in EPOCHS:
            if now > truth.t[-1]:
                continue
            observed = causal_course(gnss.t, gnss.pos, gnss.valid, now)
            rotation_deg = None
            gated_status = observed['status']
            if gated_status == 'candidate':
                rotation_deg = measured_rotation(imu, observed['first_fix_s'], now)
                if rotation_deg > TURN_LIMIT_DEG:
                    gated_status = 'turning'
            ideal = causal_course(gnss.t, antenna, gnss.valid, now)
            k = np.searchsorted(truth.t, now, side='right') - 1
            # Truth is read only AFTER candidate/refusal, solely for evaluation.
            yaw = float(np.arctan2(truth.R[k, 1, 0], truth.R[k, 0, 0]))
            row = dict(time_s=now, truth_epoch_s=float(truth.t[k]),
                       truth_reference='latest IMU epoch at or before request, evaluation only',
                       observed=observed, noiseless_antenna=ideal,
                       integrated_gyro_deg=rotation_deg, gyro_gated_status=gated_status,
                       truth_body_yaw_rad=yaw, truth_forward_speed_mps=float(truth.v_body[k, 0]),
                       course_error_deg=(np.degrees(wrap_angle(observed['course_rad']-yaw))
                                         if observed['course_rad'] is not None else None),
                       noiseless_course_error_deg=(np.degrees(wrap_angle(ideal['course_rad']-yaw))
                                                   if ideal['course_rad'] is not None else None))
            rows.append(row)
        result['rows'].append(dict(seed=seed, epochs=rows))
        (args.out/'course.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
        print(seed, [(r['time_s'], r['observed']['status'], r['course_error_deg']) for r in rows], flush=True)


if __name__ == '__main__':
    main()
