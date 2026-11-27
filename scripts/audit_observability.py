"""Offline local bias observability for the example GNSS/NHC schedule."""
import argparse
import json
from pathlib import Path

import numpy as np

from ccgins.cli import _build_inputs, _filter_config, load_config
from ccgins.filters import ESKF, NoiseParams
from ccgins.geometry import skew, so3_exp, so3_left_jacobian
from ccgins.learning import WINDOW
from ccgins.sim import G_NAV


def bias_spectrum(rows):
    """Profile out initial attitude, velocity and position without a prior."""
    design = np.vstack(rows)
    nuisance, singular, _ = np.linalg.svd(design[:, :9], full_matrices=False)
    basis = nuisance[:, singular > singular[0] * max(design.shape) * np.finfo(float).eps]
    biases = design[:, 9:]
    profiled = biases - basis @ (basis.T @ biases)
    _, strength, directions = np.linalg.svd(profiled, full_matrices=False)
    return dict(measurement_rows=len(design), nuisance_rank=basis.shape[1],
                bias_singular_values=strength.tolist(),
                weakest_bias_direction=directions[-1].tolist())


def discrete_tangent(R, gyro, accel, dt):
    """Differentiate the ENU mechanization without Earth rotation."""
    step = np.eye(15)
    rotation = -gyro * dt
    step[:3, :3] = so3_exp(rotation)
    step[:3, 9:12] = -so3_left_jacobian(rotation) * dt
    accel_attitude = -R @ skew(accel)
    step[3:6, :3] = accel_attitude * dt
    step[3:6, 12:15] = -R * dt
    step[6:9, :3] = accel_attitude * (0.5 * dt**2)
    step[6:9, 3:6] = np.eye(3) * dt
    step[6:9, 12:15] = -R * (0.5 * dt**2)
    return step


def analyze_window(truth, gnss, spec, gs, cfg, start, end, straight=False):
    """Linearize along ideal IMU inputs and whiten by nominal update sigmas."""
    dt = truth.dt
    first, stop = np.searchsorted(truth.t, [start, end])
    # Use a constant-speed level control with matched duration, fixes and cadence.
    speed = float(np.mean(truth.v_body[first:stop, 0]))
    scale = np.r_[np.full(3, np.deg2rad(1)), np.ones(6),
                  np.full(3, np.deg2rad(spec.gyro_bias_sigma) / 3600),
                  np.full(3, spec.accel_bias_sigma * 1e-3 * 9.81)]
    transition = np.diag(scale)
    filter_ = ESKF(np.eye(3), [speed, 0, 0], np.zeros(3), np.eye(15),
                   NoiseParams.from_spec(spec, dt=dt), earth={"g": G_NAV, "w_ie": None})
    fixes = set(gnss.idx[gnss.valid])
    nhc_sigma = np.sqrt(cfg.nhc_variances())
    rows = []
    turn = 0.0
    constraint_residuals = []
    for idx in range(first, stop):
        if idx > first:
            gyro = np.zeros(3) if straight else truth.gyro[idx - 1]
            accel = -G_NAV if straight else truth.accel[idx - 1]
            transition = discrete_tangent(filter_.R, gyro, accel, dt) @ transition
            turn += abs(gyro[2]) * dt
        if not straight:
            filter_.R = truth.R[idx]
            filter_.v = truth.v[idx]
            filter_.p = truth.p[idx]
        if idx in fixes:
            _, H = filter_.gnss_model(gs.lever_arm)
            rows.append((H @ transition) / gnss.sigma[:, None])
        if idx >= WINDOW and idx % cfg.pseudo_every == 0:
            _, H = filter_.body_vel_model((1, 2))
            rows.append((H @ transition) / nhc_sigma[:, None])
            if not straight:
                constraint_residuals.append(truth.v_body[idx, 1:])
    result = dict(start=start, end=end, straight=straight, mean_speed_mps=speed,
                  integrated_abs_body_z_rotation_deg=float(np.rad2deg(turn)),
                  bias_scale_si=scale[9:].tolist(), **bias_spectrum(rows))
    if constraint_residuals:
        residuals = np.asarray(constraint_residuals)
        result['truth_nhc_rms_mps'] = np.sqrt(np.mean(residuals**2, axis=0)).tolist()
        result['truth_nhc_peak_abs_mps'] = np.max(np.abs(residuals), axis=0).tolist()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--seeds', type=int, nargs='+', default=[900, 68433, 68434, 68435, 68436])
    args = parser.parse_args()
    raw = load_config('configs/example_sim.yaml')
    cfg = _filter_config(raw)
    result = dict(config=raw, bias_coordinates=['bg_x', 'bg_y', 'bg_z', 'ba_x', 'ba_y', 'ba_z'],
                  weakest_bias_direction_coordinates='standardized: multiply each component by bias_scale_si',
                  windows=[])
    args.out.parent.mkdir(parents=True, exist_ok=True)
    for seed in args.seeds:
        truth, _, gnss, spec, gs = _build_inputs(raw, seed_override=seed)
        for start, end in [(30, 60), (60, 90), (90, 120), (120, 180)]:
            for straight in (False, True):
                row = analyze_window(truth, gnss, spec, gs, cfg, start, end, straight)
                result['windows'].append(dict(seed=seed, **row))
        args.out.write_text(json.dumps(result, indent=2) + '\n')


if __name__ == '__main__':
    main()
