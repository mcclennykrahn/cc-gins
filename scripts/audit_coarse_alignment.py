"""Offline validation of a causal IMU-only gravity alignment; no filter changes.

Only samples with t < 5 s enter the estimate. Truth is used solely for scoring.
Heading is deliberately absent: gravity cannot determine rotation about nav up.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from ccgins.cli import _build_inputs, load_config


def gravity_direction(accel):
    """Estimate body up from a presumed stationary prefix, without a yaw claim."""
    a = np.asarray(accel, dtype=float)
    if a.ndim != 2 or a.shape[1] != 3 or not len(a) or not np.isfinite(a).all():
        raise ValueError('expected nonempty finite Nx3 specific force')
    mean = a.mean(axis=0)
    norm = np.linalg.norm(mean)
    if norm < 1e-6:
        raise ValueError('gravity direction undefined')
    return mean / norm


def angle_deg(a, b):
    # atan2 preserves small angles better than acos.
    return float(np.rad2deg(np.arctan2(np.linalg.norm(np.cross(a, b)), np.dot(a, b))))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--seeds', type=int, nargs='+', default=[900, 115001, 115002])
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    raw = load_config('configs/example_sim.yaml')
    result = dict(candidate_id='scheme_coarse_alignment', production_changed=False,
                  window='0 <= t < 5 s; estimate available at 5 s',
                  heading='unobservable from gravity; no heading estimate supplied', rows=[])
    for seed in args.seeds:
        truth, imu, _, _, _ = _build_inputs(raw, seed_override=seed)
        prefix = (truth.t >= 0) & (truth.t < 5.)
        end = int(np.searchsorted(truth.t, 5.))
        up_end = truth.R[end].T @ np.array([0., 0., 1.])
        ideal = gravity_direction(truth.accel[prefix])
        measured = gravity_direction(imu.accel[prefix])
        # Remove actual injected bias only as a labeled diagnostic oracle.
        debiased = gravity_direction(imu.accel[prefix] - imu.accel_bias[prefix])
        row = dict(seed=seed, samples=int(prefix.sum()),
                   max_prefix_speed_mps=float(np.linalg.norm(truth.v[prefix], axis=1).max()),
                   max_prefix_angular_rate_deg_s=float(np.rad2deg(np.linalg.norm(truth.gyro[prefix], axis=1).max())),
                   ideal_tilt_error_deg=angle_deg(ideal, up_end),
                   measured_tilt_error_deg=angle_deg(measured, up_end),
                   oracle_debiased_tilt_error_deg=angle_deg(debiased, up_end),
                   mean_accel_bias_mps2=imu.accel_bias[prefix].mean(axis=0).tolist(),
                   mean_gyro_deg_s=np.rad2deg(imu.gyro[prefix].mean(axis=0)).tolist(),
                   true_mean_gyro_deg_s=np.rad2deg(truth.gyro[prefix].mean(axis=0)).tolist())
        result['rows'].append(row)
        (args.out / 'alignment.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
        print(json.dumps(row), flush=True)


if __name__ == '__main__':
    main()
