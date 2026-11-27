"""Offline, truth-initialized INS component ablations; never filter inputs.

Sampled white noise includes the simulator's speed-dependent vibration. Bias
paths include both turn-on bias and random walk. Component RMS values do not
add: rotation, cancellation and time integration couple the signed errors.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from ccgins.cli import load_config, _build_inputs
from ccgins.filters import mechanize


def components(truth, imu):
    zero = np.zeros_like(imu.gyro)
    ng = imu.gyro - truth.gyro - imu.gyro_bias
    na = imu.accel - truth.accel - imu.accel_bias
    return {
        'ideal': (zero, zero),
        'gyro_bias': (imu.gyro_bias, zero),
        'accel_bias': (zero, imu.accel_bias),
        'gyro_sample_noise': (ng, zero),
        'accel_sample_noise': (zero, na),
        'biases': (imu.gyro_bias, imu.accel_bias),
        'sample_noise': (ng, na),
        'all': (imu.gyro - truth.gyro, imu.accel - truth.accel),
    }


def integrate(truth, gyro, accel, start, duration):
    first = int(round(start / truth.dt))
    last = int(round((start + duration) / truth.dt))
    if first < 0 or last >= len(truth.t) or last <= first:
        raise ValueError('integration interval outside truth')
    R, v, p = (x[first].copy() for x in (truth.R, truth.v, truth.p))
    errors = []
    for k in range(first, last):
        R, v, p = mechanize(R, v, p, gyro[k], accel[k], truth.dt,
                           earth={'w_ie': None, 'g': np.array([0., 0., -9.81])})
        errors.append(p - truth.p[k + 1])
    return truth.t[first + 1:last + 1], np.asarray(errors)


def metrics(error):
    h = np.linalg.norm(error[:, :2], axis=1)
    return dict(n=len(error), rmse_h=float(np.sqrt(np.mean(h**2))),
                rmse_v=float(np.sqrt(np.mean(error[:, 2]**2))),
                end_h=float(h[-1]), end_v=float(abs(error[-1, 2])),
                end_enu=error[-1].tolist())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--seeds', type=int, nargs='+', default=[900, 96001, 96002])
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    raw = load_config('configs/example_sim.yaml')
    result = dict(config=raw, scope='Offline pure INS, exact truth at gap entry; '
                  'full interval endpoint before reacquisition; no aiding or bias subtraction.', rows=[])
    for seed in args.seeds:
        truth, imu, _, _, gs = _build_inputs(raw, seed_override=seed)
        for start, duration in gs.outages:
            row = dict(seed=seed, start=start, duration=duration, components={})
            traces = {}
            for name, (dg, da) in components(truth, imu).items():
                # Use original arrays for all to avoid subtraction/addition rounding.
                gyro = imu.gyro if name == 'all' else truth.gyro + dg
                accel = imu.accel if name == 'all' else truth.accel + da
                t, error = integrate(truth, gyro, accel, start, duration)
                row['components'][name] = metrics(error)
                traces[name] = error
            np.savez_compressed(args.out / f'trace_{seed}_{start:g}.npz', t=t, **traces)
            result['rows'].append(row)
            (args.out / 'components.json').write_text(json.dumps(result, indent=2) + '\n')
            print(seed, start, row['components']['all'], flush=True)


if __name__ == '__main__':
    main()
