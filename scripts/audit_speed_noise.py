"""Offline measurement of nominal process noise versus simulator white noise.

Truth speed and recorded biases are diagnostic inputs only; never filter aiding.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from ccgins.cli import _build_inputs, load_config
from ccgins.filters import NoiseParams
from ccgins.sim import DEG


def measure(raw, seed):
    truth, imu, _, spec, _ = _build_inputs(raw, seed_override=seed)
    density = spec.densities()
    nominal = NoiseParams.from_spec(spec, dt=truth.dt)
    speed = np.abs(truth.v_body[:, 0])
    channels = {}
    for channel, coefficient in [('gyro', spec.vib_gyro_per_mps * DEG),
                                 ('accel', spec.vib_accel_per_mps)]:
        residual = (getattr(imu, channel) - getattr(truth, channel)
                    - getattr(imu, channel + '_bias'))
        sigma = density[channel] / np.sqrt(truth.dt) + coefficient * speed
        channels[channel] = (residual, sigma, getattr(nominal, channel)**2 / truth.dt)
    segments = [('available_30_60', 30., 60.), ('outage_60_90', 60., 90.),
                ('available_90_120', 90., 120.), ('outage_120_180', 120., 180.)]
    rows = []
    for name, start, end in segments:
        # IMU[k] drives [t[k],t[k+1]); exclude the unused terminal sample.
        mask = (truth.t >= start) & (truth.t < end)
        row = dict(segment=name, n=int(mask.sum()),
                   mean_speed=float(speed[mask].mean()), channels={})
        for channel, (residual, sigma, nominal_variance) in channels.items():
            values = residual[mask]
            expected = float(np.mean(sigma[mask]**2))
            row['channels'][channel] = dict(
                nominal_sample_variance=float(nominal_variance),
                expected_sample_variance=expected,
                measured_second_moment=float(np.mean(values**2)),
                expected_to_nominal=expected / nominal_variance,
                measured_to_expected=float(np.mean(values**2)) / expected,
                standardized_second_moment=float(np.mean((values / sigma[mask, None])**2)),
            )
        rows.append(row)
    return dict(seed=seed, segments=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--seeds', nargs='+', type=int, default=[900, 91001, 91002, 91003, 91004])
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    raw = load_config('configs/example_sim.yaml')
    result = dict(config=raw, scoring_start=30., online_filter_changed=False, rows=[])
    for seed in args.seeds:
        result['rows'].append(measure(raw, seed))
        (args.out / 'metrics.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')


if __name__ == '__main__':
    main()
