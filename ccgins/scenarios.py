"""Scenario factory shared by training / experiments."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .sim import GnssSpec, ImuSpec, generate_trajectory, simulate_gnss, simulate_imu

TRAIN_SEEDS = list(range(100, 120))
VAL_SEEDS = list(range(500, 504))
TEST_SEEDS = list(range(900, 910))
CALIB_SEEDS = list(range(700, 706))   # clean runs used to set CUSUM thresholds

OUTAGES = [(150.0, 30.0), (280.0, 60.0), (420.0, 120.0)]


@dataclass
class Scenario:
    seed: int
    truth: object
    imu: object
    gnss: object
    imu_spec: ImuSpec
    gnss_spec: GnssSpec

    def imu6(self):
        return np.concatenate([self.imu.gyro, self.imu.accel], axis=1)


def make_scenario(seed: int, duration: float = 600.0, outages=None, spoof_start=None,
                  spoof_rate: float = 0.0, imu_spec: ImuSpec | None = None) -> Scenario:
    tr = generate_trajectory(duration, seed=seed)
    isp = imu_spec or ImuSpec()
    imu = simulate_imu(tr, isp, seed=seed)
    gs = GnssSpec(outages=list(outages or []), spoof_start=spoof_start, spoof_rate=spoof_rate)
    gn = simulate_gnss(tr, gs, seed=seed)
    return Scenario(seed, tr, imu, gn, isp, gs)
