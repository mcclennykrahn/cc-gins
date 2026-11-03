"""KITTI-raw based scenarios: real OXTS motion/attitude as ground truth, OXTS IMU (+ optional
injected MEMS-grade errors) as IMU, synthetic 1 Hz GNSS from ground truth with outages/spoofing."""
from __future__ import annotations

from pathlib import Path

import numpy as np

from .data.kitti import segments_from_npz
from .scenarios import Scenario
from .sim import GnssSpec, ImuSpec, simulate_gnss, simulate_imu

DATA = Path(__file__).resolve().parents[1] / "data" / "kitti"
TRAIN_DRIVES = ["2011_09_30_drive_0018", "2011_09_30_drive_0027", "2011_09_30_drive_0034",
                "2011_10_03_drive_0027"]
VAL_DRIVES = ["2011_09_30_drive_0033"]
TEST_DRIVES = ["2011_09_30_drive_0028", "2011_10_03_drive_0034", "2011_10_03_drive_0042"]

# injected errors on top of the (tactical-grade, already clean) OXTS IMU: low-cost MEMS level,
# no synthetic vibration (real vibration is already in the data)
MEMS_INJECT = ImuSpec(vib_accel_per_mps=0.0, vib_gyro_per_mps=0.0)
FILTER_SPEC = ImuSpec()  # filter noise model (incl. vibration allowance)


def drive_segments(drive: str, min_len_s: float = 30.0):
    return segments_from_npz(DATA / f"{drive}_extract.npz", min_len_s=min_len_s)


def outage_schedule(duration: float, first: float = 60.0, gap: float = 40.0, tail: float = 20.0,
                    durations=(30.0, 120.0, 60.0)):
    out, s = [], first
    for d in durations:
        if s + d <= duration - tail:
            out.append((s, d))
            s = s + d + gap
    return out


def make_kitti_scenario(truth, seed: int, outages=(), spoof_start=None, spoof_rate=0.0,
                        inject: bool = True) -> Scenario:
    if inject:
        imu = simulate_imu(truth, MEMS_INJECT, seed=seed)
    else:
        imu = simulate_imu(truth, MEMS_INJECT, seed=seed, noise=False)
    gs = GnssSpec(outages=list(outages), spoof_start=spoof_start, spoof_rate=spoof_rate)
    gn = simulate_gnss(truth, gs, seed=seed)
    return Scenario(seed, truth, imu, gn, FILTER_SPEC, gs)
