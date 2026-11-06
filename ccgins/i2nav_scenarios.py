"""Real-data scenarios from the i2Nav-WHU awesome-gins-datasets drive (1617 s, Wuhan, open sky).

Same vehicle, same trajectory, FOUR real MEMS IMUs (ICM20602 consumer, ADIS16460/ADIS16465
industrial, HGuide i300 tactical-MEMS), REAL 1 Hz RTK (with receiver-reported std), and a
post-processed reference (truth.nav, per IMU centre). Nothing is simulated except the injected
GNSS outages (epochs removed) and the spoofing ramp added on top of the real RTK fixes.

Held-out design (single drive): 2-fold TIME split. Fold "A" tests on the first half and trains on
the second half; fold "B" the reverse; MotionNet is trained only on ICM20602 + ADIS16460 so
ADIS16465 and i300 are additionally UNSEEN sensors.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

from .data.i2nav import crop, load_npz
from .scenarios import Scenario
from .sim import DEG, GnssData, GnssSpec, ImuData, ImuSpec

DATA = Path(__file__).resolve().parents[1] / "data" / "i2nav"
IMUS = ["ICM20602", "ADIS16460", "ADIS16465", "i300"]
TRAIN_IMUS = ["ICM20602", "ADIS16460"]
T_TOTAL = 1617.0
T_SPLIT = 808.0
GUARD = 10.0  # s, gap between train and test halves
# reference noise table of the dataset (ARW deg/sqrt(h), VRW m/s/sqrt(h), gyro-bias instab. deg/h,
# accel-bias instab. mGal, correlation time 1 h)
REF_NOISE = {"ICM20602": (0.2, 0.2, 200.0, 1000.0), "ADIS16460": (0.2, 0.1, 20.0, 100.0),
             "ADIS16465": (0.1, 0.1, 25.0, 200.0), "i300": (0.2, 0.2, 15.0, 150.0)}
# turn-on bias prior: no calibration / static alignment is available (the drive starts moving), so the
# same wide prior is used for every sensor (2000 deg/h, 20 mg); the RTK-aided filter estimates them.
P0_BIAS = (2000.0 * DEG / 3600.0, 20e-3 * 9.81)
GNSS_SIGMA_FLOOR = 0.05  # m, floor on receiver-reported RTK std (unmodelled lever/sync/INS errors)


def filter_spec(name: str) -> ImuSpec:
    arw, vrw, gb, ab = REF_NOISE[name]
    tau_h = 1.0
    ab_mg = ab * 1e-5 / 9.81 * 1e3
    return ImuSpec(gyro_arw=arw, accel_vrw=vrw, gyro_bias_sigma=gb, accel_bias_sigma=ab_mg,
                   gyro_bias_rw=gb * np.sqrt(2.0 / tau_h),                 # deg/h/sqrt(h)
                   accel_bias_rw=ab_mg * np.sqrt(2.0 / (tau_h * 3600.0)),  # mg/sqrt(s)
                   vib_accel_per_mps=0.0, vib_gyro_per_mps=0.0)


_cache = {}


def load(name: str):
    if name not in _cache:
        _cache[name] = load_npz(DATA / f"{name}.npz")
    return _cache[name]


def fold_ranges(fold: str):
    """(train_range, test_range) in seconds."""
    a, b = (0.0, T_SPLIT - GUARD / 2), (T_SPLIT + GUARD / 2, T_TOTAL - 1.0)
    return (b, a) if fold == "A" else (a, b)


def segment(name: str, t0: float, t1: float):
    return crop(load(name), t0, t1)


def outage_schedule(duration: float, first: float = 60.0, gap: float = 40.0, tail: float = 20.0,
                    durations=(30.0, 120.0, 60.0, 30.0, 120.0, 60.0)):
    out, s = [], first
    for d in durations:
        if s + d <= duration - tail:
            out.append((s, d))
            s = s + d + gap
    return out


def make_scenario(seq, seed: int = 0, outages=(), spoof_start=None, spoof_rate: float = 0.0,
                  spoof_dir_deg: float = 45.0) -> Scenario:
    tr = seq.truth
    n = len(tr.t)
    imu = ImuData(t=tr.t, gyro=tr.gyro, accel=tr.accel, gyro_bias=np.full((n, 3), np.nan),
                  accel_bias=np.full((n, 3), np.nan), dt=tr.dt)
    t = seq.gnss_t
    valid = np.ones(len(t), bool)
    for s, d in outages:
        valid &= ~((t >= s) & (t < s + d))
    pos = seq.gnss_pos.copy()
    spoofed = np.zeros(len(t), bool)
    if spoof_start is not None:
        spoofed = t >= spoof_start
        dirv = np.array([np.cos(spoof_dir_deg * DEG), np.sin(spoof_dir_deg * DEG), 0.0])
        pos = pos + np.clip(t - spoof_start, 0, None)[:, None] * spoof_rate * dirv
    sig_nom = np.maximum(np.median(seq.gnss_std, axis=0), GNSS_SIGMA_FLOOR)
    gn = GnssData(t=t, idx=seq.gnss_idx, pos=pos, valid=valid, spoofed=spoofed, sigma=sig_nom,
                  sigma_ep=seq.gnss_std)
    gs = GnssSpec(rate_hz=1.0, sigma_h=float(sig_nom[0]), sigma_v=float(sig_nom[2]), mp_sigma=0.0,
                  lever_arm=seq.lever_flu, outages=list(outages), spoof_start=spoof_start,
                  spoof_rate=spoof_rate, spoof_dir_deg=spoof_dir_deg)
    return Scenario(seed, tr, imu, gn, filter_spec(seq.name), gs)


REAL_CFG = dict(use_epoch_sigma=True, gnss_sigma_floor=GNSS_SIGMA_FLOOR, p0_bias=P0_BIAS)


def enable_earth(seq):
    from . import filters
    filters.set_earth(seq.w_ie_enu, seq.g_local)


def calibrate_mount(seq, seed: int = 0, t_skip: float = 60.0, v_min: float = 3.0, sigma_floor: float = 0.05,
                    return_eff: bool = False):
    """Installation calibration WITHOUT reference truth: run the RTK/INS filter (no motion constraint)
    over a calibration segment and estimate (pitch, yaw) of the vehicle forward axis in the IMU frame
    from the filter's own body-frame velocity (speed > v_min, after t_skip s of convergence). Also returns
    the RMS of the residual lateral/vertical vehicle-frame velocity and the correlation-aware NHC sigma
    sigma_eff = max_c RMS_c * sqrt(n_eff,c): NHC errors on a real car are strongly time-correlated
    (n_eff ~ 50-140 at 10 Hz), so treating them as white at 0.1 m/s makes the filter overconfident.
    """
    from .pipeline import Config, mount_matrix, run
    enable_earth(seq)
    sc = make_scenario(seq, seed)
    log = run(sc.truth, sc.imu, sc.gnss, sc.imu_spec, sc.gnss_spec,
              Config(filter="inekf", pseudo="none", init_seed=seed, **REAL_CFG))
    vb = log.extra["vb_est"]
    sp = np.linalg.norm(vb, axis=1)
    m = (log.t > t_skip) & (sp > v_min)
    u = (vb[m] / sp[m, None]).mean(0)
    yaw = float(np.arctan2(u[1], u[0]))
    pitch = float(np.arctan2(u[2], np.hypot(u[0], u[1])))
    vv = vb[m] @ mount_matrix(pitch, yaw).T
    sig = np.maximum(np.sqrt((vv[:, 1:] ** 2).mean(0)), sigma_floor)
    if not return_eff:
        return (pitch, yaw), tuple(float(x) for x in sig)
    from .learning import correlation_factor
    rms = np.sqrt((vv[:, 1:] ** 2).mean(0))
    neff = correlation_factor(vv[:, 1:] / rms)
    sig_eff = float(np.max(rms * np.sqrt(neff)))
    return (pitch, yaw), tuple(float(x) for x in sig), sig_eff, tuple(float(x) for x in neff)


def earth_config(seq):
    """Explicit per-run Earth parameters from loader metadata, with no global mutation."""
    return dict(earth_rate_enu=tuple(seq.w_ie_enu), local_gravity=seq.g_local)
