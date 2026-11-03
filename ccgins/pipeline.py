"""End-to-end GNSS/INS pipeline: IMU propagation, GNSS update, motion pseudo-measurements,
ACI calibration, integrity monitoring."""
from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np
from scipy.stats import chi2

from .aci import ACICalibrator
from .filters import NoiseParams, make_filter
from .geometry import euler_to_rot
from .integrity import Cusum, dr_statistic
from .learning import WINDOW
from .sim import DEG


@dataclass
class Config:
    filter: str = "inekf"            # eskf | inekf
    pseudo: str = "none"             # none | nhc | learned_nhc | learned_vel
    nhc_sigma: float = 0.1           # m/s, fixed NHC
    pseudo_every: int = 10           # IMU samples (10 Hz)
    aci: bool = False
    aci_alpha: float = 0.05
    aci_gamma: float = 0.01
    aci_window: int = 300
    aci_beta: float = 0.5
    aci_t_ref: float = 60.0
    gnss_gate: float = float(chi2.ppf(0.999, 3))
    max_consecutive_rejects: int = 5
    monitor: str = "none"            # none | single | dual
    cusum_k: float = 3.0
    cusum_h: float = 50.0
    dr_reset_period: float = 30.0
    exclude_on_alarm: bool = True
    init_seed: int = 0
    label: str = ""


@dataclass
class RunLog:
    t: np.ndarray
    p_est: np.ndarray
    p_true: np.ndarray
    P_pos: np.ndarray
    v_est: np.ndarray
    v_true: np.ndarray
    gnss_t: np.ndarray
    stat_q: np.ndarray
    stat_g: np.ndarray
    alarm_time: float | None
    kappa_hist: np.ndarray
    runtime_per_step_us: float
    extra: dict = field(default_factory=dict)


def initial_state(truth, seed):
    rng = np.random.default_rng(seed + 30_000)
    att_sig = np.array([0.5, 0.5, 3.0]) * DEG
    datt = rng.standard_normal(3) * att_sig
    R0 = euler_to_rot(*datt) @ truth.R[0]  # small nav-frame error (Z-Y-X small angles ~ vector)
    v0 = truth.v[0] + rng.standard_normal(3) * 0.1
    p0 = truth.p[0] + rng.standard_normal(3) * np.array([1.5, 1.5, 3.0])
    P0 = np.diag(np.r_[(np.array([1.0, 1.0, 5.0]) * DEG) ** 2, np.full(3, 0.3**2),
                       np.array([2.0, 2.0, 4.0]) ** 2,
                       np.full(3, (40 * DEG / 3600) ** 2), np.full(3, (8e-3 * 9.81) ** 2)])
    return R0, v0, p0, P0


def run(truth, imu, gnss, imu_spec, gnss_spec, cfg: Config, pred=None) -> RunLog:
    """pred: (idx, mu, var) learned predictions at pseudo epochs (idx = IMU end index)."""
    noise = NoiseParams.from_spec(imu_spec)
    R0, v0, p0, P0 = initial_state(truth, cfg.init_seed)
    A = make_filter(cfg.filter, R0, v0, p0, P0, noise)
    lever = gnss_spec.lever_arm
    Rm = np.diag(gnss.sigma**2)
    n = len(truth.t)
    dt = truth.dt

    pred_map = {}
    if pred is not None:
        for i, k in enumerate(pred[0]):
            pred_map[int(k)] = (pred[1][i], pred[2][i])

    chans = (1, 2) if cfg.pseudo in ("nhc", "learned_nhc") else (0, 1, 2)
    cal = ACICalibrator(len(chans), alpha=cfg.aci_alpha, gamma=cfg.aci_gamma, window=cfg.aci_window,
                        beta=cfg.aci_beta, t_ref=cfg.aci_t_ref, enabled=cfg.aci)

    gnss_at = {int(k): m for m, k in enumerate(gnss.idx)}
    t_last_gnss = 0.0
    rejects = 0
    cus = Cusum(cfg.cusum_k, cfg.cusum_h)
    B = A.copy() if cfg.monitor == "dual" else None
    t_last_reset = 0.0
    alarm_time = None

    logs = {k: [] for k in ("t", "pe", "pt", "P", "ve", "vt")}
    gq = {"t": [], "q": [], "g": []}
    t0 = time.perf_counter()

    for k in range(n - 1):
        A.propagate(imu.gyro[k], imu.accel[k], dt)
        if B is not None:
            B.propagate(imu.gyro[k], imu.accel[k], dt)
        kk = k + 1
        t = truth.t[kk]
        tau = t - t_last_gnss

        # ---------- motion pseudo-measurement ----------
        if cfg.pseudo != "none" and kk % cfg.pseudo_every == 0 and kk >= WINDOW:
            ok = True
            if cfg.pseudo == "nhc":
                z = np.zeros(2)
                s2 = np.full(2, cfg.nhc_sigma**2)
            else:
                if kk not in pred_map:
                    ok = False
                else:
                    mu, var = pred_map[kk]
                    if cfg.pseudo == "learned_nhc":
                        z = np.zeros(2)
                        s2 = var[1:] + mu[1:] ** 2
                    else:
                        z = mu.copy()
                        s2 = var.copy()
            if ok:
                h, H = A.body_vel_model(chans)
                healthy_ref = (tau < 1.5) and (t > 30.0) and (alarm_time is None)
                if cfg.aci and healthy_ref:
                    c = np.einsum("ij,jk,ik->i", H, A.P, H)
                    cal.observe(z - h, s2, c)
                out_t = 0.0 if tau < 1.5 else tau
                var_eff = cal.effective_var(s2, out_t)
                A.update(z - h, H, np.diag(var_eff))
                if B is not None:
                    hB, HB = B.body_vel_model(chans)
                    B.update(z - hB, HB, np.diag(var_eff))

        # ---------- GNSS ----------
        if kk in gnss_at:
            m = gnss_at[kk]
            if gnss.valid[m]:
                zg = gnss.pos[m]
                # integrity statistic
                if cfg.monitor == "dual":
                    q = dr_statistic(B, zg, lever, Rm)
                elif cfg.monitor == "single":
                    h, H = A.gnss_model(lever)
                    S = H @ A.P @ H.T + Rm
                    q = float((zg - h) @ np.linalg.solve(S, zg - h))
                else:
                    q = np.nan
                if cfg.monitor != "none":
                    was = cus.alarm
                    cus.step(q)
                    if cus.alarm and not was:
                        alarm_time = t
                    gq["t"].append(t)
                    gq["q"].append(q)
                    gq["g"].append(cus.g)
                use = not (cus.alarm and cfg.exclude_on_alarm)
                if use:
                    h, H = A.gnss_model(lever)
                    force = rejects >= cfg.max_consecutive_rejects
                    info = A.update(zg - h, H, Rm, gate=cfg.gnss_gate, force=force)
                    if info.accepted:
                        rejects = 0
                        t_last_gnss = t
                    else:
                        rejects += 1
            # DR channel re-seeding
            if B is not None and (t - t_last_reset) >= cfg.dr_reset_period and not cus.alarm \
                    and cus.g < 0.25 * cfg.cusum_h and (t - t_last_gnss) < 1.5:
                B = A.copy()
                t_last_reset = t

        if kk % 10 == 0:
            logs["t"].append(t)
            logs["pe"].append(A.p.copy())
            logs["pt"].append(truth.p[kk].copy())
            logs["P"].append(A.pos_cov())
            logs["ve"].append(A.v.copy())
            logs["vt"].append(truth.v[kk].copy())

    rt = (time.perf_counter() - t0) / (n - 1) * 1e6
    return RunLog(
        t=np.array(logs["t"]), p_est=np.array(logs["pe"]), p_true=np.array(logs["pt"]),
        P_pos=np.array(logs["P"]), v_est=np.array(logs["ve"]), v_true=np.array(logs["vt"]),
        gnss_t=np.array(gq["t"]), stat_q=np.array(gq["q"]), stat_g=np.array(gq["g"]),
        alarm_time=alarm_time, kappa_hist=np.array(cal.history), runtime_per_step_us=rt,
    )
