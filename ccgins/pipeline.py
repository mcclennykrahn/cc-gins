"""End-to-end GNSS/INS pipeline: IMU propagation, GNSS update, motion pseudo-measurements,
ACI calibration, integrity monitoring."""
from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np
from scipy.stats import chi2

from .aci import ACICalibrator
from .adaptive import AdaptiveR
from .filters import NoiseParams, make_filter
from .geometry import euler_to_rot
from .integrity import Cusum, dr_statistic
from .learning import WINDOW
from .sim import DEG
from .range_coupling import BaselineRangeCoupler
from .zupt import AdaptiveZeroVelocity, zupt_update, mounted_odometry_update


@dataclass
class Config:
    filter: str = "inekf"            # eskf | inekf
    pseudo: str = "none"             # none | nhc | learned_nhc | learned_vel | hybrid
    # hybrid: NHC (fixed sigma) for lateral/vertical + learned forward speed (corr-inflated, ACI-
    # calibrated); forward channel dropped (fallback to pure NHC) when its calibrated std exceeds
    # hybrid_max_std.
    hybrid_max_std: float = 1e9
    hybrid_max_kappa: float = 1e9    # fallback to NHC when ACI scale kappa (forward) exceeds this
    # bias monitor (v3): while GNSS is healthy, test the windowed MEAN of the forward residual
    # (learned speed - GNSS-constrained filter speed) against its correlation-aware std
    # sqrt(mean(var) * n_eff / W). |z| > hybrid_bias_z -> drop the forward channel (latched,
    # re-enabled below hybrid_bias_z/2; frozen during outages). ACI only rescales variance and
    # cannot see a bias that the inflated variance already "covers"; this test can. 0 = off.
    hybrid_bias_z: float = 0.0
    hybrid_bias_window: int = 100     # pseudo-measurement epochs (10 Hz -> 10 s)
    hybrid_bias_min: int = 30         # need this many residuals before trusting the forward channel
    hybrid_bias_latch: bool = False   # once a bias is detected, never re-enable the forward channel
    nhc_sigma: float = 0.1           # m/s, fixed NHC
    pseudo_every: int = 10           # IMU samples (10 Hz)
    corr_scale: tuple | None = None  # per-channel variance inflation for temporally correlated errors (n_eff)
    aci: bool = False
    aci_alpha: float = 0.05
    aci_gamma: float = 0.01
    aci_window: int = 300
    aci_beta: float = 2.0             # chosen on VAL seeds (sweep 0.5/2/5/10, small effect)
    aci_t_ref: float = 60.0
    gnss_gate: float = float(chi2.ppf(0.999, 3))
    max_consecutive_rejects: int = 5
    monitor: str = "none"            # none | single | dual
    cusum_k: float = 3.0
    cusum_h: float = 50.0
    dr_reset_period: float = 30.0
    exclude_on_alarm: bool = True
    init_seed: int = 0
    # real data: use the receiver's per-epoch std (GnssData.sigma_ep) floored at gnss_sigma_floor
    use_epoch_sigma: bool = False
    gnss_sigma_floor: float = 0.0
    # opt-in innovation-based R estimation (ccgins/adaptive.py); off = existing behaviour
    robust_gnss_delta: float | None = None  # Huber innovation threshold
    adaptive_r: bool = False
    # IMU-to-vehicle mounting (real data): (pitch, yaw) [rad] of the vehicle forward axis seen in the
    # IMU body frame. NHC / hybrid lateral-vertical constraints are applied in the VEHICLE frame
    # v_veh = C_vb v_body. None = IMU assumed aligned with the vehicle (simulation / KITTI).
    mount_py: tuple | None = None
    p0_bias: tuple | None = None      # (gyro rad/s, accel m/s^2) initial bias std; None = legacy
    baseline_range_start_s: float = 0.0  # optional vector-RTK warmup
    baseline_range_anchor: tuple | None = None  # replace RTK vector by one scalar range
    earth_rate_enu: tuple | None = None
    local_gravity: float | None = None
    zupt: bool = False
    zupt_require_gnss_stop: bool = True  # opt out only with independent external stop assurance
    zupt_accel_var: float | None = None  # frozen held-out calibration; None = noise-only floor
    zupt_gyro_var: float | None = None
    zupt_sigma: float = 0.05
    motion_every: int = 10
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


def initial_state(truth, seed, p0_bias=None):
    rng = np.random.default_rng(seed + 30_000)
    att_sig = np.array([0.5, 0.5, 3.0]) * DEG
    datt = rng.standard_normal(3) * att_sig
    R0 = euler_to_rot(*datt) @ truth.R[0]  # small nav-frame error (Z-Y-X small angles ~ vector)
    v0 = truth.v[0] + rng.standard_normal(3) * 0.1
    p0 = truth.p[0] + rng.standard_normal(3) * np.array([1.5, 1.5, 3.0])
    P0 = np.diag(np.r_[(np.array([1.0, 1.0, 5.0]) * DEG) ** 2, np.full(3, 0.3**2),
                       np.array([2.0, 2.0, 4.0]) ** 2,
                       np.full(3, (40 * DEG / 3600) ** 2), np.full(3, (8e-3 * 9.81) ** 2)])
    if p0_bias is not None:
        P0[9:12, 9:12] = np.eye(3) * p0_bias[0] ** 2
        P0[12:15, 12:15] = np.eye(3) * p0_bias[1] ** 2
    return R0, v0, p0, P0


def mount_matrix(pitch, yaw):
    """C_vb (body -> vehicle). Vehicle x-axis expressed in body FLU = (cp*cy, cp*sy, sp)."""
    from .geometry import so3_exp
    C_bv = so3_exp(np.array([0.0, 0.0, yaw])) @ so3_exp(np.array([0.0, -pitch, 0.0]))
    return C_bv.T


def run(truth, imu, gnss, imu_spec, gnss_spec, cfg: Config, pred=None, odometry=None) -> RunLog:
    """pred: (idx, mu, var) learned predictions at pseudo epochs (idx = IMU end index)."""
    if cfg.motion_every < 1 or not np.isfinite(cfg.zupt_sigma) or cfg.zupt_sigma <= 0:
        raise ValueError("positive motion cadence and ZUPT sigma required")
    noise = NoiseParams.from_spec(imu_spec)
    C_vb = None if cfg.mount_py is None else mount_matrix(*cfg.mount_py)

    def bvm(F, ch):
        h, H = F.body_vel_model((0, 1, 2))
        if C_vb is not None and cfg.pseudo in ("nhc", "hybrid", "learned_nhc"):
            h, H = C_vb @ h, C_vb @ H
        ch = list(ch)
        return h[ch], H[ch]
    R0, v0, p0, P0 = initial_state(truth, cfg.init_seed, cfg.p0_bias)
    earth = None
    if cfg.earth_rate_enu is not None or cfg.local_gravity is not None:
        earth = {"w_ie": None if cfg.earth_rate_enu is None else np.asarray(cfg.earth_rate_enu),
                 "g": np.array([0., 0., -(9.81 if cfg.local_gravity is None else cfg.local_gravity)])}
    A = make_filter(cfg.filter, R0, v0, p0, P0, noise, earth=earth)
    lever = gnss_spec.lever_arm
    Rm = np.diag(gnss.sigma**2)
    adaptive = AdaptiveR(Rm) if cfg.adaptive_r else None
    coupler = None if cfg.baseline_range_anchor is None else BaselineRangeCoupler(cfg.baseline_range_anchor)
    range_alarm_time = None
    range_trace = []
    n = len(truth.t)
    dt = truth.dt

    detector = None
    if cfg.zupt:
        d = imu_spec.densities()
        detector = AdaptiveZeroVelocity(
            cfg.zupt_accel_var if cfg.zupt_accel_var is not None else 4*d['accel']**2/dt,
            cfg.zupt_gyro_var if cfg.zupt_gyro_var is not None else 4*d['gyro']**2/dt,
            dt=dt, gravity=9.81 if cfg.local_gravity is None else cfg.local_gravity)
    motion = {"zupt_attempted": 0, "zupt_accepted": 0, "odometry_attempted": 0,
              "odometry_accepted": 0, "zupt_times": []}
    stop_verified = False
    stop_confirmations = 0
    previous_fix = None
    motion["unconfirmed_stop_vetoes"] = 0
    odo_at = {}
    if odometry is not None:
        for j, ot in enumerate(odometry.t):
            idx = int(round((ot - truth.t[0]) / dt))
            if idx < 1 or idx >= n or abs(truth.t[idx] - ot) > 1e-7 or idx in odo_at:
                raise ValueError("odometry epochs must be unique and aligned to propagated IMU grid")
            odo_at[idx] = j

    pred_map = {}
    if pred is not None:
        for i, k in enumerate(pred[0]):
            pred_map[int(k)] = (pred[1][i], pred[2][i])

    chans = (1, 2) if cfg.pseudo in ("nhc", "learned_nhc") else (0, 1, 2)
    cal_dim = 1 if cfg.pseudo == "hybrid" else len(chans)
    n_fallback = 0
    n_pseudo = 0
    from collections import deque
    bias_r = deque(maxlen=cfg.hybrid_bias_window)
    bias_v = deque(maxlen=cfg.hybrid_bias_window)
    bias_bad = cfg.hybrid_bias_z > 0
    bias_latched = False
    n_eff_f = float(cfg.corr_scale[0]) if cfg.corr_scale is not None else 1.0
    cal = ACICalibrator(cal_dim, alpha=cfg.aci_alpha, gamma=cfg.aci_gamma, window=cfg.aci_window,
                        beta=cfg.aci_beta, t_ref=cfg.aci_t_ref, enabled=cfg.aci)

    gnss_at = {int(k): m for m, k in enumerate(gnss.idx)}
    t_last_gnss = 0.0
    rejects = 0
    cus = Cusum(cfg.cusum_k, cfg.cusum_h)
    B = A.copy() if cfg.monitor == "dual" else None
    t_last_reset = 0.0
    alarm_time = None

    logs = {k: [] for k in ("t", "pe", "pt", "P", "ve", "vt", "vb", "tg")}
    gq = {"t": [], "q": [], "g": []}
    t0 = time.perf_counter()

    for k in range(n - 1):
        A.propagate(imu.gyro[k], imu.accel[k], dt)
        if B is not None:
            B.propagate(imu.gyro[k], imu.accel[k], dt)
        kk = k + 1
        t = truth.t[kk]
        tau = t - t_last_gnss

        # Only samples already propagated are visible to the causal detector.
        stationary = False if detector is None else detector.step(
            imu.gyro[k], imu.accel[k], np.linalg.norm(A.v))
        # A quiet IMU and a small FILTER speed cannot independently prove rest.
        # Once GNSS has confirmed rest, retain it through an outage only while
        # the IMU dwell remains uninterrupted. New outage stops stay unconfirmed.
        if detector is not None and detector.quiet == 0:
            stop_verified = False
            stop_confirmations = 0
        stop_allowed = stop_verified or not cfg.zupt_require_gnss_stop
        if stationary and not stop_allowed and kk % cfg.motion_every == 0:
            motion['unconfirmed_stop_vetoes'] += 1
        if stationary and stop_allowed and kk % cfg.motion_every == 0:
            motion['zupt_attempted'] += 1
            info = zupt_update(A, cfg.zupt_sigma, gate=float(chi2.ppf(.999, 3)))
            motion['zupt_accepted'] += int(info.accepted)
            if info.accepted:
                motion['zupt_times'].append(float(t))
            if B is not None:
                zupt_update(B, cfg.zupt_sigma, gate=float(chi2.ppf(.999, 3)))
        if kk in odo_at:
            j = odo_at[kk]
            if odometry.valid[j]:
                motion['odometry_attempted'] += 1
                for F in ([A] if B is None else [A, B]):
                    info = mounted_odometry_update(F, odometry.speed[j], odometry.sigma[j],
                        imu.gyro[k], odometry.C_vb, odometry.lever_b)
                    if F is A:
                        motion['odometry_accepted'] += int(info.accepted)

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
            if ok and cfg.pseudo == "hybrid":
                n_pseudo += 1
                h, H = bvm(A, (0, 1, 2))
                healthy_ref = (tau < 1.5) and (t > 30.0) and (alarm_time is None)
                if cfg.aci and healthy_ref:
                    c = np.einsum("ij,jk,ik->i", H[:1], A.P, H[:1])
                    cal.observe(z[:1] - h[:1], s2[:1], c)
                if cfg.hybrid_bias_z > 0 and healthy_ref:
                    bias_r.append(float(z[0] - h[0]))
                    bias_v.append(float(s2[0]))
                    if len(bias_r) >= cfg.hybrid_bias_min:
                        zb = abs(np.mean(bias_r)) / np.sqrt(np.mean(bias_v) * n_eff_f / len(bias_r))
                        if zb > cfg.hybrid_bias_z:
                            bias_bad = True
                            bias_latched = cfg.hybrid_bias_latch
                        elif zb < 0.5 * cfg.hybrid_bias_z and not bias_latched:
                            bias_bad = False
                out_t = 0.0 if tau < 1.5 else tau
                vf = cal.effective_var(s2[:1], out_t)
                if cfg.corr_scale is not None:
                    vf = vf * float(cfg.corr_scale[0])
                zz = np.array([z[0], 0.0, 0.0])
                vv = np.array([vf[0], cfg.nhc_sigma**2, cfg.nhc_sigma**2])
                sel = [0, 1, 2]
                if np.sqrt(vf[0]) > cfg.hybrid_max_std or float(np.atleast_1d(cal.kappa)[0]) > cfg.hybrid_max_kappa \
                        or bias_bad:
                    sel = [1, 2]
                    n_fallback += 1
                A.update(zz[sel] - h[sel], H[sel], np.diag(vv[sel]))
                if B is not None:
                    hB, HB = bvm(B, (0, 1, 2))
                    B.update(zz[sel] - hB[sel], HB[sel], np.diag(vv[sel]))
            elif ok:
                h, H = bvm(A, chans)
                healthy_ref = (tau < 1.5) and (t > 30.0) and (alarm_time is None)
                if cfg.aci and healthy_ref:
                    c = np.einsum("ij,jk,ik->i", H, A.P, H)
                    cal.observe(z - h, s2, c)
                out_t = 0.0 if tau < 1.5 else tau
                var_eff = cal.effective_var(s2, out_t)
                if cfg.corr_scale is not None and cfg.pseudo == "learned_vel":
                    var_eff = var_eff * np.asarray(cfg.corr_scale, float)
                A.update(z - h, H, np.diag(var_eff))
                if B is not None:
                    hB, HB = bvm(B, chans)
                    B.update(z - hB, HB, np.diag(var_eff))

        # ---------- GNSS ----------
        if kk in gnss_at:
            m = gnss_at[kk]
            if gnss.valid[m]:
                zg = gnss.pos[m]
                if cfg.use_epoch_sigma and gnss.sigma_ep is not None:
                    Rm = np.diag(np.maximum(gnss.sigma_ep[m], cfg.gnss_sigma_floor) ** 2)
                slow_fix = False
                if detector is not None and cfg.zupt_require_gnss_stop:
                    if previous_fix is not None:
                        tp, zp, Rp = previous_fix
                        gap = t - tp
                        uncertainty = 2*np.sqrt(np.linalg.eigvalsh(Rp + Rm).max())
                        slow_fix = (0 < gap <= 2.0 and
                                    (np.linalg.norm(zg-zp) + uncertainty)/gap < .25)
                    # Even a filter-rejected moving fix must revoke a stop latch:
                    # otherwise a false ZUPT could gate out evidence of motion.
                    if not slow_fix:
                        stop_confirmations = 0
                        stop_verified = False
                    previous_fix = (t, zg.copy(), Rm.copy())
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
                    Rm_eff = Rm if adaptive is None else adaptive.update(zg - h, H, A.P)
                    if coupler is None or t < cfg.baseline_range_start_s:
                        info = A.update_gnss(zg, lever, Rm_eff, gate=cfg.gnss_gate, force=force,
                                             robust_delta=cfg.robust_gnss_delta)
                    else:
                        info = coupler.update(A, zg, lever, Rm_eff,
                                              gate=float(chi2.ppf(0.999, 1)), force=force)
                        range_trace.append((float(t), coupler.last_residual, info.nis))
                        if coupler.monitor.alarm and range_alarm_time is None:
                            range_alarm_time = float(t)
                    if info.accepted:
                        if detector is not None and cfg.zupt_require_gnss_stop:
                            stop_confirmations = stop_confirmations + 1 if slow_fix else 0
                            stop_verified = stop_confirmations >= 2
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
            logs["vb"].append(A.R.T @ A.v)
            logs["tg"].append(tau)

    rt = (time.perf_counter() - t0) / (n - 1) * 1e6
    return RunLog(
        t=np.array(logs["t"]), p_est=np.array(logs["pe"]), p_true=np.array(logs["pt"]),
        P_pos=np.array(logs["P"]), v_est=np.array(logs["ve"]), v_true=np.array(logs["vt"]),
        gnss_t=np.array(gq["t"]), stat_q=np.array(gq["q"]), stat_g=np.array(gq["g"]),
        alarm_time=alarm_time, kappa_hist=np.array(cal.history), runtime_per_step_us=rt,
        extra={"motion": motion, "range_alarm_time": range_alarm_time, "range_trace": range_trace,
               "fallback_frac": n_fallback / max(n_pseudo, 1), "vb_est": np.array(logs["vb"]),
               "since_gnss": np.array(logs["tg"])},
    )
