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
    # IMU-to-vehicle mounting (real data): (pitch, yaw) [rad] of the vehicle forward axis seen in the
    # IMU body frame. NHC / hybrid lateral-vertical constraints are applied in the VEHICLE frame
    # v_veh = C_vb v_body. None = IMU assumed aligned with the vehicle (simulation / KITTI).
    mount_py: tuple | None = None
    p0_bias: tuple | None = None      # (gyro rad/s, accel m/s^2) initial bias std; None = legacy
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


def run(truth, imu, gnss, imu_spec, gnss_spec, cfg: Config, pred=None) -> RunLog:
    """pred: (idx, mu, var) learned predictions at pseudo epochs (idx = IMU end index)."""
    noise = NoiseParams.from_spec(imu_spec)
    C_vb = None if cfg.mount_py is None else mount_matrix(*cfg.mount_py)

    def bvm(F, ch):
        h, H = F.body_vel_model((0, 1, 2))
        if C_vb is not None and cfg.pseudo in ("nhc", "hybrid", "learned_nhc"):
            h, H = C_vb @ h, C_vb @ H
        ch = list(ch)
        return h[ch], H[ch]
    R0, v0, p0, P0 = initial_state(truth, cfg.init_seed, cfg.p0_bias)
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
            logs["vb"].append(A.R.T @ A.v)
            logs["tg"].append(tau)

    rt = (time.perf_counter() - t0) / (n - 1) * 1e6
    return RunLog(
        t=np.array(logs["t"]), p_est=np.array(logs["pe"]), p_true=np.array(logs["pt"]),
        P_pos=np.array(logs["P"]), v_est=np.array(logs["ve"]), v_true=np.array(logs["vt"]),
        gnss_t=np.array(gq["t"]), stat_q=np.array(gq["q"]), stat_g=np.array(gq["g"]),
        alarm_time=alarm_time, kappa_hist=np.array(cal.history), runtime_per_step_us=rt,
        extra={"fallback_frac": n_fallback / max(n_pseudo, 1), "vb_est": np.array(logs["vb"]),
               "since_gnss": np.array(logs["tg"])},
    )
