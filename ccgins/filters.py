"""Strapdown mechanization + ESKF and right-invariant EKF (InEKF) for loosely coupled GNSS/INS.

Error-state ordering (15): [rot(3), vel(3), pos(3), gyro bias(3), accel bias(3)].
  ESKF : R = R_hat Exp(dtheta), v = v_hat + dv, p = p_hat + dp           (body-frame attitude error)
  InEKF: X = Exp_SE23(xi) X_hat (right-invariant), biases additive ("imperfect" InEKF)
"""
from __future__ import annotations

import copy
from dataclasses import dataclass

import numpy as np

from .geometry import skew, so3_exp, so3_left_jacobian
from .sim import G_NAV

I3 = np.eye(3)


# Optional earth model (real data). Default: earth rate neglected, g = G_NAV (simulation / KITTI
# results are unchanged). For real MEMS data set_earth(w_ie_enu, g_local) enables a fixed local-level
# ENU frame with earth rotation (attitude transport + Coriolis) and local normal gravity; transport
# rate is neglected (local tangent frame, short drives).
EARTH = {"w_ie": None, "g": G_NAV.copy()}


def set_earth(w_ie_enu=None, g_local=None):
    EARTH["w_ie"] = None if w_ie_enu is None else np.asarray(w_ie_enu, float)
    EARTH["g"] = G_NAV.copy() if g_local is None else np.array([0.0, 0.0, -float(g_local)])


def mechanize(R, v, p, gyro, accel, dt, earth=None):
    """Single discrete strapdown step (nav = local-level ENU; earth rate optional via set_earth)."""
    earth = EARTH if earth is None else earth
    w = earth["w_ie"]
    a = R @ accel + earth["g"]
    if w is not None:
        a = a - 2.0 * np.cross(w, v)
    p_new = p + v * dt + 0.5 * a * dt * dt
    v_new = v + a * dt
    R_new = R @ so3_exp(gyro * dt)
    if w is not None:
        R_new = so3_exp(-w * dt) @ R_new
    return R_new, v_new, p_new


@dataclass
class NoiseParams:
    gyro: float          # rad/s/sqrt(Hz)
    accel: float         # m/s^2/sqrt(Hz)
    gyro_bias_rw: float  # rad/s^2/sqrt(Hz)
    accel_bias_rw: float # m/s^3/sqrt(Hz)

    @classmethod
    def from_spec(cls, spec, vmax_eff: float = 12.0, dt: float = 0.01):
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError("positive finite IMU dt required")
        d = spec.densities()
        # simulate_imu adds base and vibration sample standard deviations.
        # Convert that same sum to a density at the nominal speed and actual dt.
        # vibration (white, speed dependent) is folded into the white-noise density at a nominal speed
        vib_a = spec.vib_accel_per_mps * vmax_eff * np.sqrt(dt)
        vib_g = spec.vib_gyro_per_mps * np.pi / 180 * vmax_eff * np.sqrt(dt)
        return cls(gyro=float(d["gyro"] + vib_g), accel=float(d["accel"] + vib_a),
                   gyro_bias_rw=d["gyro_bias_rw"], accel_bias_rw=d["accel_bias_rw"])


@dataclass
class UpdateInfo:
    nis: float
    accepted: bool
    dof: int


class NavFilter:
    kind = "base"

    def __init__(self, R, v, p, P0, noise: NoiseParams, bg=None, ba=None, earth=None):
        self.R = np.array(R, float)
        self.v = np.array(v, float)
        self.p = np.array(p, float)
        self.bg = np.zeros(3) if bg is None else np.array(bg, float)
        self.ba = np.zeros(3) if ba is None else np.array(ba, float)
        self.P = np.array(P0, float)
        self.noise = noise
        self.earth = copy.deepcopy(EARTH if earth is None else earth)

    def copy(self):
        return copy.deepcopy(self)

    # ---------------- propagation ----------------
    def propagate(self, gyro_m, accel_m, dt):
        w = gyro_m - self.bg
        f = accel_m - self.ba
        F, G = self._jacobians(w, f)
        Phi = np.eye(15) + F * dt
        Qc = np.diag(np.r_[np.full(3, self.noise.gyro**2), np.full(3, self.noise.accel**2),
                           np.full(3, self.noise.gyro_bias_rw**2), np.full(3, self.noise.accel_bias_rw**2)])
        Qd = G @ Qc @ G.T * dt
        self.P = Phi @ self.P @ Phi.T + Qd
        self.R, self.v, self.p = mechanize(self.R, self.v, self.p, w, f, dt, earth=self.earth)

    # ---------------- generic update ----------------
    def update(self, r, H, Rm, gate: float | None = None, force: bool = False) -> UpdateInfo:
        S = H @ self.P @ H.T + Rm
        Sinv = np.linalg.inv(S)
        nis = float(r @ Sinv @ r)
        if gate is not None and nis > gate and not force:
            return UpdateInfo(nis, False, len(r))
        K = self.P @ H.T @ Sinv
        dx = K @ r
        IKH = np.eye(15) - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ Rm @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        self._inject(dx)
        return UpdateInfo(nis, True, len(r))

    # ---------------- measurement models ----------------
    def gnss_model(self, lever):
        h = self.p + self.R @ lever
        return h, self._H_gnss(lever)

    def update_gnss(self, z, lever, Rm, gate=None, force=False, robust_delta=None):
        """Optional Huber weights on whitened innovations, with covariance inflation.

        Squared inverse weights bound influence; NIS and gates use original R.
        """
        if robust_delta is not None and (not np.isfinite(robust_delta) or robust_delta <= 0):
            raise ValueError("robust_delta must be finite and positive")
        h, H = self.gnss_model(lever)
        r = np.asarray(z, float) - h
        S = H @ self.P @ H.T + Rm
        nis = float(r @ np.linalg.solve(S, r))
        if gate is not None and nis > gate and not force:
            return UpdateInfo(nis, False, 3)
        if robust_delta is not None:
            L = np.linalg.cholesky(S)
            u = np.linalg.solve(L, r)
            weights = np.minimum(1.0, robust_delta / np.maximum(np.abs(u), 1e-15))
            D = L @ np.diag(1.0 / weights) @ np.linalg.inv(L)
            Rm = D @ Rm @ D.T
        info = self.update(r, H, Rm)
        return UpdateInfo(nis, info.accepted, 3)

    def body_vel_model(self, channels=(0, 1, 2)):
        vb = self.R.T @ self.v
        H = self._H_body_vel()
        ch = list(channels)
        return vb[ch], H[ch]

    def pos_cov(self):
        """Covariance of the nav-frame position error p - p_hat."""
        J = self._J_pos()
        return J @ self.P @ J.T

    # to be implemented by subclasses
    def _jacobians(self, w, f):
        raise NotImplementedError

    def _inject(self, dx):
        raise NotImplementedError

    def _H_gnss(self, lever):
        raise NotImplementedError

    def _H_body_vel(self):
        raise NotImplementedError

    def _J_pos(self):
        raise NotImplementedError


class ESKF(NavFilter):
    kind = "eskf"

    def _jacobians(self, w, f):
        F = np.zeros((15, 15))
        F[0:3, 0:3] = -skew(w)
        F[0:3, 9:12] = -I3
        F[3:6, 0:3] = -self.R @ skew(f)
        if self.earth["w_ie"] is not None:
            F[3:6, 3:6] = -2 * skew(self.earth["w_ie"])
        F[3:6, 12:15] = -self.R
        F[6:9, 3:6] = I3
        G = np.zeros((15, 12))
        G[0:3, 0:3] = -I3
        G[3:6, 3:6] = -self.R
        G[9:12, 6:9] = I3
        G[12:15, 9:12] = I3
        return F, G

    def _inject(self, dx):
        self.R = self.R @ so3_exp(dx[0:3])
        self.v = self.v + dx[3:6]
        self.p = self.p + dx[6:9]
        self.bg = self.bg + dx[9:12]
        self.ba = self.ba + dx[12:15]

    def _H_gnss(self, lever):
        H = np.zeros((3, 15))
        H[:, 0:3] = -self.R @ skew(lever)
        H[:, 6:9] = I3
        return H

    def _H_body_vel(self):
        H = np.zeros((3, 15))
        H[:, 0:3] = skew(self.R.T @ self.v)
        H[:, 3:6] = self.R.T
        return H

    def _J_pos(self):
        J = np.zeros((3, 15))
        J[:, 6:9] = I3
        return J


class InEKF(NavFilter):
    """Right-invariant EKF on SE_2(3) with additive IMU biases."""
    kind = "inekf"

    def _jacobians(self, w, f):
        R, v, p = self.R, self.v, self.p
        F = np.zeros((15, 15))
        F[3:6, 0:3] = skew(self.earth["g"])
        F[6:9, 3:6] = I3
        if self.earth["w_ie"] is not None:
            W = skew(self.earth["w_ie"])
            F[0:3, 0:3] = -W
            F[3:6, 0:3] += skew(v) @ W
            F[3:6, 3:6] = -2 * W
            F[6:9, 0:3] = -skew(p) @ W
        F[0:3, 9:12] = -R
        F[3:6, 9:12] = -skew(v) @ R
        F[6:9, 9:12] = -skew(p) @ R
        F[3:6, 12:15] = -R
        G = np.zeros((15, 12))
        G[0:3, 0:3] = -R
        G[3:6, 0:3] = -skew(v) @ R
        G[6:9, 0:3] = -skew(p) @ R
        G[3:6, 3:6] = -R
        G[9:12, 6:9] = I3
        G[12:15, 9:12] = I3
        return F, G

    def _inject(self, dx):
        phi = dx[0:3]
        dR = so3_exp(phi)
        Jl = so3_left_jacobian(phi)
        self.R = dR @ self.R
        self.v = dR @ self.v + Jl @ dx[3:6]
        self.p = dR @ self.p + Jl @ dx[6:9]
        self.bg = self.bg + dx[9:12]
        self.ba = self.ba + dx[12:15]

    def _H_gnss(self, lever):
        H = np.zeros((3, 15))
        H[:, 0:3] = -skew(self.p + self.R @ lever)
        H[:, 6:9] = I3
        return H

    def _H_body_vel(self):
        H = np.zeros((3, 15))
        H[:, 3:6] = self.R.T
        return H

    def _J_pos(self):
        J = np.zeros((3, 15))
        J[:, 0:3] = -skew(self.p)
        J[:, 6:9] = I3
        return J


FILTERS = {"eskf": ESKF, "inekf": InEKF}


def make_filter(kind, R, v, p, P0_nav, noise, bg=None, ba=None, earth=None):
    """P0_nav: 15x15 covariance expressed for nav-frame errors [att(nav), v, p, bg, ba].
    Converted to each filter's error definition."""
    cls = FILTERS[kind]
    P0 = np.array(P0_nav, float)
    if kind == "eskf":
        # body-frame attitude error: dtheta_b = R^T dtheta_n
        T = np.eye(15)
        T[0:3, 0:3] = np.asarray(R).T
        P0 = T @ P0 @ T.T
    else:
        # xi_R = dtheta_n, xi_v = dv - dtheta x v = dv + v^ dtheta, xi_p = dp + p^ dtheta
        T = np.eye(15)
        T[3:6, 0:3] = skew(v)
        T[6:9, 0:3] = skew(p)
        P0 = T @ P0 @ T.T
    return cls(R, v, p, P0, noise, bg, ba, earth=earth)
