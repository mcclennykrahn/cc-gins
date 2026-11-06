"""Zero-velocity detection and ZUPT / odometry velocity measurement updates.

AdaptiveZeroVelocity is the causal, noise/vibration-calibrated main-pipeline detector.
The centered fixed-threshold detect_zero_velocity helper is retained for compatibility
and is not used in the main pipeline. OdometryData requires explicit wheel geometry;
mounted_odometry_model supplies the ESKF/InEKF Jacobians at the wheel measurement point.
"""
from __future__ import annotations

import numpy as np

# Defaults are tuned for MEMS-grade IMUs at 100 Hz, window = 50 samples (0.5 s).  They are
# expressed as variances, i.e. (std threshold)^2.
DEFAULT_WINDOW = 50
DEFAULT_ACCEL_VAR_THRESHOLD = 0.10**2    # (m/s^2)^2
DEFAULT_GYRO_VAR_THRESHOLD = 0.05**2     # (rad/s)^2


def sliding_variance(x, window: int):
    """Centered, edge-padded sliding population variance along axis 0.

    x: (N,) or (N, C).  Returns the same shape as x.  The window is clipped to N samples.
    """
    x = np.asarray(x, float)
    if x.ndim == 1:
        var = sliding_variance(x[:, None], window)
        return var[:, 0]
    n = len(x)
    if n == 0:
        return x.copy()
    w = max(1, min(int(window), n))
    before = w // 2
    after = w - 1 - before
    xp = np.pad(x, ((before, after), (0, 0)), mode="edge")
    zero = np.zeros((1, x.shape[1]))
    c1 = np.vstack([zero, np.cumsum(xp, axis=0)])
    c2 = np.vstack([zero, np.cumsum(xp * xp, axis=0)])
    s1 = c1[w:] - c1[:-w]
    s2 = c2[w:] - c2[:-w]
    mean = s1 / w
    return np.maximum(s2 / w - mean * mean, 0.0)


def detect_zero_velocity(gyro, accel, window: int = DEFAULT_WINDOW,
                         accel_var_threshold: float = DEFAULT_ACCEL_VAR_THRESHOLD,
                         gyro_var_threshold: float = DEFAULT_GYRO_VAR_THRESHOLD,
                         min_duration: int = 1, return_variance: bool = False):
    """Label IMU samples as stationary from sliding-window gyro/accel variance.

    gyro, accel: (N, 3) arrays in rad/s and m/s^2.  ``window`` and ``min_duration`` are in
    samples.  Runs shorter than ``min_duration`` are discarded to suppress isolated detections.

    Returns a boolean (N,) mask, or ``(mask, accel_var, gyro_var)`` when ``return_variance``.
    """
    gyro = np.asarray(gyro, float)
    accel = np.asarray(accel, float)
    if gyro.shape != accel.shape or gyro.ndim != 2 or gyro.shape[1] != 3:
        raise ValueError("gyro and accel must both have shape (N, 3)")
    # Per-axis variance; requiring the worst axis to be quiet avoids masking rotation about one axis.
    va = np.max(sliding_variance(accel, window), axis=1)
    vg = np.max(sliding_variance(gyro, window), axis=1)
    detected = (va < accel_var_threshold) & (vg < gyro_var_threshold)

    min_duration = max(1, int(min_duration))
    if min_duration > 1 and detected.any():
        # Remove short True runs (and short False runs inside them are left untouched).
        edges = np.diff(np.r_[False, detected, False].astype(np.int8))
        starts = np.flatnonzero(edges == 1)
        stops = np.flatnonzero(edges == -1)
        for s, e in zip(starts, stops):
            if e - s < min_duration:
                detected[s:e] = False

    if return_variance:
        return detected, va, vg
    return detected


def velocity_update(nav, z_nav, sigma, gate: float | None = None, force: bool = False):
    """Update with a nav-frame velocity measurement ``z_nav`` (m/s), sigma scalar or (3,)."""
    z = np.asarray(z_nav, float).reshape(3)
    sigma = np.asarray(sigma, float)
    Rm = np.diag(np.full(3, float(sigma) ** 2)) if sigma.ndim == 0 else np.diag(sigma ** 2)
    H = np.zeros((3, 15))
    H[:, 3:6] = np.eye(3)
    if nav.kind == "inekf":
        from .geometry import skew
        H[:, :3] = -skew(nav.v)
    return nav.update(z - nav.v, H, Rm, gate=gate, force=force)


def zupt_update(nav, sigma: float = 0.01, gate: float | None = None, force: bool = False):
    """Zero-velocity update: constrain the nav-frame velocity to zero."""
    return velocity_update(nav, np.zeros(3), sigma, gate=gate, force=force)


def body_velocity_update(nav, z_body, sigma, channels=(0, 1, 2), gate: float | None = None,
                         force: bool = False):
    """Update with body-frame velocity measurements on ``channels`` (FLU: 0=fwd, 1=left, 2=up)."""
    channels = tuple(channels)
    z = np.asarray(z_body, float).reshape(-1)
    if z.size != len(channels):
        raise ValueError("z_body must have one entry per channel")
    h, H = nav.body_vel_model(channels)
    sigma = np.asarray(sigma, float)
    Rm = np.diag(np.full(len(channels), float(sigma) ** 2)) if sigma.ndim == 0 else np.diag(sigma ** 2)
    return nav.update(z - h, H, Rm, gate=gate, force=force)


def odometry_update(nav, speed: float, sigma: float = 0.1, channel: int = 0,
                    gate: float | None = None, force: bool = False):
    """Legacy scalar speed at IMU centre, aligned with body; no lever/slip correction.

    Use mounted_odometry_update and explicit calibrated geometry for wheel sensors.
    """
    return body_velocity_update(nav, [speed], sigma, channels=(channel,), gate=gate, force=force)


class AdaptiveZeroVelocity:
    """Causal trailing-window detector with frozen, sensor-specific calibration.

    Calibration uses only IMU samples, preferably a disjoint recording. The lower
    decile of one-second variance windows estimates the quiet vibration floor;
    noise density supplies a lower bound and physical caps prevent learning motion
    as rest. No IMU-only detector can distinguish smooth translation from rest:
    callers MUST also supply an independent/conservative speed veto.
    """

    def __init__(self, accel_var, gyro_var, dt=0.01, window_s=0.5, dwell_s=1.0,
                 gravity=9.81):
        from collections import deque
        if min(accel_var, gyro_var, dt, window_s, dwell_s, gravity) <= 0 or not np.isfinite(
                [accel_var, gyro_var, dt, window_s, dwell_s, gravity]).all():
            raise ValueError("detector parameters must be finite and positive")
        self.accel_var, self.gyro_var = float(accel_var), float(gyro_var)
        self.window = max(2, int(round(window_s / dt)))
        self.dwell = max(1, int(round(dwell_s / dt)))
        self.gravity = gravity
        self.a = deque(maxlen=self.window)
        self.g = deque(maxlen=self.window)
        self.quiet = 0

    @classmethod
    def calibrate(cls, gyro, accel, spec, dt=0.01, accel_std_cap=3.0,
                  gyro_std_cap=0.05, **kw):
        if min(accel_std_cap, gyro_std_cap, dt) <= 0 or not np.isfinite(
                [accel_std_cap, gyro_std_cap, dt]).all():
            raise ValueError("calibration caps and dt must be finite and positive")
        gyro, accel = np.asarray(gyro), np.asarray(accel)
        if gyro.shape != accel.shape or gyro.ndim != 2 or gyro.shape[1] != 3:
            raise ValueError("calibration arrays must have shape (N, 3)")
        w = max(2, int(round(1 / dt)))
        n = len(gyro) // w
        if n < 10 or not np.isfinite(gyro).all() or not np.isfinite(accel).all():
            raise ValueError("need at least ten finite one-second calibration windows")
        av = np.max(accel[:n*w].reshape(n, w, 3).var(axis=1), axis=1)
        gv = np.max(gyro[:n*w].reshape(n, w, 3).var(axis=1), axis=1)
        d = spec.densities()
        # 4x variance margin for vibration variation, bounded at physical maxima.
        va = min(accel_std_cap**2, 4 * max(float(np.quantile(av, .1)), d['accel']**2 / dt))
        vg = min(gyro_std_cap**2, 4 * max(float(np.quantile(gv, .1)), d['gyro']**2 / dt))
        return cls(va, vg, dt=dt, **kw)

    def step(self, gyro, accel, speed):
        g, a = np.asarray(gyro, float), np.asarray(accel, float)
        if g.shape != (3,) or a.shape != (3,):
            raise ValueError("IMU samples must have shape (3,)")
        if not np.isfinite(np.r_[g, a, speed]).all():
            self.a.clear(); self.g.clear(); self.quiet = 0
            return False
        self.g.append(g.copy()); self.a.append(a.copy())
        if len(self.a) < self.window:
            return False
        aa, gg = np.array(self.a), np.array(self.g)
        quiet = (np.max(aa.var(axis=0)) < self.accel_var
                 and np.max(gg.var(axis=0)) < self.gyro_var
                 and np.linalg.norm(gg.mean(axis=0)) < 0.02
                 and abs(np.linalg.norm(aa.mean(axis=0)) - self.gravity) < 0.25
                 and abs(speed) < 0.5)
        self.quiet = self.quiet + 1 if quiet else 0
        return self.quiet >= self.dwell


class OdometryData:
    """Timestamped signed vehicle-forward speed, never interpolated or held.

    C_vb maps IMU FLU to vehicle FLU; lever_b is IMU-to-wheel measurement
    point in IMU FLU metres. Explicit geometry is required, including identity/zero.
    Invalid/slipping epochs can be excluded with valid. No slip estimator is implied.
    """
    def __init__(self, t, speed, sigma, C_vb, lever_b, valid=None):
        self.t, self.speed = np.asarray(t, float), np.asarray(speed, float)
        self.sigma = np.broadcast_to(np.asarray(sigma, float), self.t.shape).copy()
        self.C_vb, self.lever_b = np.asarray(C_vb, float), np.asarray(lever_b, float)
        self.valid = np.ones(self.t.shape, bool) if valid is None else np.asarray(valid, bool)
        if (self.t.ndim != 1 or self.speed.shape != self.t.shape or self.valid.shape != self.t.shape
                or not np.isfinite(np.r_[self.t, self.speed, self.sigma]).all()
                or np.any(self.sigma <= 0) or np.any(np.diff(self.t) <= 0)):
            raise ValueError("odometry must have ordered finite timestamps/speeds and positive sigma")
        if (self.C_vb.shape != (3, 3) or self.lever_b.shape != (3,)
                or not np.isfinite(self.lever_b).all()
                or not np.allclose(self.C_vb @ self.C_vb.T, np.eye(3), atol=1e-8)
                or not np.isclose(np.linalg.det(self.C_vb), 1)):
            raise ValueError("explicit proper mounting rotation and finite lever arm required")


def mounted_odometry_model(nav, gyro, C_vb, lever_b):
    """Wheel point speed: C_vb (R.T v + (gyro-bg-R.T earth_rate) x lever)."""
    from .geometry import skew
    h, H = nav.body_vel_model()
    w_ie = nav.earth['w_ie']
    wb = np.zeros(3) if w_ie is None else nav.R.T @ w_ie
    h = h + np.cross(np.asarray(gyro) - nav.bg - wb, lever_b)
    H[:, 9:12] += skew(lever_b)
    if w_ie is not None:
        H[:, :3] += skew(lever_b) @ skew(wb) @ (nav.R.T if nav.kind == 'inekf' else np.eye(3))
    return (C_vb @ h)[:1], (C_vb @ H)[:1]


def mounted_odometry_update(nav, speed, sigma, gyro, C_vb, lever_b, gate=10.827566):
    h, H = mounted_odometry_model(nav, gyro, C_vb, lever_b)
    return nav.update(np.array([speed]) - h, H, np.array([[sigma**2]]), gate=gate)


class OdometryStopGuard:
    """Two accepted wheel-zero upper bounds, <=0.2 s apart; never held indefinitely.

    Forward zero alone cannot exclude lateral motion/slip. Enable only for a
    calibrated ground vehicle with independently valid wheel data.
    """
    def __init__(self):
        self.last = -np.inf
        self.count = 0

    def observe(self, t, speed, sigma, accepted):
        zero = accepted and abs(speed) + 3*sigma < .1
        self.count = (self.count + 1 if 0 < t-self.last <= .200000001 else 1) if zero else 0
        self.last = t

    def confirmed(self, t):
        return self.count >= 2 and 0 <= t-self.last <= .200000001
