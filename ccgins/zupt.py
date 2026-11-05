"""Zero-velocity detection and ZUPT / odometry velocity measurement updates.

The detector thresholds the sliding-window variance of the raw gyro and accelerometer samples:
a window is labelled stationary when both variances stay below their thresholds.  The update
helpers follow the generic ``NavFilter.update(r, H, Rm, gate, force)`` convention used by
``ccgins.filters`` so they work with both the ESKF and the InEKF.
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
    """Scalar wheel/odometry speed update in the body frame (default: forward channel)."""
    return body_velocity_update(nav, [speed], sigma, channels=(channel,), gate=gate, force=force)
