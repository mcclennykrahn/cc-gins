"""Real-time streaming wrapper around the existing GNSS/INS filters.

The batch pipeline (`ccgins.pipeline.run`) propagates a whole IMU log and applies GNSS
updates at known indices.  `StreamingNavigator` exposes the same estimator as an
incremental interface:

    nav.push_imu(t, acc, gyro)   # IMU sample at time t
    nav.push_gnss(t, pos, cov)   # GNSS position measurement at time t
    state = nav.get_state()      # position / velocity / attitude / covariance

IMU samples are propagated when the next sample arrives (the previous sample is held back
so the interval is known).  GNSS measurements may arrive out of order: they are buffered
in timestamp order and applied when the filter reaches their epoch, splitting the IMU
interval at the measurement time when necessary.  This reproduces the batch ordering
(propagate, then update at the GNSS epoch) for measurements on or between IMU epochs.

Only the IMU + GNSS estimator is wrapped; the batch-only motion pseudo-measurements,
ACI calibration, integrity monitoring and adaptive-R estimation are not.
"""
from __future__ import annotations

import bisect
from dataclasses import dataclass

import numpy as np
from scipy.stats import chi2

from .filters import NoiseParams, make_filter

_EPS = 1e-9


@dataclass(frozen=True)
class StreamState:
    """Filter state at the latest processed epoch (arrays are copies)."""
    t: float
    position: np.ndarray             # (3,) nav-frame ENU position, m
    velocity: np.ndarray             # (3,) nav-frame ENU velocity, m/s
    attitude: np.ndarray             # (3,3) body -> nav rotation
    covariance: np.ndarray           # (15,15) filter covariance
    position_covariance: np.ndarray  # (3,3) nav-frame position error covariance
    gyro_bias: np.ndarray            # (3,) rad/s
    accel_bias: np.ndarray           # (3,) m/s^2


class StreamingNavigator:
    """Incremental GNSS/INS navigator reusing `ccgins.filters.make_filter`.

    Parameters mirror the state initialization of `ccgins.pipeline.run`; use
    `StreamingNavigator.from_truth` to build one with the same initial state and noise
    model as a batch run.
    """

    def __init__(self, R, v, p, P0, noise: NoiseParams, kind: str = "inekf",
                 bg=None, ba=None, lever_arm=None, gate: float | None = None,
                 max_consecutive_rejects: int = 5):
        self.filter = make_filter(kind, R, v, p, P0, noise, bg=bg, ba=ba)
        self.lever_arm = np.zeros(3) if lever_arm is None else np.asarray(lever_arm, float)
        self.gate = float(chi2.ppf(0.999, 3)) if gate is None else gate
        self.max_consecutive_rejects = int(max_consecutive_rejects)

        self.t: float | None = None
        self._last_imu: tuple[float, np.ndarray, np.ndarray] | None = None
        self._gnss: list[tuple[float, int, np.ndarray, np.ndarray]] = []
        self._seq = 0
        self._rejects = 0

    # ------------------------------------------------------------------ factories
    @classmethod
    def from_truth(cls, truth, imu_spec, gnss_spec, cfg=None) -> "StreamingNavigator":
        """Build a navigator with the batch pipeline's initial state/noise for `truth`.

        `cfg` is a `ccgins.pipeline.Config`; `init_seed`, `p0_bias`, `filter`,
        `gnss_gate` and `max_consecutive_rejects` are honoured.  Batch-only options
        (pseudo-measurements, ACI, monitoring, adaptive R) are not used.
        """
        from .pipeline import Config, initial_state

        cfg = Config() if cfg is None else cfg
        R0, v0, p0, P0 = initial_state(truth, cfg.init_seed, cfg.p0_bias)
        noise = NoiseParams.from_spec(imu_spec)
        return cls(R0, v0, p0, P0, noise, kind=cfg.filter,
                   lever_arm=getattr(gnss_spec, "lever_arm", None),
                   gate=cfg.gnss_gate,
                   max_consecutive_rejects=cfg.max_consecutive_rejects)

    # ------------------------------------------------------------------ streaming API
    def push_imu(self, t: float, acc, gyro) -> None:
        """Push an IMU sample valid from time `t` until the next sample.

        The sample is propagated once the next timestamp is known, so no integration
        step is taken when the first sample arrives.
        """
        t = float(t)
        acc = np.asarray(acc, float).reshape(3)
        gyro = np.asarray(gyro, float).reshape(3)

        if self.t is None:
            self.t = t
            self._last_imu = (t, acc, gyro)
            self._apply_due_gnss(t)
            return

        if t <= self.t + _EPS:
            raise ValueError(f"IMU timestamps must increase: got {t} after {self.t}")

        prev_t, prev_acc, prev_gyro = self._last_imu
        # Walk the buffered GNSS epochs in time order, splitting the IMU interval so each
        # update is applied exactly at its own timestamp (batch-equivalent on-grid case).
        while self._gnss and self._gnss[0][0] <= t + _EPS:
            g_t = self._gnss[0][0]
            if g_t > prev_t + _EPS:
                self.filter.propagate(prev_gyro, prev_acc, g_t - prev_t)
                prev_t = g_t
                self.t = g_t
            self._pop_and_apply_gnss()
        if t > prev_t + _EPS:
            self.filter.propagate(prev_gyro, prev_acc, t - prev_t)
        self.t = t
        self._last_imu = (t, acc, gyro)

    def push_gnss(self, t: float, pos, cov) -> None:
        """Push a GNSS position measurement (antenna position) with covariance `cov`.

        Measurements may arrive in any order.  Future measurements are buffered and
        applied in timestamp order when the filter reaches them.  A measurement at the
        current epoch is applied immediately; an epoch that the filter has already moved
        past cannot be reconstructed and raises `ValueError`.
        """
        t = float(t)
        pos = np.asarray(pos, float).reshape(3)
        cov = self._as_cov(cov)

        if self.t is None:
            self._insert_gnss(t, pos, cov)
            return
        if t < self.t - _EPS:
            raise ValueError(
                f"GNSS epoch {t} is older than the current filter epoch {self.t}; "
                "late measurements cannot be applied retroactively")
        if t <= self.t + _EPS:
            self._apply_gnss(pos, cov)
            return
        self._insert_gnss(t, pos, cov)

    def get_state(self) -> StreamState:
        """Return the current filter state (safe copies)."""
        if self.t is None:
            raise RuntimeError("no IMU sample has been pushed yet")
        f = self.filter
        return StreamState(
            t=float(self.t),
            position=f.p.copy(),
            velocity=f.v.copy(),
            attitude=f.R.copy(),
            covariance=f.P.copy(),
            position_covariance=f.pos_cov(),
            gyro_bias=f.bg.copy(),
            accel_bias=f.ba.copy(),
        )

    # ------------------------------------------------------------------ internals
    def _insert_gnss(self, t: float, pos: np.ndarray, cov: np.ndarray) -> None:
        # seq keeps the arrival order stable for identical timestamps
        bisect.insort(self._gnss, (t, self._seq, pos, cov), key=lambda item: (item[0], item[1]))
        self._seq += 1

    def _apply_due_gnss(self, t: float) -> None:
        while self._gnss and self._gnss[0][0] <= t + _EPS:
            self._pop_and_apply_gnss()

    def _pop_and_apply_gnss(self) -> None:
        _, _, pos, cov = self._gnss.pop(0)
        self._apply_gnss(pos, cov)

    def _apply_gnss(self, pos: np.ndarray, cov: np.ndarray) -> None:
        h, H = self.filter.gnss_model(self.lever_arm)
        force = self._rejects >= self.max_consecutive_rejects
        info = self.filter.update(pos - h, H, cov, gate=self.gate, force=force)
        if info.accepted:
            self._rejects = 0
        else:
            self._rejects += 1

    @staticmethod
    def _as_cov(cov) -> np.ndarray:
        c = np.asarray(cov, float)
        if c.ndim == 0:
            return np.eye(3) * float(c)
        if c.ndim == 1:
            return np.diag(c.reshape(3))
        return c.reshape(3, 3).copy()
