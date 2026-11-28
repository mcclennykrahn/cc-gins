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

Adaptive causal ZUPT and mounted wheel updates share the batch detector/models/guards.
Fixed mounted NHC is supported; learned pseudo-measurements, ACI, monitoring and
adaptive-R are rejected by this baseline class. The separate opt-in
`ccgins.stream_learning.LearnedStreamingNavigator` supports learned motion/ACI/integrity.
"""
from __future__ import annotations

import bisect
import heapq
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
                 max_consecutive_rejects: int = 5, robust_gnss_delta=None, earth=None,
                 motion_cfg=None, imu_spec=None, dt=None):
        self.filter = make_filter(kind, R, v, p, P0, noise, bg=bg, ba=ba, earth=earth)
        self.lever_arm = np.zeros(3) if lever_arm is None else np.asarray(lever_arm, float)
        self.robust_gnss_delta = robust_gnss_delta
        self.gate = float(chi2.ppf(0.999, 3)) if gate is None else gate
        self.max_consecutive_rejects = int(max_consecutive_rejects)

        self.t: float | None = None
        self._last_imu: tuple[float, np.ndarray, np.ndarray] | None = None
        self._gnss: list[tuple[float, int, np.ndarray, np.ndarray]] = []
        self._seq = 0
        self._rejects = 0
        from .pipeline import Config
        from .zupt import AdaptiveZeroVelocity, OdometryStopGuard
        self.cfg = Config() if motion_cfg is None else motion_cfg
        if (self.cfg.pseudo not in ('none', 'nhc') or self.cfg.aci or
                self.cfg.monitor != 'none' or self.cfg.adaptive_r or self.cfg.gnss_aci or
                self.cfg.baseline_range_anchor is not None):
            raise ValueError("streaming supports none/fixed NHC, without ACI, monitoring, adaptive R or range coupling")
        self.cfg.nhc_variances()
        if self.cfg.pseudo_every < 1:
            raise ValueError("positive pseudo cadence required")
        from .pipeline import mount_matrix
        self.C_vb = None if self.cfg.mount_py is None else mount_matrix(*self.cfg.mount_py)
        self.detector = None
        if self.cfg.motion_every < 1 or not np.isfinite(self.cfg.zupt_sigma) or self.cfg.zupt_sigma <= 0:
            raise ValueError("invalid motion cadence/sigma")
        if self.cfg.zupt:
            if imu_spec is None or dt is None:
                raise ValueError("ZUPT requires imu_spec and fixed dt")
            d = imu_spec.densities()
            self.detector = AdaptiveZeroVelocity(
                self.cfg.zupt_accel_var if self.cfg.zupt_accel_var is not None else 4*d['accel']**2/dt,
                self.cfg.zupt_gyro_var if self.cfg.zupt_gyro_var is not None else 4*d['gyro']**2/dt,
                dt=dt, gravity=9.81 if self.cfg.local_gravity is None else self.cfg.local_gravity)
        self.dt = dt
        self._index = 0
        self._odometry = {}
        self._odo_epochs = []
        self._stop = False
        self._confirmations = 0
        self._previous_fix = None
        self._odo_stop = OdometryStopGuard()
        self.motion = dict(zupt_attempted=0, zupt_accepted=0, odometry_attempted=0,
                           odometry_accepted=0, zupt_times=[], unconfirmed_stop_vetoes=0)

    # ------------------------------------------------------------------ factories
    @classmethod
    def from_truth(cls, truth, imu_spec, gnss_spec, cfg=None) -> "StreamingNavigator":
        """Build a navigator with the batch pipeline's initial state/noise for `truth`.

        `cfg` is a `ccgins.pipeline.Config`; `init_seed`, `p0_bias`, `filter`,
        `gnss_gate`, `max_consecutive_rejects` and motion options are honoured.
        Fixed mounted NHC is supported; learned pseudo-measurements, ACI,
        monitoring and adaptive R remain batch-only and are rejected.
        Supply frozen held-out detector variances through cfg, as for batch.
        """
        from .pipeline import Config, initial_state

        cfg = Config() if cfg is None else cfg
        R0, v0, p0, P0 = initial_state(truth, cfg.init_seed, cfg.p0_bias)
        noise = NoiseParams.from_spec(imu_spec, dt=truth.dt)
        return cls(R0, v0, p0, P0, noise, kind=cfg.filter,
                   motion_cfg=cfg, imu_spec=imu_spec, dt=truth.dt,
                   lever_arm=getattr(gnss_spec, "lever_arm", None),
                   gate=cfg.gnss_gate,
                   max_consecutive_rejects=cfg.max_consecutive_rejects,
                   robust_gnss_delta=cfg.robust_gnss_delta,
                   earth=None if cfg.earth_rate_enu is None and cfg.local_gravity is None else
                   {"w_ie": None if cfg.earth_rate_enu is None else np.asarray(cfg.earth_rate_enu),
                    "g": np.array([0., 0., -(9.81 if cfg.local_gravity is None else cfg.local_gravity)])})

    # ------------------------------------------------------------------ streaming API
    def push_imu(self, t: float, acc, gyro) -> None:
        """Push an IMU sample valid from time `t` until the next sample.

        The sample is propagated once the next timestamp is known, so no integration
        step is taken when the first sample arrives.
        """
        t = float(t)
        acc = np.asarray(acc, float).reshape(3)
        gyro = np.asarray(gyro, float).reshape(3)

        if self._odo_epochs and self._odo_epochs[0] < t-_EPS:
            raise ValueError("odometry must align with propagated IMU epochs")
        if self.t is None and self._odo_epochs and self._odo_epochs[0] <= t+_EPS:
            raise ValueError("odometry requires a propagated IMU epoch")
        if self.t is None:
            self.t = t
            self._last_imu = (t, acc, gyro)
            self._apply_due_gnss(t)
            return

        if t <= self.t + _EPS:
            raise ValueError(f"IMU timestamps must increase: got {t} after {self.t}")

        prev_t, prev_acc, prev_gyro = self._last_imu
        if self.detector is not None and not np.isclose(t-prev_t, self.dt, atol=1e-9, rtol=0):
            raise ValueError("motion detector requires fixed-rate IMU")
        # Walk the buffered GNSS epochs in time order, splitting the IMU interval so each
        # update is applied exactly at its own timestamp (batch-equivalent on-grid case).
        while self._gnss and self._gnss[0][0] < t - _EPS:
            g_t = self._gnss[0][0]
            if g_t > prev_t + _EPS:
                self._propagate(prev_gyro, prev_acc, g_t - prev_t)
                prev_t = g_t
                self.t = g_t
            self._pop_and_apply_gnss()
        if t > prev_t + _EPS:
            interval = t - prev_t
            # Batch uses nominal truth.dt. Do not round a remainder after an
            # off-grid GNSS split: that intentionally uses actual subintervals.
            if (prev_t == self._last_imu[0] and self.dt is not None and
                    np.isclose(interval, self.dt, atol=1e-9, rtol=0)):
                interval = self.dt
            self._propagate(prev_gyro, prev_acc, interval)
        self.t = t
        self._last_imu = (t, acc, gyro)
        self._index += 1
        self._apply_motion(t, prev_gyro, prev_acc)
        from .learning import WINDOW
        if (self.cfg.pseudo == 'nhc' and self._index >= WINDOW and
                self._index % self.cfg.pseudo_every == 0):
            h, H = self.filter.body_vel_model((0, 1, 2))
            if self.C_vb is not None:
                h, H = self.C_vb @ h, self.C_vb @ H
            self.filter.update(-h[1:], H[1:], np.diag(self.cfg.nhc_variances()))
        self._apply_due_gnss(t)

    def push_odometry(self, t, speed, sigma, C_vb, lever_b, valid=True):
        """Queue explicit mounted wheel data before its IMU epoch; no holding/interpolation.

        Epochs must coincide with a future IMU sample. GNSS at that epoch follows
        motion updates. Late wheel data cannot retroactively change the detector.
        """
        from .zupt import OdometryData
        o = OdometryData([t], [speed], sigma, C_vb, lever_b, [valid])
        t = float(t)
        if (self.t is not None and t <= self.t + _EPS) or t in self._odometry:
            raise ValueError("odometry must be unique and queued before its IMU epoch")
        self._odometry[t] = o
        heapq.heappush(self._odo_epochs, t)

    def _apply_motion(self, t, gyro, acc):
        from .zupt import zupt_update, mounted_odometry_update
        stationary = False if self.detector is None else self.detector.step(gyro, acc, np.linalg.norm(self.filter.v))
        if self.detector is not None and self.detector.quiet == 0:
            self._stop = False
            self._confirmations = 0
        allowed = self._stop or not self.cfg.zupt_require_gnss_stop or (self.cfg.zupt_odometry_stop and self._odo_stop.confirmed(t))
        if stationary and self._index % self.cfg.motion_every == 0:
            if allowed:
                self.motion['zupt_attempted'] += 1
                info = zupt_update(self.filter, self.cfg.zupt_sigma, gate=float(chi2.ppf(.999, 3)))
                self.motion['zupt_accepted'] += int(info.accepted)
                if info.accepted:
                    self.motion['zupt_times'].append(float(t))
            else:
                self.motion['unconfirmed_stop_vetoes'] += 1
        if self._odo_epochs and abs(self._odo_epochs[0]-t) <= _EPS:
            ot = heapq.heappop(self._odo_epochs)
            o = self._odometry.pop(ot)
            if o.valid[0]:
                self.motion['odometry_attempted'] += 1
                info = mounted_odometry_update(self.filter, o.speed[0], o.sigma[0], gyro, o.C_vb, o.lever_b)
                self.motion['odometry_accepted'] += int(info.accepted)
                self._odo_stop.observe(t, o.speed[0], o.sigma[0], info.accepted)

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
    def _propagate(self, gyro, acc, dt):
        self.filter.propagate(gyro, acc, dt)

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
        slow = False
        if self.detector is not None and self.cfg.zupt_require_gnss_stop:
            if self._previous_fix is not None:
                tp, zp, Rp = self._previous_fix
                gap = self.t-tp
                slow = 0 < gap <= 2 and (np.linalg.norm(pos-zp)+2*np.sqrt(np.linalg.eigvalsh(Rp+cov).max()))/gap < .25
            if not slow:
                self._confirmations = 0
                self._stop = False
        force = self._rejects >= self.max_consecutive_rejects
        info = self.filter.update_gnss(pos, self.lever_arm, cov, gate=self.gate, force=force,
                                       robust_delta=self.robust_gnss_delta)
        if info.accepted:
            self._rejects = 0
        else:
            self._rejects += 1
        if self.detector is not None and self.cfg.zupt_require_gnss_stop:
            if info.accepted and not force:
                self._previous_fix = (self.t, pos.copy(), cov.copy())
                self._confirmations = self._confirmations + 1 if slow else 0
                self._stop = self._confirmations >= 2
            else:
                self._previous_fix = None
                self._confirmations = 0
                self._stop = False

    @staticmethod
    def _as_cov(cov) -> np.ndarray:
        c = np.asarray(cov, float)
        if c.ndim == 0:
            return np.eye(3) * float(c)
        if c.ndim == 1:
            return np.diag(c.reshape(3))
        return c.reshape(3, 3).copy()
