"""Opt-in causal learned streaming engine; baseline stream remains independent.

A bounded arrival delay holds IMU samples, never rewinds emitted decisions. GNSS
must arrive before its epoch is processed. flush() drains the pending tail.
"""
from collections import deque
from dataclasses import replace

import numpy as np

from .aci import ACICalibrator, GNSSACICalibrator
from .integrity import Cusum, dr_statistic
from .learning import WINDOW, predict, roll_pitch
from .stream import StreamingNavigator, _EPS


class LearnedStreamingNavigator(StreamingNavigator):
    """Select explicitly via this class and supply a loaded MotionNet checkpoint.

    Supports learned_vel/learned_nhc, ACI, GNSS ACI and single/dual CUSUM.
    Hybrid/bias fallback, wheel/ZUPT, adaptive-R and range coupling are rejected.
    delay is in seconds of IMU event time; get_state() reports processed time.
    """

    def __init__(self, *args, model, delay=0., motion_cfg=None, **kwargs):
        cfg = motion_cfg
        if cfg is None or cfg.pseudo not in ('learned_vel', 'learned_nhc'):
            raise ValueError('learned streaming requires learned_vel or learned_nhc')
        if (cfg.zupt or cfg.adaptive_r or cfg.baseline_range_anchor is not None or
                cfg.monitor not in ('none', 'single', 'dual')):
            raise ValueError('unsupported learned streaming motion/adaptive/range/monitor option')
        if not np.isfinite(delay) or delay < 0:
            raise ValueError('delay must be finite and nonnegative')
        if cfg.corr_scale is not None and (np.shape(cfg.corr_scale) != (3,) or
                not np.isfinite(cfg.corr_scale).all() or np.any(np.asarray(cfg.corr_scale) < 1)):
            raise ValueError('corr_scale requires three finite inflations >= 1')
        baseline = replace(cfg, pseudo='none', aci=False, gnss_aci=False, monitor='none')
        super().__init__(*args, motion_cfg=baseline, **kwargs)
        dt = kwargs.get("dt")
        if dt is None or not np.isfinite(dt) or dt <= 0:
            raise ValueError("positive fixed dt required")
        self.cfg = replace(cfg)
        self.model = model
        self.delay = delay
        self._pending = deque()
        self._arrival_t = None
        self._raw = deque(maxlen=WINDOW)
        self._att = deque(maxlen=WINDOW)
        self._att_b = deque(maxlen=WINDOW)
        self.guard = self.filter.copy() if cfg.monitor == 'dual' else None
        self.cal = ACICalibrator(2 if cfg.pseudo == 'learned_nhc' else 3,
            alpha=cfg.aci_alpha, gamma=cfg.aci_gamma, window=cfg.aci_window,
            beta=cfg.aci_beta, t_ref=cfg.aci_t_ref, enabled=cfg.aci)
        self.gnss_cal = GNSSACICalibrator(alpha=cfg.aci_alpha, gamma=cfg.aci_gamma,
            window=cfg.aci_window) if cfg.gnss_aci else None
        self.cusum = Cusum(cfg.cusum_k, cfg.cusum_h)
        self.last_gnss = -np.inf
        self.last_reset = None
        self.start_time = None
        self.alarm_time = None
        self.decisions = []
        self.gnss_decisions = []
        self._seen_gnss = set()

    @classmethod
    def from_truth(cls, truth, imu_spec, gnss_spec, cfg=None, *, model, delay=0.):
        # Reuse the baseline initialization (including Earth/mount/noise options).
        from .pipeline import initial_state
        from .filters import NoiseParams
        if cfg is None:
            raise ValueError("explicit learned streaming Config required")
        R, v, p, P = initial_state(truth, cfg.init_seed, cfg.p0_bias)
        earth = None
        if cfg.earth_rate_enu is not None or cfg.local_gravity is not None:
            earth = dict(w_ie=None if cfg.earth_rate_enu is None else np.asarray(cfg.earth_rate_enu),
                         g=np.array([0., 0., -(9.81 if cfg.local_gravity is None else cfg.local_gravity)]))
        return cls(R, v, p, P, NoiseParams.from_spec(imu_spec), kind=cfg.filter,
            model=model, delay=delay, motion_cfg=cfg, imu_spec=imu_spec, dt=truth.dt,
            lever_arm=gnss_spec.lever_arm, gate=cfg.gnss_gate,
            max_consecutive_rejects=cfg.max_consecutive_rejects,
            robust_gnss_delta=cfg.robust_gnss_delta, earth=earth)

    def push_imu(self, t, acc, gyro):
        t = float(t)
        acc, gyro = np.asarray(acc, float).reshape(3), np.asarray(gyro, float).reshape(3)
        if not np.isfinite(np.r_[t, acc, gyro]).all():
            raise ValueError('finite IMU required')
        if self._arrival_t is not None and (t <= self._arrival_t or
                not np.isclose(t-self._arrival_t, self.dt, atol=1e-9, rtol=0)):
            raise ValueError('learned streaming requires increasing fixed-rate IMU')
        if self._arrival_t is None:
            for epoch, *_ in self._gnss:
                if epoch < t-_EPS or not np.isclose((epoch-t)/self.dt,
                        round((epoch-t)/self.dt), atol=1e-6):
                    raise ValueError("learned streaming GNSS must align to IMU grid")
        self._arrival_t = t
        self._pending.append((t, acc.copy(), gyro.copy()))
        while self._pending and self._pending[0][0] <= t-self.delay+_EPS:
            self._process(*self._pending.popleft())

    def flush(self):
        """Process the tail after all GNSS within the permitted delay has arrived."""
        while self._pending:
            self._process(*self._pending.popleft())

    def push_odometry(self, *args, **kwargs):
        raise ValueError('wheel updates use the supported baseline StreamingNavigator')

    def push_gnss(self, t, pos, cov):
        # Require on-grid fixes: splitting would change stored feature semantics.
        c = self._as_cov(cov)
        if not np.isfinite(np.r_[t, np.asarray(pos).ravel(), c.ravel()]).all():
            raise ValueError('finite GNSS required')
        if not np.allclose(c, c.T):
            raise ValueError('symmetric GNSS covariance required')
        np.linalg.cholesky(c)
        origin = self.t if self.t is not None else (self._pending[0][0] if self._pending else None)
        if origin is not None and not np.isclose((t-origin)/self.dt, round((t-origin)/self.dt), atol=1e-6):
            raise ValueError('learned streaming GNSS must align to IMU grid')
        # Current-epoch insertion is too late: its pseudo decision was emitted.
        if self.t is not None and t <= self.t+_EPS:
            raise ValueError('GNSS must arrive before its epoch is processed; increase delay')
        if float(t) in self._seen_gnss:
            raise ValueError("duplicate GNSS epoch")
        super().push_gnss(t, pos, c)
        self._seen_gnss.add(float(t))

    def _process(self, t, acc, gyro):
        if self.t is None:
            self.last_reset = t
            self.start_time = t
        else:
            self._raw.append(np.r_[self._last_imu[2], self._last_imu[1]])
            self._att.append(roll_pitch(self.filter.R))
            if self.guard is not None:
                self._att_b.append(roll_pitch(self.guard.R))
                self.guard.propagate(self._last_imu[2], self._last_imu[1], self.dt)
        if self._gnss and self._gnss[0][0] < t-_EPS:
            raise ValueError("learned streaming GNSS must align to IMU grid")
        super().push_imu(t, acc, gyro)

    def _apply_motion(self, t, gyro, acc):
        if len(self._raw) < WINDOW or self._index % self.cfg.pseudo_every:
            return
        raw = np.asarray(self._raw)
        inference_bias = np.r_[self.filter.bg, self.filter.ba].copy()
        mu, var = predict(self.model, raw, np.array([WINDOW]), attitude=np.asarray(self._att),
                          bias=inference_bias)
        mu, var = mu[0], var[0]
        if not np.isfinite(mu).all() or not np.isfinite(var).all() or np.any(var <= 0):
            raise ValueError('nonfinite or nonpositive MotionNet prediction')
        nhc = self.cfg.pseudo == 'learned_nhc'
        ch = [1, 2] if nhc else [0, 1, 2]
        z = np.zeros(2) if nhc else mu
        s2 = var[1:]+mu[1:]**2 if nhc else var
        h, H = self._body(self.filter, ch)
        tau = t-self.last_gnss
        healthy = tau < 1.5 and t > 30. and not self.cusum.alarm
        before = self.cal.kappa.copy()
        if healthy:
            self.cal.observe(z-h, s2, np.einsum('ij,jk,ik->i', H, self.filter.P, H))
        outage = 0. if tau < 1.5 else tau
        # No reference ever received: retain nominal kappa without infinite inflation.
        if not np.isfinite(outage):
            outage = t-self.start_time
            if outage < 1.5:
                outage = 0.
        effective = self.cal.effective_var(s2, outage)
        if not nhc and self.cfg.corr_scale is not None:
            effective *= np.asarray(self.cfg.corr_scale)
        info = self.filter.update(z-h, H, np.diag(effective))
        if self.guard is not None:
            mb, vb = predict(self.model, raw, np.array([WINDOW]), attitude=np.asarray(self._att_b),
                             bias=np.r_[self.guard.bg, self.guard.ba])
            zb = np.zeros(2) if nhc else mb[0]
            sb = vb[0, 1:]+mb[0, 1:]**2 if nhc else vb[0]
            ev = self.cal.effective_var(sb, outage)
            if not nhc and self.cfg.corr_scale is not None:
                ev *= np.asarray(self.cfg.corr_scale)
            hb, Hb = self._body(self.guard, ch)
            self.guard.update(zb-hb, Hb, np.diag(ev))
        self.decisions.append(dict(t=float(t), window_start=float(t-WINDOW*self.dt),
            feature_mean=raw.mean(0).tolist(), attitude_mean=np.asarray(self._att).mean(0).tolist(),
            estimated_bias=inference_bias.tolist(),
            mu=mu.tolist(), variance=var.tolist(),
            effective_variance=effective.tolist(), eligible=bool(healthy and self.cfg.aci),
            observations=self.cal._n, kappa_before=before.tolist(), kappa=self.cal.kappa.tolist(),
            accepted=bool(info.accepted), alarm=bool(self.cusum.alarm)))

    def _body(self, f, ch):
        h, H = f.body_vel_model((0, 1, 2))
        if self.C_vb is not None and self.cfg.pseudo == 'learned_nhc':
            h, H = self.C_vb@h, self.C_vb@H
        return h[ch], H[ch]

    def _apply_gnss(self, pos, cov):
        Rg = cov if self.gnss_cal is None else self.gnss_cal.effective_cov(cov)
        h, H = self.filter.gnss_model(self.lever_arm)
        residual, C = pos-h, H@self.filter.P@H.T
        q = None
        if self.cfg.monitor != 'none':
            q = (dr_statistic(self.guard, pos, self.lever_arm, Rg) if self.guard is not None
                 else float(residual@np.linalg.solve(C+Rg, residual)))
            if self.cusum.step(q) and self.alarm_time is None:
                self.alarm_time = float(self.t)
        use = not (self.cusum.alarm and self.cfg.exclude_on_alarm)
        forced = self._rejects >= self.max_consecutive_rejects
        accepted, eligible = False, False
        scale = 1. if self.gnss_cal is None else self.gnss_cal.kappa
        if use:
            info = self.filter.update_gnss(pos, self.lever_arm, Rg, gate=self.gate,
                                          force=forced, robust_delta=self.robust_gnss_delta)
            accepted = bool(info.accepted)
            self._rejects = 0 if accepted else self._rejects+1
            if accepted:
                self.last_gnss = self.t
            eligible = accepted and not forced and not self.cusum.alarm and self.t > 30.
            if self.gnss_cal is not None and eligible:
                self.gnss_cal.observe(residual, cov, C)
        self.gnss_decisions.append(dict(t=float(self.t), q=q, g=float(self.cusum.g),
            excluded=not use, accepted=accepted, eligible=bool(eligible), kappa_used=scale,
            kappa_next=1. if self.gnss_cal is None else self.gnss_cal.kappa))
        if (self.guard is not None and self.t-self.last_reset >= self.cfg.dr_reset_period
                and not self.cusum.alarm and self.cusum.g < .25*self.cfg.cusum_h
                and self.t-self.last_gnss < 1.5):
            self.guard = self.filter.copy()
            self.last_reset = self.t
