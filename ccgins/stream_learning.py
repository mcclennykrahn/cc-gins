"""Opt-in causal learned streaming engine; baseline stream remains independent.

A bounded arrival delay holds IMU samples, never rewinds emitted decisions. GNSS
must arrive before its epoch is processed. flush() drains the pending tail.
"""
from collections import deque
from dataclasses import replace

import numpy as np

from .aci import ACICalibrator, GNSSACICalibrator
from .integrity import Cusum, GuardReferenceHealth, dr_statistic
from .learning import WINDOW, predict, roll_pitch
from .stream import StreamingNavigator, _EPS


class LearnedStreamingNavigator(StreamingNavigator):
    """Select explicitly via this class and supply a loaded MotionNet checkpoint.

    Supports learned_vel/learned_nhc, ACI, GNSS ACI and single/dual CUSUM.
    Hybrid/bias fallback and wheels are supported. ZUPT, adaptive-R and range coupling are rejected.
    delay is in seconds of IMU event time; get_state() reports processed time.
    """

    def __init__(self, *args, model, delay=0., motion_cfg=None, max_buffer=4096, callback=None, **kwargs):
        cfg = motion_cfg
        if cfg is None or cfg.pseudo not in ('learned_vel', 'learned_nhc', 'hybrid'):
            raise ValueError('learned streaming requires learned_vel, learned_nhc or hybrid')
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
        if not isinstance(max_buffer, int) or max_buffer < np.ceil(delay/self.dt)+1:
            raise ValueError("max_buffer must cover delay budget")
        self.max_buffer, self.callback = max_buffer, callback
        self._pending = deque()
        self._arrival_t = None
        self._raw = deque(maxlen=WINDOW)
        self._att = deque(maxlen=WINDOW)
        self._att_b = deque(maxlen=WINDOW)
        self.guard = self.filter.copy() if cfg.monitor == 'dual' else None
        self.cal = ACICalibrator(1 if cfg.pseudo == 'hybrid' else (2 if cfg.pseudo == 'learned_nhc' else 3),
            alpha=cfg.aci_alpha, gamma=cfg.aci_gamma, window=cfg.aci_window,
            beta=cfg.aci_beta, t_ref=cfg.aci_t_ref, enabled=cfg.aci)
        self.gnss_cal = GNSSACICalibrator(alpha=cfg.aci_alpha, gamma=cfg.aci_gamma,
            window=cfg.aci_window) if cfg.gnss_aci else None
        self.cusum = Cusum(cfg.cusum_k, cfg.cusum_h, recovery_epochs=cfg.recovery_epochs,
                           recovery_q=cfg.recovery_q)
        self.guard_ref = (GuardReferenceHealth(cfg.guard_ref_max_age)
                          if cfg.guard_ref_health else None)
        self.next_reset = None
        self.guard_ref_fallbacks = 0
        self.guard_ref_first_fallback = None
        self.last_gnss = -np.inf
        self.last_reset = None
        self.start_time = None
        self.alarm_time = None
        self.bias_r = deque(maxlen=cfg.hybrid_bias_window)
        self.bias_v = deque(maxlen=cfg.hybrid_bias_window)
        self.bias_bad = cfg.hybrid_bias_z > 0
        self.bias_latched = False
        self.decisions = []
        self.gnss_decisions = []
        self._gnss_arrivals = {}
        self._active_arrival = None

    @classmethod
    def from_truth(cls, truth, imu_spec, gnss_spec, cfg=None, *, model, delay=0., **options):
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
        return cls(R, v, p, P, NoiseParams.from_spec(imu_spec, dt=truth.dt), kind=cfg.filter,
            model=model, delay=delay, **options, motion_cfg=cfg, imu_spec=imu_spec, dt=truth.dt,
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
        if self.t is None and self._gnss and self._gnss[0][0] < (self._pending[0][0] if self._pending else t)-_EPS:
            raise ValueError("GNSS precedes first IMU epoch")
        if len(self._pending) >= self.max_buffer:
            raise BufferError("IMU buffer full")
        self._arrival_t = t
        self._pending.append((t, acc.copy(), gyro.copy()))
        while self._pending and self._pending[0][0] <= t-self.delay+_EPS:
            self._process(*self._pending.popleft())

    def flush(self):
        """Process the tail after all GNSS within the permitted delay has arrived."""
        while self._pending:
            self._process(*self._pending.popleft())

    def push_odometry(self, *args, **kwargs):
        if len(self._odometry) >= self.max_buffer:
            raise BufferError('wheel buffer full')
        t = float(args[0] if args else kwargs['t'])
        origin = self.start_time if self.start_time is not None else (self._pending[0][0] if self._pending else None)
        if not np.isfinite(t) or (origin is not None and not np.isclose((t-origin)/self.dt, round((t-origin)/self.dt), atol=1e-6)):
            raise ValueError('wheel must align to nominal IMU grid')
        return super().push_odometry(*args, **kwargs)

    def push_gnss(self, t, pos, cov, *, arrival_time=None):
        """Queue before processing; optional arrival_time is caller clock metadata."""
        if arrival_time is not None and (not np.isfinite(arrival_time) or arrival_time < t):
            raise ValueError("arrival_time must be finite and no earlier than epoch")
        c = self._as_cov(cov)
        if not np.isfinite(np.r_[t, np.asarray(pos).ravel(), c.ravel()]).all():
            raise ValueError('finite GNSS required')
        if not np.allclose(c, c.T):
            raise ValueError('symmetric GNSS covariance required')
        np.linalg.cholesky(c)
        # Current-epoch insertion is too late: its pseudo decision was emitted.
        if self.t is not None and t <= self.t+_EPS:
            raise ValueError('GNSS must arrive before its epoch is processed; increase delay')
        if any(abs(float(t)-g[0]) <= _EPS for g in self._gnss):
            raise ValueError("duplicate GNSS epoch")
        if len(self._gnss) >= self.max_buffer:
            raise BufferError("GNSS buffer full")
        super().push_gnss(t, pos, c)
        self._gnss_arrivals[float(t)] = self._arrival_t if arrival_time is None else float(arrival_time)

    def _process(self, t, acc, gyro):
        if self.t is None:
            self.last_reset = t
            self.start_time = t
            if self.guard_ref is not None:
                self.guard_ref.begin(t)
                self.next_reset = t + self.cfg.dr_reset_period
        else:
            self._raw.append(np.r_[self._last_imu[2], self._last_imu[1]])
            self._att.append(roll_pitch(self.filter.R))
            if self.guard is not None:
                self._att_b.append(roll_pitch(self.guard.R))
        super().push_imu(t, acc, gyro)
        if self.callback is not None:
            self.callback(dict(event='imu', epoch=float(t), arrival_time=self._arrival_t,
                               status='processed', state=self.get_state()))

    def _propagate(self, gyro, acc, dt):
        super()._propagate(gyro, acc, dt)
        if self.guard is not None:
            self.guard.propagate(gyro, acc, dt)

    def _apply_motion(self, t, gyro, acc):
        wheel = self._odometry.get(self._odo_epochs[0]) if self._odo_epochs and abs(self._odo_epochs[0]-t) <= _EPS else None
        super()._apply_motion(t, gyro, acc)
        if wheel is not None and wheel.valid[0] and self.guard is not None:
            from .zupt import mounted_odometry_update
            mounted_odometry_update(self.guard, wheel.speed[0], wheel.sigma[0], gyro, wheel.C_vb, wheel.lever_b)
        if len(self._raw) < WINDOW or self._index % self.cfg.pseudo_every:
            return
        raw = np.asarray(self._raw)
        inference_bias = np.r_[self.filter.bg, self.filter.ba].copy()
        mu, var = predict(self.model, raw, np.array([WINDOW]), attitude=np.asarray(self._att),
                          bias=inference_bias)
        mu, var = mu[0], var[0]
        if not np.isfinite(mu).all() or not np.isfinite(var).all() or np.any(var <= 0):
            raise ValueError('nonfinite or nonpositive MotionNet prediction')
        if self.cfg.pseudo == 'hybrid':
            self._hybrid(t, raw, mu, var)
            return
        nhc = self.cfg.pseudo == 'learned_nhc'
        ch = [1, 2] if nhc else [0, 1, 2]
        z = np.zeros(2) if nhc else mu
        s2 = var[1:]+mu[1:]**2 if nhc else var
        h, H = self._body(self.filter, ch)
        tau = t-self.last_gnss
        ref_ok = self._guard_ref_ok(t)
        healthy = tau < 1.5 and t > 30. and not self.cusum.alarm and ref_ok
        before = self.cal.kappa.copy()
        if healthy:
            self.cal.observe(z-h, s2, np.einsum('ij,jk,ik->i', H, self.filter.P, H))
        outage = 0. if tau < 1.5 else tau
        # No reference ever received: retain nominal kappa without infinite inflation.
        if not np.isfinite(outage):
            outage = t-self.start_time
            if outage < 1.5:
                outage = 0.
        ref_fallback = (not nhc) and self.guard_ref is not None and not ref_ok
        if ref_fallback:
            # Untrustworthy reference: retain lateral/vertical fixed NHC and drop
            # the learned forward channel; ACI/biasing above did not update.
            sel = [1, 2]
            self.filter.update(-h[sel], H[sel], np.diag(self.cfg.nhc_variances()))
            effective = self.cfg.nhc_variances()
            info = None
            self.guard_ref_fallbacks += 1
            if self.guard_ref_first_fallback is None:
                self.guard_ref_first_fallback = float(t)
        else:
            effective = self.cal.effective_var(s2, outage)
            if not nhc and self.cfg.corr_scale is not None:
                effective *= np.asarray(self.cfg.corr_scale)
            info = self.filter.update(z-h, H, np.diag(effective))
        if self.guard is not None:
            mb, vb = predict(self.model, raw, np.array([WINDOW]), attitude=np.asarray(self._att_b),
                             bias=np.r_[self.guard.bg, self.guard.ba])
            zb = np.zeros(2) if nhc else mb[0]
            sb = vb[0, 1:]+mb[0, 1:]**2 if nhc else vb[0]
            if ref_fallback:
                hb, Hb = self._body(self.guard, ch)
                self.guard.update(-hb[1:], Hb[1:], np.diag(self.cfg.nhc_variances()))
            else:
                ev = self.cal.effective_var(sb, outage)
                if not nhc and self.cfg.corr_scale is not None:
                    ev *= np.asarray(self.cfg.corr_scale)
                hb, Hb = self._body(self.guard, ch)
                self.guard.update(zb-hb, Hb, np.diag(ev))
        self._record(self.decisions, dict(t=float(t), window_start=float(t-WINDOW*self.dt),
            feature_mean=raw.mean(0).tolist(), attitude_mean=np.asarray(self._att).mean(0).tolist(),
            estimated_bias=inference_bias.tolist(),
            mu=mu.tolist(), variance=var.tolist(),
            effective_variance=effective.tolist(), eligible=bool(healthy and self.cfg.aci),
            observations=self.cal._n, kappa_before=before.tolist(), kappa=self.cal.kappa.tolist(),
            accepted=bool(info.accepted) if info is not None else False,
            guard_ref_healthy=bool(ref_ok), guard_ref_fallback=bool(ref_fallback),
            alarm=bool(self.cusum.alarm)))

    def _guard_ref_ok(self, t):
        if self.guard_ref is None:
            return True
        return bool(self.guard_ref.check_fresh(t))

    def _hybrid(self, t, raw, mu, var):
        h, H = self._body(self.filter, [0, 1, 2])
        tau = t-self.last_gnss
        ref_ok = self._guard_ref_ok(t)
        healthy = tau < 1.5 and t > 30. and not self.cusum.alarm and ref_ok
        if healthy:
            self.cal.observe(mu[:1]-h[:1], var[:1],
                np.einsum('ij,jk,ik->i', H[:1], self.filter.P, H[:1]))
            if self.cfg.hybrid_bias_z > 0:
                self.bias_r.append(float(mu[0]-h[0]))
                self.bias_v.append(float(var[0]))
                if len(self.bias_r) >= self.cfg.hybrid_bias_min:
                    # Same overlap correction as the batch hybrid bias monitor.
                    n_eff = 1. if self.cfg.corr_scale is None else self.cfg.corr_scale[0]
                    score = abs(np.mean(self.bias_r))/np.sqrt(np.mean(self.bias_v)*n_eff/len(self.bias_r))
                    if score > self.cfg.hybrid_bias_z:
                        self.bias_bad = True
                        self.bias_latched = self.cfg.hybrid_bias_latch
                    elif score < self.cfg.hybrid_bias_z/2 and not self.bias_latched:
                        self.bias_bad = False
        outage = 0. if tau < 1.5 else (tau if np.isfinite(tau) else t-self.start_time)
        if not np.isfinite(tau) and outage < 1.5:
            outage = 0.
        def variance(v):
            forward = self.cal.effective_var(v[:1], outage)[0]
            if self.cfg.corr_scale is not None:
                forward *= self.cfg.corr_scale[0]
            return np.r_[forward, self.cfg.nhc_variances()]
        ev = variance(var)
        ref_fallback = self.guard_ref is not None and not ref_ok
        fallback = (self.bias_bad or np.sqrt(ev[0]) > self.cfg.hybrid_max_std or
                    self.cal.kappa[0] > self.cfg.hybrid_max_kappa or ref_fallback)
        sel = [1, 2] if fallback else [0, 1, 2]
        z = np.array([mu[0], 0., 0.])
        info = self.filter.update((z-h)[sel], H[sel], np.diag(ev[sel]))
        if ref_fallback:
            self.guard_ref_fallbacks += 1
            if self.guard_ref_first_fallback is None:
                self.guard_ref_first_fallback = float(t)
        if self.guard is not None:
            mb, vb = predict(self.model, raw, np.array([WINDOW]), attitude=np.asarray(self._att_b),
                             bias=np.r_[self.guard.bg, self.guard.ba])
            hb, Hb = self._body(self.guard, [0, 1, 2])
            zb = np.array([mb[0, 0], 0., 0.])
            self.guard.update((zb-hb)[sel], Hb[sel], np.diag(variance(vb[0])[sel]))
        self._record(self.decisions, dict(t=float(t), fallback=bool(fallback), accepted=bool(info.accepted),
            eligible=bool(healthy and self.cfg.aci), effective_variance=ev.tolist(),
            guard_ref_healthy=bool(ref_ok), guard_ref_fallback=bool(ref_fallback)))

    def _record(self, history, event):
        # Calibration adaptation uses cal.buf, not this diagnostic history.
        if len(self.cal.history) > self.max_buffer:
            del self.cal.history[:-self.max_buffer]
        history.append(event)
        if len(history) > self.max_buffer:
            del history[0]

    def _body(self, f, ch):
        h, H = f.body_vel_model((0, 1, 2))
        if self.C_vb is not None and self.cfg.pseudo in ('learned_nhc', 'hybrid'):
            h, H = self.C_vb@h, self.C_vb@H
        return h[ch], H[ch]

    def _pop_and_apply_gnss(self):
        # Use the queued key, even when the base treats a near-grid epoch as equal.
        self._active_arrival = self._gnss_arrivals.pop(self._gnss[0][0], None)
        super()._pop_and_apply_gnss()

    def _apply_gnss(self, pos, cov):
        Rg = cov if self.gnss_cal is None else self.gnss_cal.effective_cov(cov)
        h, H = self.filter.gnss_model(self.lever_arm)
        residual, C = pos-h, H@self.filter.P@H.T
        nis = float(residual@np.linalg.solve(C+Rg, residual))
        q = None
        was = self.cusum.alarm
        if self.cfg.monitor != 'none':
            q = (dr_statistic(self.guard, pos, self.lever_arm, Rg) if self.guard is not None
                 else float(residual@np.linalg.solve(C+Rg, residual)))
            if self.cusum.step(q) and self.alarm_time is None:
                self.alarm_time = float(self.t)
        use = not ((was or self.cusum.alarm) and self.cfg.exclude_on_alarm)
        forced = self._rejects >= self.max_consecutive_rejects
        recovered = bool(was and not self.cusum.alarm)
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
        if self.guard_ref is not None:
            self.guard_ref.observe(self.t, accepted=accepted, q=q, forced=forced,
                                   alarm=bool(was or self.cusum.alarm), recovery=recovered)
        self._record(self.gnss_decisions, dict(t=float(self.t), arrival_time=self._active_arrival, processed_arrival_time=self._arrival_t, alarm=bool(self.cusum.alarm), status='accepted' if accepted else ('excluded' if not use else 'rejected'), nis=nis, q=q, g=float(self.cusum.g),
            recoveries=self.cusum.recoveries, healthy_streak=self.cusum.healthy_streak,
            excluded=not use, accepted=accepted, eligible=bool(eligible), kappa_used=scale,
            guard_ref_healthy=(None if self.guard_ref is None else bool(self.guard_ref.healthy)),
            guard_ref_fallback_epochs=self.guard_ref_fallbacks,
            kappa_next=1. if self.gnss_cal is None else self.gnss_cal.kappa))
        if self.callback is not None:
            self.callback(dict(event='gnss', **self.gnss_decisions[-1]))
        if self.guard is not None and self.t-self.last_reset >= self.cfg.dr_reset_period:
            if self.guard_ref is None:
                if (not self.cusum.alarm and self.cusum.g < .25*self.cfg.cusum_h
                        and self.t-self.last_gnss < 1.5):
                    self.guard = self.filter.copy()
                    self.last_reset = self.t
            else:
                self.guard_ref.check_fresh(self.t)
                if (self.guard_ref.healthy and not self.cusum.alarm and
                        self.cusum.g < .25*self.cfg.cusum_h and
                        self.t-self.last_gnss < 1.5):
                    self.guard = self.filter.copy()
                    self.guard_ref.resets += 1
                else:
                    self.guard_ref.reset_skips += 1
                # Every scheduled boundary opens a fresh assessment interval,
                # reseeded or not, avoiding a permanent outage deadlock.
                self.guard_ref.begin(self.t)
                # Boundary evidence belongs to the new interval, including
                # active disqualifiers that must keep learned aiding disabled.
                self.guard_ref.observe(self.t, accepted=accepted, q=q, forced=forced,
                                       alarm=bool(was or self.cusum.alarm),
                                       recovery=recovered)
                self.last_reset = self.t
