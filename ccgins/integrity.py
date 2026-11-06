"""GNSS integrity / spoofing monitors.

Single channel: CUSUM on the main filter's GNSS innovation NIS.
Dual channel  : an independent dead-reckoning (DR) filter B (IMU + calibrated motion pseudo-
                measurements, NO GNSS) is periodically re-seeded from the main filter A.
                Statistic q = d^T (R + H P_B H^T)^{-1} d, d = y_gnss - h_B ;
                CUSUM g = max(0, g + q - 3 - k); alarm when g > h (latched by default).

Opt-in recovery uses consecutive received GNSS statistics <= recovery_q; it
asserts statistical agreement only, not source authentication. Missing epochs do
not count. Callers exclude the completing recovery epoch and resume next fix.
"""
from __future__ import annotations

import numpy as np


class GuardReferenceHealth:
    """Sticky health of the GNSS reference over one whole DR-reset interval.

    A guard (DR filter B) may only be reseeded from a reference whose entire
    interval since the last scheduled reset was trustworthy.  The state records
    accepted/fresh/finite/non-forced/non-alarm/non-recovery epochs and becomes
    permanently unhealthy for the rest of the interval on any violation.  A
    later healthy fix therefore cannot erase earlier suspicious history.

    ``begin(t)`` opens a new assessment interval at a scheduled DR boundary;
    callers evaluate ``healthy`` at that boundary, reseed only if healthy, then
    call ``begin`` again even when they did not reseed.  That rules out a
    permanent deadlock after an outage. Callers must record the boundary GNSS
    epoch again after ``begin``, including all disqualifiers, so an active alarm
    or recovery cannot become healthy merely through renewal.
    ``check_fresh(t)`` marks the interval
    stale/missing when no accepted fix is recent enough; it is deliberately
    causal (no look-ahead).

    This is reference-consistency bookkeeping, not source authentication.
    """

    REASONS = ("missing", "stale", "rejected", "forced", "nonfinite",
               "alarm", "recovery")

    def __init__(self, max_age: float = 1.5):
        if not np.isfinite(max_age) or max_age <= 0:
            raise ValueError("guard-reference max_age must be finite and positive")
        self.max_age = float(max_age)
        self.start_t = None
        self.last_fix_t = None
        self.epochs = 0
        self.accepted = 0
        self.bad = False
        self.reasons = []
        self.intervals = 0
        self.resets = 0
        self.reset_skips = 0
        self.unhealthy_intervals = 0
        self.history_reasons = []
        self.begin(0.0)

    def begin(self, t) -> None:
        """Start a new whole-interval assessment (scheduled DR boundary)."""
        if not np.isfinite(t):
            raise ValueError("assessment boundary must be finite")
        if self.bad:
            self.unhealthy_intervals += 1
            for reason in self.reasons:
                if reason not in self.history_reasons:
                    self.history_reasons.append(reason)
        self.start_t = float(t)
        self.last_fix_t = None
        self.epochs = 0
        self.accepted = 0
        self.bad = False
        self.reasons = []
        self.intervals += 1

    @property
    def healthy(self) -> bool:
        # A fresh interval before its first fix is pending, not yet bad.  Callers
        # must call check_fresh(t) before trusting this at a decision epoch; after
        # max_age without an accepted fix it becomes "missing".
        return not self.bad

    def invalidate(self, reason: str) -> None:
        if reason not in self.REASONS:
            raise ValueError(f"unknown guard-reference reason {reason!r}")
        if reason not in self.reasons:
            self.reasons.append(reason)
        self.bad = True

    def observe(self, t, *, accepted: bool, q=None, forced: bool = False,
                alarm: bool = False, recovery: bool = False) -> bool:
        """Record one received GNSS epoch; returns health after the epoch."""
        t = float(t)
        if not np.isfinite(t):
            raise ValueError("guard-reference epoch must be finite")
        if self.start_t is None:
            self.begin(t)
        self.epochs += 1
        # Order is intentional: the first recorded cause is the diagnostic one,
        # while all applicable causes are retained for the reason-code summary.
        if alarm:
            self.invalidate("alarm")
        if recovery:
            self.invalidate("recovery")
        if forced:
            self.invalidate("forced")
        if q is not None and not np.isfinite(q):
            self.invalidate("nonfinite")
        if not accepted:
            self.invalidate("rejected")
        elif not (alarm or recovery or forced):
            self.accepted += 1
            self.last_fix_t = t
        return self.healthy

    def check_fresh(self, t) -> bool:
        """Causally invalidate the interval if its last accepted fix is too old."""
        t = float(t)
        if not np.isfinite(t):
            raise ValueError("guard-reference time must be finite")
        if self.last_fix_t is None:
            if self.start_t is not None and t - self.start_t > self.max_age:
                self.invalidate("missing")
        elif t - self.last_fix_t > self.max_age:
            self.invalidate("stale")
        return self.healthy

    def reason_codes(self) -> dict:
        return {r: int(r in self.reasons) for r in self.REASONS}

    def summary(self) -> dict:
        return dict(enabled=True, healthy=bool(self.healthy), bad=bool(self.bad),
                    intervals=self.intervals, resets=self.resets,
                    reset_skips=self.reset_skips,
                    unhealthy_intervals=self.unhealthy_intervals,
                    accepted=self.accepted, epochs=self.epochs,
                    reasons=list(self.reasons), reason_codes=self.reason_codes(),
                    history_reasons=list(self.history_reasons),
                    last_fix_t=None if self.last_fix_t is None else float(self.last_fix_t),
                    start_t=None if self.start_t is None else float(self.start_t))


class Cusum:
    def __init__(self, k: float = 3.0, h: float = 50.0, dof: int = 3,
                 recovery_epochs: int = 0, recovery_q: float = 3.0):
        if (not isinstance(recovery_epochs, int) or isinstance(recovery_epochs, bool)
                or recovery_epochs < 0):
            raise ValueError("recovery_epochs must be a nonnegative integer")
        if not np.isfinite(recovery_q) or recovery_q < 0:
            raise ValueError("recovery_q must be finite and nonnegative")
        self.recovery_epochs, self.recovery_q = recovery_epochs, recovery_q
        self.healthy_streak = 0
        self.recoveries = 0
        self.k, self.h, self.dof = k, h, dof
        self.g = 0.0
        self.alarm = False

    def step(self, q: float) -> bool:
        # Invalid evidence must never silently clear the latch or advance recovery.
        if not np.isfinite(q) or q < 0:
            self.healthy_streak = 0
            raise ValueError("CUSUM requires finite nonnegative q")
        if self.alarm and self.recovery_epochs:
            self.healthy_streak = self.healthy_streak + 1 if q <= self.recovery_q else 0
            if self.healthy_streak >= self.recovery_epochs:
                self.alarm = False
                self.g = 0.0
                self.healthy_streak = 0
                self.recoveries += 1
                return False
        self.g = max(0.0, self.g + q - self.dof - self.k)
        if self.g > self.h:
            self.alarm = True
        return self.alarm


def dr_statistic(fB, z, lever, Rm):
    h, H = fB.gnss_model(lever)
    d = z - h
    S = H @ fB.P @ H.T + Rm
    return float(d @ np.linalg.solve(S, d))
