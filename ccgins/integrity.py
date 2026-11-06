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
