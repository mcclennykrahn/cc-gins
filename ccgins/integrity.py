"""GNSS integrity / spoofing monitors.

Single channel: CUSUM on the main filter's GNSS innovation NIS.
Dual channel  : an independent dead-reckoning (DR) filter B (IMU + calibrated motion pseudo-
                measurements, NO GNSS) is periodically re-seeded from the main filter A.
                Statistic q = d^T (R + H P_B H^T)^{-1} d, d = y_gnss - h_B ;
                CUSUM g = max(0, g + q - 3 - k); alarm when g > h (latched).
"""
from __future__ import annotations

import numpy as np


class Cusum:
    def __init__(self, k: float = 3.0, h: float = 50.0, dof: int = 3):
        self.k, self.h, self.dof = k, h, dof
        self.g = 0.0
        self.alarm = False

    def step(self, q: float) -> bool:
        self.g = max(0.0, self.g + q - self.dof - self.k)
        if self.g > self.h:
            self.alarm = True
        return self.alarm


def dr_statistic(fB, z, lever, Rm):
    h, H = fB.gnss_model(lever)
    d = z - h
    S = H @ fB.P @ H.T + Rm
    return float(d @ np.linalg.solve(S, d))
