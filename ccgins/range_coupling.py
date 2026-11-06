"""Scalar pseudo-range coupling from a local RTK baseline, not raw satellite ranges.

Only the radial direction is observed. Never fuse this and the source RTK fix as
independent measurements: their noise is shared. Anchor coordinates are fixed ENU.
"""
import numpy as np
from .integrity import Cusum


def baseline_range_model(nav, z, lever, covariance, anchor):
    """Return scalar innovation, state Jacobian and delta-method range covariance."""
    z, anchor = np.asarray(z, float), np.asarray(anchor, float)
    h, H = nav.gnss_model(np.asarray(lever, float))
    measured, predicted = z-anchor, h-anchor
    rz, rh = np.linalg.norm(measured), np.linalg.norm(predicted)
    if min(rz, rh) < 1e-6:
        raise ValueError('range geometry is undefined at the anchor')
    u = measured/rz
    variance = float(u @ np.asarray(covariance) @ u)
    if not np.isfinite(variance) or variance <= 0:
        raise ValueError('projected range variance must be positive')
    return np.array([rz-rh]), (predicted/rh)[None,:] @ H, np.array([[variance]])


class BaselineRangeCoupler:
    """Fuse one scalar per epoch; latch a pre-update residual CUSUM alarm.

    Alarm signals inconsistency, not uniquely lever-arm error. Radial lever drift
    is observable only when the inertial prediction supplies an independent constraint.
    """
    def __init__(self, anchor, k=3., threshold=50.):
        self.anchor = np.asarray(anchor, float)
        self.monitor = Cusum(k=k, h=threshold, dof=1)
        self.last_residual = 0.

    def update(self, nav, z, lever, covariance, gate=None, force=False):
        r,H,R = baseline_range_model(nav,z,lever,covariance,self.anchor)
        S = H @ nav.P @ H.T + R
        nis = float(r @ np.linalg.solve(S,r))
        self.last_residual = float(r[0])
        self.monitor.step(nis)
        return nav.update(r,H,R,gate=gate,force=force)
