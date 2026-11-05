"""Innovation-based adaptive GNSS/RTK measurement-noise (R) estimation.

For a position update the innovation ``r = z - h`` has covariance ``S = H P H^T + R`` (the
filter's predicted position covariance plus the measurement noise).  A Sage-Husa style
estimator therefore observes ``r r^T - H P H^T`` and averages it over a trailing window with
exponential forgetting.  The result is clamped element-wise to
``[min_scale, max_scale] * R_nominal`` so that a few outliers or a short healthy stretch
cannot drive R to zero or to infinity.

The estimator is deliberately opt-in: ``gnss_update(..., estimator=None)`` (the default)
reduces to ``nav.update`` with the nominal R, so existing results are unchanged.
"""
from __future__ import annotations

from collections import deque

import numpy as np

DEFAULT_WINDOW = 50
DEFAULT_FORGETTING = 0.9
DEFAULT_MIN_SCALE = 0.25
DEFAULT_MAX_SCALE = 16.0


class AdaptiveR:
    """Sliding-window Sage-Husa estimate of the (diagonal) GNSS position noise matrix.

    The window stores de-biased per-epoch estimates ``diag(r r^T) - diag(H P H^T)``.  Samples
    are averaged with weights ``forgetting**age`` (the newest sample has weight 1), so a
    forgetting factor close to 1 gives a long, smooth memory while smaller values track changes
    faster.  Only the diagonal is estimated: RTK position noise is treated as axis-independent,
    which matches the diagonal R used by the pipeline.
    """

    def __init__(self, R_nominal, window: int = DEFAULT_WINDOW,
                 forgetting: float = DEFAULT_FORGETTING,
                 min_scale: float = DEFAULT_MIN_SCALE,
                 max_scale: float = DEFAULT_MAX_SCALE):
        R_nominal = np.asarray(R_nominal, float)
        if R_nominal.ndim == 1:
            R_nominal = np.diag(R_nominal)
        if R_nominal.ndim != 2 or R_nominal.shape[0] != R_nominal.shape[1]:
            raise ValueError("R_nominal must be a square matrix or a vector of variances")
        if window < 1:
            raise ValueError("window must be >= 1")
        if not 0.0 < forgetting <= 1.0:
            raise ValueError("forgetting must be in (0, 1]")
        if min_scale <= 0.0 or max_scale < min_scale:
            raise ValueError("need 0 < min_scale <= max_scale")

        self.R_nominal = R_nominal
        self._nominal_diag = np.diag(R_nominal).copy()
        self.window = int(window)
        self.forgetting = float(forgetting)
        self.min_scale = float(min_scale)
        self.max_scale = float(max_scale)
        self._samples: deque = deque(maxlen=self.window)
        self.R = R_nominal.copy()
        self.n_updates = 0

    def update(self, innovation, H, P) -> np.ndarray:
        """Feed one innovation and return the clamped R estimate (dim x dim).

        innovation: (d,) measurement residual ``z - h``.
        H: (d, m) measurement Jacobian, P: (m, m) predicted state covariance.
        """
        r = np.asarray(innovation, float).reshape(-1)
        H = np.asarray(H, float)
        P = np.asarray(P, float)
        if H.ndim != 2 or H.shape[0] != r.size:
            raise ValueError("H must have shape (len(innovation), m)")
        if P.shape != (H.shape[1], H.shape[1]):
            raise ValueError("P must have shape (m, m) matching H")

        hpht = H @ P @ H.T
        # Sample estimate of R: remove the part of the innovation covariance explained by P.
        self._samples.append(np.diag(np.outer(r, r)) - np.diag(hpht))

        samples = np.asarray(self._samples)
        ages = np.arange(samples.shape[0] - 1, -1, -1)
        weights = self.forgetting ** ages
        raw = (weights[:, None] * samples).sum(axis=0) / weights.sum()

        lo = self.min_scale * self._nominal_diag
        hi = self.max_scale * self._nominal_diag
        # A tiny positive floor keeps the matrix invertible even when R_nominal is singular.
        self.R = np.diag(np.clip(raw, lo, hi))
        self.n_updates += 1
        return self.R.copy()

    def reset(self):
        """Drop the window and restore the nominal R."""
        self._samples.clear()
        self.R = self.R_nominal.copy()
        self.n_updates = 0


def gnss_update(nav, z, lever, R, estimator: AdaptiveR | None = None,
                gate: float | None = None, force: bool = False):
    """GNSS position update with optional adaptive R.

    Drop-in replacement for ``h, H = nav.gnss_model(lever); nav.update(z - h, H, R, ...)``.
    ``estimator=None`` (default) uses ``R`` unchanged, so the update is identical to the
    existing pipeline.  Pass an :class:`AdaptiveR` instance to estimate R from the innovation
    before the update.
    """
    h, H = nav.gnss_model(lever)
    r = np.asarray(z, float) - h
    if estimator is not None:
        R = estimator.update(r, H, nav.P)
    return nav.update(r, H, R, gate=gate, force=force)
