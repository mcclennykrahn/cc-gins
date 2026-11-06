"""Adaptive conformal calibration of (learned) measurement variances for Kalman filters.

For each scalar channel j with predicted variance s2_j and residual r_j against a reference
with variance c_j, the nonconformity score is

    q_j(kappa) = r_j^2 / (kappa * s2_j + c_j)

Target: P(q_j <= chi2_1(1-alpha)) = 1 - alpha.  kappa_j is found by bisection so that the
empirical (1-alpha_t) quantile of the last `window` scores equals chi2_1(1-alpha); alpha_t is
updated with Adaptive Conformal Inference (Gibbs & Candes, 2021):

    alpha_{t+1} = alpha_t + gamma * (alpha - err_t)

During GNSS outages (no reference) kappa is frozen and inflated:
    kappa_eff = kappa * (1 + beta * tau / T_ref)
"""
from __future__ import annotations

from collections import deque

import numpy as np
from scipy.stats import chi2


class ACICalibrator:
    def __init__(self, dim: int, alpha: float = 0.05, gamma: float = 0.01, window: int = 300,
                 min_samples: int = 30, update_every: int = 5, beta: float = 0.5, t_ref: float = 60.0,
                 kappa_bounds=(1e-2, 1e4), kappa0: float = 1.0, enabled: bool = True):
        self.dim = dim
        self.alpha = alpha
        self.gamma = gamma
        self.alpha_t = np.full(dim, alpha)
        self.kappa = np.full(dim, kappa0)
        self.q_target = float(chi2.ppf(1 - alpha, 1))
        self.buf = deque(maxlen=window)
        self.min_samples = min_samples
        self.update_every = update_every
        self.beta = beta
        self.t_ref = t_ref
        self.kb = kappa_bounds
        self.enabled = enabled
        self._n = 0
        self.history = []  # (kappa copy)

    def effective_var(self, s2: np.ndarray, outage_time: float = 0.0) -> np.ndarray:
        if not self.enabled:
            return s2
        infl = 1.0 + self.beta * max(outage_time, 0.0) / self.t_ref
        return self.kappa * infl * s2

    def observe(self, r: np.ndarray, s2: np.ndarray, ref_var: np.ndarray) -> None:
        if not self.enabled:
            return
        r2 = np.asarray(r, float) ** 2
        s2 = np.asarray(s2, float)
        c = np.asarray(ref_var, float)
        err = (r2 / (self.kappa * s2 + c)) > self.q_target
        self.alpha_t = np.clip(self.alpha_t + self.gamma * (self.alpha - err), 1e-3, 0.5)
        self.buf.append(np.stack([r2, s2, c]))
        self._n += 1
        if len(self.buf) >= self.min_samples and self._n % self.update_every == 0:
            self._solve()

    def _solve(self, iters: int = 30) -> None:
        B = np.asarray(self.buf)            # (W, 3, dim)
        r2, s2, c = B[:, 0, :], B[:, 1, :], B[:, 2, :]
        lo = np.full(self.dim, np.log(self.kb[0]))
        hi = np.full(self.dim, np.log(self.kb[1]))
        level = 1.0 - self.alpha_t
        for _ in range(iters):
            mid = 0.5 * (lo + hi)
            q = r2 / (np.exp(mid) * s2 + c)
            qq = np.array([np.quantile(q[:, j], level[j]) for j in range(self.dim)])
            too_big = qq > self.q_target         # need larger kappa
            lo = np.where(too_big, mid, lo)
            hi = np.where(too_big, hi, mid)
        self.kappa = np.exp(0.5 * (lo + hi))
        self.history.append(self.kappa.copy())


class GNSSACICalibrator:
    """One joint scale for a vector GNSS innovation (proposal §3.5).

    Recompute NIS(kappa)=r.T inv(C+kappa R) r from stored *pre-update*
    innovations and projected state covariances. The target is chi-square(dim),
    equivalently NIS/dim against chi-square(dim)/dim, never scalar chi-square(1).
    observe() affects the NEXT measurement. Call only for healthy eligible fixes;
    rejected, excluded and outage epochs must not enter its reference buffer.
    """

    def __init__(self, dim=3, alpha=.05, gamma=.01, window=300,
                 min_samples=30, update_every=5, kappa_bounds=(.01, 1e4)):
        self.dim, self.alpha, self.gamma = dim, alpha, gamma
        self.alpha_t = alpha
        self.kappa = 1.0
        self.q_target = float(chi2.ppf(1-alpha, dim))
        self.buf = deque(maxlen=window)
        self.min_samples, self.update_every = min_samples, update_every
        self.kb = kappa_bounds
        self.n = 0

    @staticmethod
    def _nis(r, R, C, kappa):
        return np.einsum('...i,...i->...', r, np.linalg.solve(C+kappa*R, r[..., None])[..., 0])

    def effective_cov(self, nominal):
        return self.kappa * np.asarray(nominal)

    def observe(self, residual, nominal, projected):
        r, R, C = (np.asarray(x, float) for x in (residual, nominal, projected))
        if r.shape != (self.dim,) or R.shape != (self.dim, self.dim) or C.shape != R.shape:
            raise ValueError('GNSS ACI vector/covariance shape mismatch')
        if not all(np.isfinite(x).all() for x in (r, R, C)):
            raise ValueError('finite GNSS ACI inputs required')
        # Cholesky also rejects a non-positive nominal measurement covariance.
        np.linalg.cholesky(R)
        err = float(self._nis(r, R, C, self.kappa)) > self.q_target
        self.alpha_t = float(np.clip(self.alpha_t+self.gamma*(self.alpha-err), .001, .5))
        self.buf.append((r.copy(), R.copy(), C.copy()))
        self.n += 1
        if len(self.buf) < self.min_samples or self.n % self.update_every:
            return
        rs, Rs, Cs = (np.asarray(x) for x in zip(*self.buf))
        lo, hi = np.log(self.kb)
        for _ in range(30):
            mid = (lo+hi)/2
            q = np.quantile(self._nis(rs, Rs, Cs, np.exp(mid)), 1-self.alpha_t)
            if q > self.q_target:
                lo = mid
            else:
                hi = mid
        self.kappa = float(np.exp((lo+hi)/2))
