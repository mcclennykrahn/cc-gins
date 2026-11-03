"""Evaluation metrics: RMSE, outage errors, NEES and 95% coverage."""
from __future__ import annotations

import numpy as np

CHI2_2_95 = 5.991464547107979


def horizontal_errors(log):
    return log.p_est[:, :2] - log.p_true[:, :2]


def summarize(log, outages, t_start: float = 60.0) -> dict:
    e = log.p_est - log.p_true
    eh = np.linalg.norm(e[:, :2], axis=1)
    m = log.t >= t_start
    out = {
        "rmse_h": float(np.sqrt(np.mean(eh[m] ** 2))),
        "rmse_3d": float(np.sqrt(np.mean(np.sum(e[m] ** 2, axis=1)))),
        "runtime_us": log.runtime_per_step_us,
    }
    # consistency (position, nav frame)
    Ph = log.P_pos[:, :2, :2]
    nees_h = np.einsum("ni,nij,nj->n", e[:, :2], np.linalg.inv(Ph), e[:, :2])
    nees_3 = np.einsum("ni,nij,nj->n", e, np.linalg.inv(log.P_pos), e)
    out["anees_pos"] = float(np.mean(nees_3[m]) / 3.0)
    out["coverage95_h"] = float(np.mean(nees_h[m] <= CHI2_2_95))
    in_out = np.zeros_like(m)
    for (s, d) in outages:
        w = (log.t >= s) & (log.t < s + d)
        in_out |= w
        out[f"max_err_outage_{int(d)}s"] = float(eh[w].max())
        out[f"end_err_outage_{int(d)}s"] = float(eh[w][-1])
    if in_out.any():
        out["coverage95_h_outage"] = float(np.mean(nees_h[in_out] <= CHI2_2_95))
        out["anees_pos_outage"] = float(np.mean(nees_3[in_out]) / 3.0)
    return out
