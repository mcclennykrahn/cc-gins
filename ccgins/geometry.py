"""SO(3) / SE_2(3) utilities (numpy)."""
from __future__ import annotations

import numpy as np

_EPS = 1e-9


def skew(w: np.ndarray) -> np.ndarray:
    return np.array([[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]])


def so3_exp(phi: np.ndarray) -> np.ndarray:
    th = float(np.linalg.norm(phi))
    K = skew(phi)
    if th < 1e-7:
        return np.eye(3) + K + 0.5 * K @ K
    return np.eye(3) + np.sin(th) / th * K + (1.0 - np.cos(th)) / th**2 * K @ K


def so3_log(R: np.ndarray) -> np.ndarray:
    c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    th = np.arccos(c)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if th < 1e-7:
        return 0.5 * v
    if np.pi - th < 1e-5:
        # near pi: use symmetric part
        A = (R + np.eye(3)) / 2.0
        i = int(np.argmax(np.diag(A)))
        ax = A[:, i] / np.sqrt(max(A[i, i], _EPS))
        return th * ax / np.linalg.norm(ax)
    return th / (2.0 * np.sin(th)) * v


def so3_left_jacobian(phi: np.ndarray) -> np.ndarray:
    th = float(np.linalg.norm(phi))
    K = skew(phi)
    if th < 1e-7:
        return np.eye(3) + 0.5 * K
    return (
        np.eye(3)
        + (1.0 - np.cos(th)) / th**2 * K
        + (th - np.sin(th)) / th**3 * K @ K
    )


def euler_to_rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """R_nb (body FLU -> nav ENU) from ZYX Euler angles."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1.0]])
    Ry = np.array([[cp, 0, sp], [0, 1.0, 0], [-sp, 0, cp]])
    Rx = np.array([[1.0, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def rot_to_yaw(R: np.ndarray) -> float:
    return float(np.arctan2(R[1, 0], R[0, 0]))
