"""Loader for i2Nav-WHU GNSS/INS text formats (awesome-gins-datasets, KF-GINS demo).

Formats (all times = GPS seconds of week):
  IMU  *.txt : t, dtheta_xyz [rad], dvel_xyz [m/s]   (increments over the preceding interval, body FRD)
  GNSS *.pos : t, lat [deg], lon [deg], h [m], std_lat, std_lon, std_h [m]   (antenna position)
  truth .nav : week, t, lat, lon, h, vN, vE, vD [m/s], roll, pitch, yaw [deg]   (IMU centre, FRD->NED)

Conversion to this repo's conventions (nav = local ENU, body = FLU):
  * body FRD -> FLU:  D = diag(1, -1, -1)          (applied to increments and lever arm)
  * nav  NED -> ENU:  T = [[0,1,0],[1,0,0],[0,0,-1]]; R_enu_flu = T R_ned_frd D
  * geodetic -> ECEF -> ENU at a fixed origin (WGS-84)
  * IMU resampled to a uniform 100 Hz grid by interpolating the CUMULATIVE increments (preserves
    the integral, handles jitter/drops), grid aligned so GNSS whole seconds fall on grid points.
  * truth interpolated to the grid (linear pos/vel, SLERP attitude).
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from ..sim import Truth

A_WGS = 6378137.0
F_WGS = 1 / 298.257223563
E2 = F_WGS * (2 - F_WGS)
D_FRD2FLU = np.diag([1.0, -1.0, -1.0])
T_NED2ENU = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
OMEGA_IE = 7.292115e-5


def geodetic_to_ecef(lat_deg, lon_deg, h):
    lat, lon = np.radians(lat_deg), np.radians(lon_deg)
    N = A_WGS / np.sqrt(1 - E2 * np.sin(lat) ** 2)
    return np.stack([(N + h) * np.cos(lat) * np.cos(lon), (N + h) * np.cos(lat) * np.sin(lon),
                     (N * (1 - E2) + h) * np.sin(lat)], axis=-1)


def ecef_to_enu_matrix(lat0_deg, lon0_deg):
    la, lo = np.radians(lat0_deg), np.radians(lon0_deg)
    return np.array([[-np.sin(lo), np.cos(lo), 0.0],
                     [-np.sin(la) * np.cos(lo), -np.sin(la) * np.sin(lo), np.cos(la)],
                     [np.cos(la) * np.cos(lo), np.cos(la) * np.sin(lo), np.sin(la)]])


def geodetic_to_enu(lat, lon, h, origin):
    lat0, lon0, h0 = origin
    x = geodetic_to_ecef(lat, lon, h) - geodetic_to_ecef(lat0, lon0, h0)
    return x @ ecef_to_enu_matrix(lat0, lon0).T


def local_gravity(lat_deg, h=0.0):
    """Normal gravity (Somigliana + free-air), m/s^2."""
    s2 = np.sin(np.radians(lat_deg)) ** 2
    g0 = 9.7803253359 * (1 + 0.00193185265241 * s2) / np.sqrt(1 - E2 * s2)
    return g0 - 3.086e-6 * h


def earth_rate_enu(lat_deg):
    la = np.radians(lat_deg)
    return np.array([0.0, OMEGA_IE * np.cos(la), OMEGA_IE * np.sin(la)])


def euler_ned_to_R_enu_flu(roll_deg, pitch_deg, yaw_deg):
    R_ned_frd = Rotation.from_euler("ZYX", np.stack([yaw_deg, pitch_deg, roll_deg], -1),
                                    degrees=True).as_matrix()
    return T_NED2ENU @ R_ned_frd @ D_FRD2FLU


def read_imu_increments(path):
    d = np.loadtxt(path)
    return d[:, 0], d[:, 1:4], d[:, 4:7]


def resample_increments(t, dth, dv, grid):
    """Increments over (t[k-1], t[k]] -> mean rates on grid intervals (grid[k], grid[k+1]].
    Returns (len(grid), 3) arrays; row k = rate over (grid[k], grid[k+1]] (last row repeated)."""
    cth = np.vstack([np.zeros(3), np.cumsum(dth[1:], axis=0)])
    cv = np.vstack([np.zeros(3), np.cumsum(dv[1:], axis=0)])
    Cth = np.stack([np.interp(grid, t, cth[:, i]) for i in range(3)], 1)
    Cv = np.stack([np.interp(grid, t, cv[:, i]) for i in range(3)], 1)
    dt = np.diff(grid)[:, None]
    w = np.diff(Cth, axis=0) / dt
    f = np.diff(Cv, axis=0) / dt
    return np.vstack([w, w[-1:]]), np.vstack([f, f[-1:]])


@dataclass
class I2NavSequence:
    name: str
    truth: Truth            # truth on the 100 Hz grid (gyro/accel fields = MEASURED IMU, FLU)
    gnss_t: np.ndarray      # GNSS epoch times relative to grid start
    gnss_idx: np.ndarray    # grid index of each epoch
    gnss_pos: np.ndarray    # antenna ENU [m]
    gnss_std: np.ndarray    # (M,3) reported std, ENU order [m]
    lever_flu: np.ndarray   # antenna lever arm, FLU [m]
    origin: tuple           # (lat, lon, h)
    t0_sow: float
    g_local: float
    w_ie_enu: np.ndarray


def load_sequence(imu_path, truth_path, gnss_path, lever_frd, name="", rate=100.0,
                  t_start=None, t_end=None) -> I2NavSequence:
    ti, dth, dv = read_imu_increments(imu_path)
    nav = np.loadtxt(truth_path)
    gp = np.loadtxt(gnss_path)
    tn = nav[:, 1]
    lo = max(ti[0], tn[0], gp[0, 0] - 1.0) if t_start is None else t_start
    hi = min(ti[-1], tn[-1]) if t_end is None else t_end
    dt = 1.0 / rate
    g0 = np.ceil(lo / dt) * dt + dt
    grid = np.arange(g0, hi - dt, dt)
    grid = np.round(grid / dt) * dt
    w_frd, f_frd = resample_increments(ti, dth, dv, grid)
    gyro = w_frd @ D_FRD2FLU
    accel = f_frd @ D_FRD2FLU

    origin = (nav[0, 2], nav[0, 3], nav[0, 4])
    p_all = geodetic_to_enu(nav[:, 2], nav[:, 3], nav[:, 4], origin)
    v_all = np.stack([nav[:, 6], nav[:, 5], -nav[:, 7]], 1)
    R_all = euler_ned_to_R_enu_flu(nav[:, 8], nav[:, 9], nav[:, 10])
    p = np.stack([np.interp(grid, tn, p_all[:, i]) for i in range(3)], 1)
    v = np.stack([np.interp(grid, tn, v_all[:, i]) for i in range(3)], 1)
    R = Slerp(tn, Rotation.from_matrix(R_all))(grid).as_matrix()
    vb = np.einsum("nji,nj->ni", R, v)
    tt = grid - grid[0]
    truth = Truth(t=tt, R=R, v=v, p=p, v_body=vb, gyro=gyro, accel=accel, dt=dt)

    gpos = geodetic_to_enu(gp[:, 1], gp[:, 2], gp[:, 3], origin)
    k = np.round((gp[:, 0] - grid[0]) / dt).astype(int)
    on = (k >= 1) & (k < len(grid)) & (np.abs(grid[np.clip(k, 0, len(grid) - 1)] - gp[:, 0]) < 0.5 * dt)
    std = np.stack([gp[:, 5], gp[:, 4], gp[:, 6]], 1)
    lat0 = origin[0]
    return I2NavSequence(name=name, truth=truth, gnss_t=tt[k[on]], gnss_idx=k[on], gnss_pos=gpos[on],
                         gnss_std=std[on], lever_flu=D_FRD2FLU @ np.asarray(lever_frd, float),
                         origin=origin, t0_sow=float(grid[0]), g_local=float(local_gravity(lat0, origin[2])),
                         w_ie_enu=earth_rate_enu(lat0))


def save_npz(seq: I2NavSequence, path):
    tr = seq.truth
    np.savez_compressed(path, name=seq.name, t=tr.t, R=tr.R, v=tr.v, p=tr.p, v_body=tr.v_body,
                        gyro=tr.gyro, accel=tr.accel, dt=tr.dt, gnss_t=seq.gnss_t, gnss_idx=seq.gnss_idx,
                        gnss_pos=seq.gnss_pos, gnss_std=seq.gnss_std, lever_flu=seq.lever_flu,
                        origin=np.array(seq.origin), t0_sow=seq.t0_sow, g_local=seq.g_local,
                        w_ie_enu=seq.w_ie_enu)


def load_npz(path) -> I2NavSequence:
    z = np.load(path)
    tr = Truth(t=z["t"], R=z["R"], v=z["v"], p=z["p"], v_body=z["v_body"], gyro=z["gyro"],
               accel=z["accel"], dt=float(z["dt"]))
    return I2NavSequence(name=str(z["name"]), truth=tr, gnss_t=z["gnss_t"], gnss_idx=z["gnss_idx"],
                         gnss_pos=z["gnss_pos"], gnss_std=z["gnss_std"], lever_flu=z["lever_flu"],
                         origin=tuple(z["origin"]), t0_sow=float(z["t0_sow"]), g_local=float(z["g_local"]),
                         w_ie_enu=z["w_ie_enu"])


def crop(seq: I2NavSequence, t_from: float, t_to: float) -> I2NavSequence:
    """Sub-sequence [t_from, t_to) in relative seconds, re-based to start at 0."""
    tr = seq.truth
    a, b = int(round(t_from / tr.dt)), min(int(round(t_to / tr.dt)), len(tr.t))
    sl = slice(a, b)
    nt = Truth(t=tr.t[sl] - tr.t[a], R=tr.R[sl], v=tr.v[sl], p=tr.p[sl], v_body=tr.v_body[sl],
               gyro=tr.gyro[sl], accel=tr.accel[sl], dt=tr.dt)
    m = (seq.gnss_idx >= a + 1) & (seq.gnss_idx < b)
    return I2NavSequence(name=seq.name, truth=nt, gnss_t=seq.gnss_t[m] - tr.t[a], gnss_idx=seq.gnss_idx[m] - a,
                         gnss_pos=seq.gnss_pos[m], gnss_std=seq.gnss_std[m], lever_flu=seq.lever_flu,
                         origin=seq.origin, t0_sow=seq.t0_sow + tr.t[a], g_local=seq.g_local,
                         w_ie_enu=seq.w_ie_enu)
