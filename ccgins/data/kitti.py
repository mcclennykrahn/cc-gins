"""KITTI raw OXTS loader (for running MotionNet / filters on real data).

Each oxts/data/*.txt line has 30 fields:
  0 lat 1 lon 2 alt 3 roll 4 pitch 5 yaw 6 vn 7 ve 8 vf 9 vl 10 vu 11 ax 12 ay 13 az
  14 af 15 al 16 au 17 wx 18 wy 19 wz 20 wf 21 wl 22 wu 23 pos_acc 24 vel_acc 25 navstat ...
We use the VEHICLE-frame channels ax/ay/az (11-13) and wx/wy/wz (17-19): x fwd, y left, z up
= ccgins FLU body frame, specific force incl. gravity. The f/l/u channels (af.., wf..) are NOT in
the body frame (they are heading-aligned/levelled): integrating them with the OXTS attitude gives
~39 m median free-INS drift in 20 s vs ~3.8 m with ax/wx on drive 0028 (checked empirically). Note: sync sequences are 10 Hz while MotionNet
was trained on 100 Hz simulated IMU -> use *_extract (100 Hz) data or retrain with WINDOW=10.
"""
from __future__ import annotations

from datetime import datetime
from pathlib import Path

import numpy as np

EARTH_R = 6378137.0


def _parse_ts(line: str) -> float:
    line = line.strip()
    base, frac = line.split(".")
    dt = datetime.strptime(base, "%Y-%m-%d %H:%M:%S")
    return dt.timestamp() + float("0." + frac)


def load_oxts(seq_dir: str | Path) -> dict:
    """seq_dir = .../2011_09_26_drive_0001_sync (or _extract) containing oxts/."""
    seq_dir = Path(seq_dir)
    files = sorted((seq_dir / "oxts" / "data").glob("*.txt"))
    if not files:
        raise FileNotFoundError(f"no oxts data under {seq_dir}")
    rows = np.array([np.loadtxt(f) for f in files])
    ts_file = seq_dir / "oxts" / "timestamps.txt"
    if ts_file.exists():
        t = np.array([_parse_ts(s) for s in ts_file.read_text().splitlines() if s.strip()])
        t = t - t[0]
    else:
        t = np.arange(len(rows)) * 0.1
    lat0 = np.deg2rad(rows[0, 0])
    lat, lon = np.deg2rad(rows[:, 0]), np.deg2rad(rows[:, 1])
    east = (lon - lon[0]) * EARTH_R * np.cos(lat0)
    north = (lat - lat[0]) * EARTH_R
    up = rows[:, 2] - rows[0, 2]
    return {
        "t": t,
        "pos_enu": np.stack([east, north, up], 1),
        "rpy": rows[:, 3:6],
        "v_body": rows[:, 8:11],     # forward, left, up
        "accel": rows[:, 11:14],     # specific force, vehicle frame x/y/z (fwd/left/up)
        "gyro": rows[:, 17:20],      # rad/s, vehicle frame x/y/z
        "imu6": np.concatenate([rows[:, 17:20], rows[:, 11:14]], 1),
    }


def _read_rows(files) -> np.ndarray:
    rows = []
    for f in files:
        rows.append(np.array(f.read_text().split(), dtype=float)[:30])
    return np.array(rows)


def consolidate(seq_dir: str | Path, out_npz: str | Path) -> None:
    """Pack oxts/data/*.txt + timestamps into one compressed npz (raw 30 fields + absolute time)."""
    seq_dir = Path(seq_dir)
    files = sorted((seq_dir / "oxts" / "data").glob("*.txt"))
    rows = _read_rows(files)
    ts = [s for s in (seq_dir / "oxts" / "timestamps.txt").read_text().splitlines() if s.strip()]
    t = np.array([_parse_ts(s) for s in ts])
    n = min(len(t), len(rows))
    np.savez_compressed(out_npz, rows=rows[:n], t=t[:n])


def load_npz(path: str | Path) -> dict:
    d = np.load(path)
    rows, t = d["rows"], d["t"]
    return _to_dict(rows, t - t[0])


def _to_dict(rows, t):
    lat0 = np.deg2rad(rows[0, 0])
    lat, lon = np.deg2rad(rows[:, 0]), np.deg2rad(rows[:, 1])
    east = (lon - lon[0]) * EARTH_R * np.cos(lat0)
    north = (lat - lat[0]) * EARTH_R
    up = rows[:, 2] - rows[0, 2]
    return {
        "t": t,
        "pos_enu": np.stack([east, north, up], 1),
        "rpy": rows[:, 3:6],
        "v_body": rows[:, 8:11],
        "accel": rows[:, 11:14],
        "gyro": rows[:, 17:20],
        "imu6": np.concatenate([rows[:, 17:20], rows[:, 11:14]], 1),
        "pos_acc": rows[:, 23],
        "navstat": rows[:, 25] if rows.shape[1] > 25 else None,
    }


def _gt_bad_mask(p, v, thr: float = 2.0, pad: int = 500):
    """Flag samples where OXTS position and velocity disagree (|dp(1 s) - v dt| > thr m), padded
    by `pad` samples. Catches ground-truth glitches (e.g. 30 m position jumps in 2011_09_30_0028)."""
    n = len(p)
    bad = np.zeros(n, bool)
    if n <= 100:
        return bad
    e = np.linalg.norm((p[100:] - p[:-100]) - 0.5 * (v[100:] + v[:-100]), axis=1)
    for i in np.where(e > thr)[0]:
        bad[max(0, i - pad):min(n, i + 100 + pad)] = True
    return bad


def segments_from_npz(path: str | Path, min_len_s: float = 30.0, max_gap_s: float = 0.1,
                      gt_check: bool = True):
    """Split a consolidated drive into gap-free segments and return ccgins Truth objects.

    The extract OXTS stream is 100 Hz but its timestamps jitter (even negative steps), so inside a
    segment we use a uniform 0.01 s grid (sample index); real dropouts (>max_gap_s, ~1.6 s) split
    segments. Truth: R from OXTS roll/pitch/yaw (ENU, yaw 0 = east, CCW), v = (ve, vn, vu),
    v_body = R^T v,
    p = local ENU of the OXTS unit, gyro/accel = OXTS body rates / specific force.
    Spans with inconsistent OXTS position/velocity are cut out when gt_check=True.
    """
    from ..geometry import euler_to_rot
    from ..sim import Truth

    d = np.load(path)
    rows, t = d["rows"], d["t"]
    br = np.where(np.diff(t) > max_gap_s)[0]
    segs = []
    for seg in np.split(np.arange(len(t)), br + 1):
        if gt_check and len(seg) > 100:
            k = _to_dict(rows[seg], t[seg] - t[seg][0])
            v = np.stack([rows[seg, 7], rows[seg, 6], rows[seg, 10]], 1)
            bad = _gt_bad_mask(k["pos_enu"], v)
            good = np.where(~bad)[0]
            if len(good):
                cut = np.where(np.diff(good) > 1)[0]
                segs += [seg[g] for g in np.split(good, cut + 1)]
        else:
            segs.append(seg)
    out = []
    for seg in segs:
        if len(seg) * 0.01 < min_len_s:
            continue
        k = _to_dict(rows[seg], t[seg] - t[seg][0])
        n = len(seg)
        R = np.stack([euler_to_rot(*r) for r in k["rpy"]])
        r = rows[seg]
        # OXTS vf/vl/vu are in the heading-aligned *levelled* frame, not the body frame; build the
        # ENU velocity from ve/vn/vu and rotate it into the body frame.
        v = np.stack([r[:, 7], r[:, 6], r[:, 10]], 1)
        vb = np.einsum("nji,nj->ni", R, v)
        tt = np.arange(n) * 0.01
        out.append(Truth(t=tt, R=R, v=v, p=k["pos_enu"], v_body=vb,
                         gyro=k["gyro"].copy(), accel=k["accel"].copy(), dt=0.01))
    return out
