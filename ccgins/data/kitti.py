"""KITTI raw OXTS loader (for running MotionNet / filters on real data).

Each oxts/data/*.txt line has 30 fields:
  0 lat 1 lon 2 alt 3 roll 4 pitch 5 yaw 6 vn 7 ve 8 vf 9 vl 10 vu 11 ax 12 ay 13 az
  14 af 15 al 16 au 17 wx 18 wy 19 wz 20 wf 21 wl 22 wu 23 pos_acc 24 vel_acc 25 navstat ...
Body frame f/l/u matches ccgins' FLU convention. af/al/au are specific force (gravity
included), matching ccgins' accelerometer model. Note: sync sequences are 10 Hz while MotionNet
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
        "accel": rows[:, 14:17],     # specific force, body f/l/u
        "gyro": rows[:, 20:23],      # rad/s, body f/l/u
        "imu6": np.concatenate([rows[:, 20:23], rows[:, 14:17]], 1),
    }
