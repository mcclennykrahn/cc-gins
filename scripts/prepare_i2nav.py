"""Convert awesome-gins-datasets (i2Nav-WHU) raw text files to data/i2nav/<imu>.npz (gitignored).

Usage: python scripts/prepare_i2nav.py <extracted_dir>   (dir with <IMU>/<IMU>.txt, <IMU>/truth.nav, GNSS_RTK.pos)
Raw data: https://github.com/i2Nav-WHU/awesome-gins-datasets (*.7z, extract with py7zr).
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ccgins.data.i2nav import load_sequence, save_npz  # noqa: E402

# antenna lever arms (IMU -> GNSS antenna phase centre), FRD [m], from the dataset README
LEVER_FRD = {"ICM20602": (-0.073, 0.302, 0.087), "ADIS16460": (0.045, 0.46, -0.238),
             "ADIS16465": (-0.073, 0.302, 0.087), "i300": (-0.075, 0.46, -0.218)}

if __name__ == "__main__":
    src = Path(sys.argv[1] if len(sys.argv) > 1 else "/tmp/ag")
    out = Path(__file__).resolve().parents[1] / "data" / "i2nav"
    out.mkdir(parents=True, exist_ok=True)
    for name, lev in LEVER_FRD.items():
        seq = load_sequence(src / name / f"{name}.txt", src / name / "truth.nav", src / "GNSS_RTK.pos", lev, name=name)
        save_npz(seq, out / f"{name}.npz")
        tr = seq.truth
        print(f"{name}: N={len(tr.t)} T={tr.t[-1]:.1f}s gnss={len(seq.gnss_idx)} g={seq.g_local:.4f} "
              f"speed max {np.linalg.norm(tr.v, axis=1).max():.1f} m/s")
