from pathlib import Path
import numpy as np
import pytest

from ccgins import filters
from ccgins.data import i2nav as D
from ccgins.pipeline import mount_matrix


def test_euler_frames():
    R = D.euler_ned_to_R_enu_flu(0.0, 0.0, 90.0)  # heading east
    assert np.allclose(R @ [1, 0, 0], [1, 0, 0], atol=1e-12)    # forward -> East
    assert np.allclose(R @ [0, 1, 0], [0, 1, 0], atol=1e-12)    # left -> North
    assert np.allclose(R @ [0, 0, 1], [0, 0, 1], atol=1e-12)    # up -> Up
    R = D.euler_ned_to_R_enu_flu(10.0, 0.0, 0.0)  # positive roll = right side down -> left side up
    assert (R @ [0, 1, 0])[2] > 0.1
    R = D.euler_ned_to_R_enu_flu(0.0, 10.0, 0.0)  # positive pitch = nose up
    assert (R @ [1, 0, 0])[2] > 0.1


def test_resample_increments_preserves_rate():
    rng = np.random.default_rng(0)
    t = np.cumsum(0.005 + rng.uniform(-4e-4, 4e-4, 4000)) + 100.0
    w, f = np.array([0.1, -0.2, 0.3]), np.array([1.0, 2.0, -9.8])
    dt = np.diff(t, prepend=t[0] - 0.005)[:, None]
    grid = np.arange(101.0, t[-1] - 0.02, 0.01)
    W, F = D.resample_increments(t, w * dt, f * dt, grid)
    assert np.allclose(W, w, atol=1e-9) and np.allclose(F, f, atol=1e-9)


def test_geodetic_enu():
    o = (30.0, 114.0, 20.0)
    p = D.geodetic_to_enu(np.array([30.001]), np.array([114.0]), np.array([20.0]), o)[0]
    assert abs(p[0]) < 1e-3 and 110.0 < p[1] < 111.5 and abs(p[2]) < 0.01
    assert 9.79 < D.local_gravity(30.5, 23.0) < 9.795


def _write_synthetic(tmp_path, T=20.0):
    sow0 = 1000.0
    ti = sow0 + np.arange(0, T, 0.005)
    lat0, lon0, h0, v = 30.0, 114.0, 20.0, 10.0
    # level drive due East at 10 m/s; FRD increments: accel only gravity reaction (-g in D -> -g on z_FRD)
    g = D.local_gravity(lat0, h0)
    imu = np.zeros((len(ti), 7))
    imu[:, 0] = ti
    imu[:, 6] = -g * 0.005
    np.savetxt(tmp_path / "imu.txt", imu)
    tn = sow0 + np.arange(0, T, 0.005)
    E = v * (tn - sow0)
    lat = lat0 + 0 * tn
    lon = lon0 + np.degrees(E / (D.A_WGS * np.cos(np.radians(lat0))))
    nav = np.c_[np.zeros_like(tn), tn, lat, lon, np.full_like(tn, h0), 0 * tn, np.full_like(tn, v), 0 * tn,
                0 * tn, 0 * tn, np.full_like(tn, 90.0)]
    np.savetxt(tmp_path / "truth.nav", nav)
    tg = sow0 + np.arange(1.0, T - 1)
    lev_frd = np.array([0.5, 0.2, -1.0])  # antenna: 0.5 fwd, 0.2 right, 1.0 up
    Eg = v * (tg - sow0) + 0.5           # fwd = East
    gp = np.c_[tg, lat0 - np.degrees(0.2 / D.A_WGS) + 0 * tg,
               lon0 + np.degrees(Eg / (D.A_WGS * np.cos(np.radians(lat0)))), np.full_like(tg, h0 + 1.0),
               np.full((len(tg), 3), 0.01)]
    np.savetxt(tmp_path / "gnss.pos", gp)
    return lev_frd


def test_load_sequence_synthetic(tmp_path):
    lev = _write_synthetic(tmp_path)
    s = D.load_sequence(tmp_path / "imu.txt", tmp_path / "truth.nav", tmp_path / "gnss.pos", lev, name="syn")
    tr = s.truth
    assert tr.dt == pytest.approx(0.01)
    assert np.allclose(tr.v_body[5:-5], [10, 0, 0], atol=1e-6)
    assert np.allclose(tr.accel[5:-5], [0, 0, s.g_local], atol=1e-6)
    assert np.allclose(s.lever_flu, [0.5, -0.2, 1.0])
    # GNSS epochs map onto the grid at the right time and the lever arm closes the geometry (cm level)
    assert np.allclose(tr.t[s.gnss_idx], s.gnss_t)
    ant = tr.p[s.gnss_idx] + np.einsum("nij,j->ni", tr.R[s.gnss_idx], s.lever_flu)
    assert np.abs(ant - s.gnss_pos).max() < 0.02
    c = D.crop(s, 5.0, 15.0)
    assert c.truth.t[0] == 0 and len(c.truth.t) == 1000 and np.all(c.gnss_idx >= 1)


def test_earth_model_default_off_and_stationary():
    R, v, p = np.eye(3), np.array([1.0, 2.0, 0.0]), np.zeros(3)
    w, f = np.array([0.01, 0.0, 0.02]), np.array([0.1, 0.0, 9.81])
    filters.set_earth()
    R1, v1, p1 = filters.mechanize(R, v, p, w, f, 0.01)
    assert np.allclose(v1, v + (R @ f + np.array([0, 0, -9.81])) * 0.01)
    lat = 30.5
    wie, g = D.earth_rate_enu(lat), D.local_gravity(lat)
    filters.set_earth(wie, g)
    try:
        R, v, p = np.eye(3), np.zeros(3), np.zeros(3)
        for _ in range(6000):  # 60 s stationary, IMU senses earth rate + gravity reaction
            R, v, p = filters.mechanize(R, v, p, wie, np.array([0, 0, g]), 0.01)
        assert np.linalg.norm(v) < 1e-6 and np.abs(R - np.eye(3)).max() < 1e-9
    finally:
        filters.set_earth()


def test_mount_matrix():
    pitch, yaw = np.radians(0.5), np.radians(2.0)
    fwd_b = np.array([np.cos(pitch) * np.cos(yaw), np.cos(pitch) * np.sin(yaw), np.sin(pitch)])
    assert np.allclose(mount_matrix(pitch, yaw) @ fwd_b, [1, 0, 0], atol=1e-12)


def test_scenarios_fold_and_outages():
    from ccgins import i2nav_scenarios as I
    trA, teA = I.fold_ranges("A")          # (train, test); fold A tests on the first half
    assert teA[0] == 0.0 and teA[1] < trA[0] and trA[0] - teA[1] >= I.GUARD - 1e-9
    trB, teB = I.fold_ranges("B")
    assert trB == teA and teB == trA       # folds swap halves, no overlap
    outs = I.outage_schedule(800.0, first=50.0)
    assert [d for _, d in outs] == [30, 120, 60, 30, 120, 60]
    assert all(s2 >= s1 + d1 for (s1, d1), (s2, _) in zip(outs, outs[1:]))
    assert outs[-1][0] + outs[-1][1] <= 800.0


@pytest.mark.skipif(not Path("data/i2nav/ADIS16460.npz").exists(), reason="i2Nav npz not prepared (raw data not in repo)")
def test_mount_calibration_real_adis16460():
    """Truth-free mounting calibration on a real 400 s segment: yaw ~2.8 deg (truth-consistent), and the
    correlation-aware NHC sigma is much larger than the white-noise RMS."""
    from ccgins import i2nav_scenarios as I
    (p, y), sig, sig_eff, neff = I.calibrate_mount(I.segment("ADIS16460", 0.0, 400.0), return_eff=True)
    assert 2.3 < np.degrees(y) < 3.3 and abs(np.degrees(p)) < 1.5
    assert max(sig) < 0.15 and sig_eff > 2 * max(sig) and min(neff) >= 1.0
