import numpy as np

from ccgins import learning
from ccgins.data.kitti import segments_from_npz
from ccgins.kitti_scenarios import outage_schedule
from ccgins.pipeline import Config, run
from ccgins.scenarios import make_scenario

HYB = dict(filter="inekf", pseudo="hybrid", corr_scale=(4.0, 4.0, 4.0), aci=True, aci_gamma=0.05,
           aci_window=100)


def _run(cfg, fwd_bias=0.0):
    sc = make_scenario(5, duration=120, outages=[(70, 20)])
    idx = np.arange(learning.WINDOW, len(sc.truth.t), 10)
    rng = np.random.default_rng(0)
    mu = sc.truth.v_body[idx - 1] + 0.1 * rng.standard_normal((len(idx), 3))
    mu[:, 0] += fwd_bias
    var = np.full_like(mu, 0.01)
    return run(sc.truth, sc.imu, sc.gnss, sc.imu_spec, sc.gnss_spec, cfg, pred=(idx, mu, var))


def test_hybrid_kappa_fallback_equals_nhc():
    a = _run(Config(**HYB, hybrid_max_kappa=0.0))   # always falls back
    b = _run(Config(filter="inekf", pseudo="nhc"))
    assert a.extra["fallback_frac"] == 1.0
    assert np.allclose(a.p_est, b.p_est, atol=1e-9)


def test_bias_monitor_keeps_good_and_drops_biased_forward():
    good = _run(Config(**HYB, hybrid_bias_z=4.0, hybrid_bias_latch=True))
    bad = _run(Config(**HYB, hybrid_bias_z=4.0, hybrid_bias_latch=True), fwd_bias=3.0)
    nobm = _run(Config(**HYB), fwd_bias=3.0)
    assert good.extra["fallback_frac"] < 0.4         # only the warm-up (first residuals)
    assert bad.extra["fallback_frac"] > 0.6          # bias detected, latched
    e = lambda lg: np.linalg.norm(lg.p_est[:, :2] - lg.p_true[:, :2], axis=1).max()
    assert e(bad) < e(nobm)                          # monitor protects the outage


def _write_npz(path):
    n = 12000
    t = 1e9 + np.arange(n) * 0.01
    t[6000:] += 2.0                                  # real dropout -> split
    rows = np.zeros((n, 30))
    lat0, v = np.deg2rad(49.0), 10.0
    east = v * (np.arange(n) * 0.01)
    rows[:, 0] = 49.0
    rows[:, 1] = 8.4 + np.rad2deg(east / (6378137.0 * np.cos(lat0)))
    rows[:, 7] = v                                   # ve
    rows[:, 8] = v                                   # vf
    rows[:, 13] = 9.81                               # az
    rows[:, 19] = 0.001                              # wz
    rows[9000:9010, 1] += 1e-3                       # GT glitch (~70 m jump) -> cut
    np.savez(path, rows=rows, t=t)


def test_segments_from_npz(tmp_path):
    p = tmp_path / "d.npz"
    _write_npz(p)
    segs = segments_from_npz(p, min_len_s=10.0)
    assert len(segs) == 3                            # gap split + glitch split
    assert np.isclose(segs[0].t[-1], 59.99)
    s = segs[0]
    assert np.allclose(s.v_body, [v for v in [[10.0, 0, 0]] * len(s.t)], atol=1e-9)
    assert np.allclose(s.accel[:, 2], 9.81) and np.allclose(s.gyro[:, 2], 0.001)
    assert len(segments_from_npz(p, min_len_s=10.0, gt_check=False)) == 2


def test_outage_schedule():
    assert outage_schedule(407) == [(60.0, 30.0), (130.0, 120.0), (290.0, 60.0)]
    assert outage_schedule(122) == [(60.0, 30.0)]
    assert outage_schedule(80) == []
