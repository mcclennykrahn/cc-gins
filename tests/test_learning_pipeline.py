import numpy as np
import torch

from ccgins import learning
from ccgins.data.kitti import load_oxts
from ccgins.metrics import summarize
from ccgins.pipeline import Config, run
from ccgins.scenarios import make_scenario


def test_motionnet_shapes_and_training_reduces_loss():
    sc = make_scenario(3, duration=60)
    msgs = []
    net = learning.train_motionnet([sc.imu6()], [sc.truth.v_body], epochs=4, stride=20, log=msgs.append)
    losses = [float(m.split()[-1]) for m in msgs if "beta-NLL" in m]
    assert losses[-1] < losses[0]
    idx = np.arange(learning.WINDOW, len(sc.truth.t), 50)
    with torch.no_grad():
        mu, var = learning.predict(net, sc.imu6(), idx)
    assert mu.shape == (len(idx), 3) and var.shape == (len(idx), 3)
    assert np.all(var > 0)


def test_correlation_factor():
    rng = np.random.default_rng(0)
    white = rng.standard_normal((20000, 1))
    assert abs(learning.correlation_factor(white)[0] - 1.0) < 0.3
    # AR(1) with phi=0.9 -> n_eff = (1+phi)/(1-phi) = 19
    x = np.zeros(20000)
    e = rng.standard_normal(20000)
    for i in range(1, 20000):
        x[i] = 0.9 * x[i - 1] + e[i]
    assert 12 < learning.correlation_factor(x[:, None])[0] < 26


def _smoke(cfg, **kw):
    sc = make_scenario(5, duration=120, outages=[(60, 20)], **kw)
    idx = np.arange(learning.WINDOW, len(sc.truth.t), 10)
    rng = np.random.default_rng(0)
    mu = sc.truth.v_body[idx - 1] + 0.1 * rng.standard_normal((len(idx), 3))
    var = np.full_like(mu, 0.01)
    return run(sc.truth, sc.imu, sc.gnss, sc.imu_spec, sc.gnss_spec, cfg, pred=(idx, mu, var))


def test_pipeline_learned_aci_dual_runs():
    log = _smoke(Config(filter="inekf", pseudo="learned_vel", aci=True, corr_scale=(2, 2, 2),
                        monitor="dual", cusum_h=1e9))
    m = summarize(log, [(60, 20)], t_start=20)
    assert np.isfinite(m["rmse_h"]) and m["rmse_h"] < 10
    assert len(log.stat_g) > 50 and log.alarm_time is None


def test_dual_monitor_detects_large_spoof():
    log = _smoke(Config(filter="inekf", pseudo="learned_vel", monitor="dual", cusum_h=50),
                 spoof_start=40, spoof_rate=3.0)
    assert log.alarm_time is not None and log.alarm_time > 40


def test_kitti_loader(tmp_path):
    d = tmp_path / "seq" / "oxts" / "data"
    d.mkdir(parents=True)
    for i in range(3):
        row = np.zeros(30)
        row[0], row[1], row[2] = 49.0 + i * 1e-5, 8.4, 110.0
        row[8], row[11], row[13], row[19] = 10.0, 0.1, 9.81, 0.01
        np.savetxt(d / f"{i:010d}.txt", row[None])
    o = load_oxts(tmp_path / "seq")
    assert o["imu6"].shape == (3, 6)
    assert np.allclose(o["v_body"][:, 0], 10.0)
    assert np.allclose(o["imu6"][:, 5], 9.81) and np.allclose(o["imu6"][:, 2], 0.01)
    assert o["pos_enu"][2, 1] > 2.0
