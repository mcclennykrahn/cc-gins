"""Causal chain tests use synthetic IMU/GNSS; no test-set model selection."""
from dataclasses import replace
import copy

import numpy as np
import pytest
import torch

from ccgins.learning import MotionNet, MotionNetTCN
from ccgins.pipeline import Config, run
from ccgins.scenarios import make_scenario
from ccgins.stream_learning import LearnedStreamingNavigator


def model(tcn=False):
    torch.set_num_threads(1)
    torch.manual_seed(42)
    return (MotionNetTCN() if tcn else MotionNet()).eval()


def drive(sc, cfg, net, delay=.05, lag=.03, stop=None, nav=None):
    nav = nav or LearnedStreamingNavigator.from_truth(sc.truth, sc.imu_spec, sc.gnss_spec,
                                                       cfg, model=net, delay=delay)
    js = list(np.flatnonzero(sc.gnss.valid & (sc.gnss.idx > 0)))
    j = 0
    for k, t in enumerate(sc.truth.t):
        if stop is not None and t > stop:
            break
        while j < len(js) and sc.gnss.t[js[j]]+lag <= t+1e-9:
            m = js[j]
            nav.push_gnss(sc.gnss.t[m], sc.gnss.pos[m], sc.gnss.sigma**2)
            j += 1
        nav.push_imu(t, sc.imu.accel[k], sc.imu.gyro[k])
    if stop is None:
        while j < len(js):
            m = js[j]
            nav.push_gnss(sc.gnss.t[m], sc.gnss.pos[m], sc.gnss.sigma**2)
            j += 1
        nav.flush()
    return nav


@pytest.mark.parametrize('tcn', [False, True])
@pytest.mark.parametrize('pseudo', ['learned_vel', 'learned_nhc'])
def test_delayed_stream_matches_batch_semantics(tcn, pseudo):
    sc = make_scenario(710, duration=4.)
    sc.gnss.valid[sc.gnss.idx == 0] = False
    cfg = Config(pseudo=pseudo, aci=True, gnss_aci=True, monitor='dual', cusum_h=1e9,
                 corr_scale=(3., 3., 3.))
    net = model(tcn)
    batch = run(sc.truth, sc.imu, sc.gnss, sc.imu_spec, sc.gnss_spec, cfg, model=net,
                log_covariance=True)
    nav = drive(sc, cfg, net)
    # Same window epochs and cached historical attitude; delay only changes availability.
    np.testing.assert_allclose(nav.get_state().position, batch.p_est[-1], atol=1e-10)
    np.testing.assert_allclose(nav.get_state().covariance, batch.extra['covariance'][-1], atol=1e-10)
    np.testing.assert_allclose([r['q'] for r in nav.gnss_decisions], batch.stat_q, atol=1e-10)
    assert nav.decisions[0]['t'] == 1.
    assert len(nav.decisions) == 31
    assert all(r['accepted'] for r in nav.decisions)


def test_future_cannot_change_emitted_decisions():
    sc = make_scenario(711, duration=5.)
    cfg = Config(pseudo='learned_vel', aci=True, monitor='dual')
    a = drive(sc, cfg, model(True), stop=3.)
    decisions = copy.deepcopy(a.decisions)
    state = a.get_state()
    changed = copy.deepcopy(sc)
    changed.imu.accel[sc.truth.t > 3.] += 100
    changed.gnss.pos[sc.gnss.t > 3.] += 10000
    b = drive(changed, cfg, model(True), stop=3.)
    assert decisions == b.decisions
    np.testing.assert_array_equal(state.covariance, b.get_state().covariance)
    # A continuation changes the new trajectory, never mutates prior snapshots.
    for k in np.flatnonzero(sc.truth.t > 3.):
        a.push_imu(sc.truth.t[k], changed.imu.accel[k], changed.imu.gyro[k])
    a.flush()
    assert a.decisions[:len(decisions)] == decisions
    np.testing.assert_array_equal(state.covariance, b.get_state().covariance)


def test_both_aci_states_freeze_during_outage():
    sc = make_scenario(712, duration=39., outages=[(34., 5.)])
    cfg = Config(pseudo='learned_vel', aci=True, gnss_aci=True, gnss_gate=1e20)
    nav = drive(sc, cfg, model(), stop=35.6)
    assert nav.cal._n > 0 and nav.gnss_cal.n > 0
    before = (nav.cal._n, nav.cal.kappa.copy(), nav.cal.alpha_t.copy(),
              nav.gnss_cal.n, nav.gnss_cal.kappa, nav.gnss_cal.alpha_t)
    for k in np.flatnonzero(sc.truth.t > 35.6):
        nav.push_imu(sc.truth.t[k], sc.imu.accel[k], sc.imu.gyro[k])
    nav.flush()
    after = (nav.cal._n, nav.cal.kappa, nav.cal.alpha_t,
             nav.gnss_cal.n, nav.gnss_cal.kappa, nav.gnss_cal.alpha_t)
    for x, y in zip(before, after):
        np.testing.assert_array_equal(x, y)
    assert not any(r['eligible'] for r in nav.decisions if r['t'] >= 35.6)
    assert nav.decisions[-1]['effective_variance'][0] > nav.cal.kappa[0]*nav.decisions[-1]['variance'][0]


def test_dual_cusum_excludes_precise_malicious_epoch():
    sc = make_scenario(713, duration=6.)
    cfg = Config(pseudo='learned_vel', aci=True, gnss_aci=True, monitor='dual',
                 cusum_h=50., dr_reset_period=30.)
    clean = drive(sc, cfg, model())
    assert clean.alarm_time is None
    attacked = copy.deepcopy(sc)
    attacked.gnss.pos[attacked.gnss.t >= 3.] += [10000., 0., 0.]
    bad = drive(attacked, cfg, model())
    assert bad.alarm_time == 3.
    assert all(r['accepted'] and not r['excluded'] for r in bad.gnss_decisions if r['t'] < 3.)
    assert all(r['excluded'] and not r['accepted'] and not r['eligible']
               for r in bad.gnss_decisions if r['t'] >= 3.)
    # Excluded attack acts like missing GNSS; healthy-reference age legitimately
    # differs from the clean run and inflates DR pseudo covariance during outage.
    missing = copy.deepcopy(sc)
    missing.gnss.valid[missing.gnss.t >= 3.] = False
    absent = drive(missing, cfg, model())
    np.testing.assert_array_equal(absent.guard.P, bad.guard.P)
    np.testing.assert_array_equal(absent.guard.p, bad.guard.p)
    assert all(not r['eligible'] for r in bad.decisions if r['t'] > 3.)


def test_rejects_late_offgrid_and_unsupported():
    sc = make_scenario(714, duration=2.)
    cfg = Config(pseudo='learned_vel')
    nav = drive(sc, cfg, model())
    with pytest.raises(ValueError, match='before its epoch'):
        nav.push_gnss(1., [0, 0, 0], 1.)
    with pytest.raises(ValueError, match='align'):
        nav.push_gnss(2.003, [0, 0, 0], 1.)
    for option in [dict(pseudo='hybrid'), dict(zupt=True), dict(adaptive_r=True)]:
        with pytest.raises(ValueError):
            LearnedStreamingNavigator.from_truth(sc.truth, sc.imu_spec, sc.gnss_spec,
                replace(cfg, **option), model=model())
