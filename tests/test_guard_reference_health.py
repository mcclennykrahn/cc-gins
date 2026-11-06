"""Opt-in whole-reset-interval guard-reference health (V0.8 Stage 3)."""
from dataclasses import replace

import numpy as np
import pytest
import torch

from ccgins.integrity import GuardReferenceHealth
from ccgins.learning import load
from ccgins.pipeline import Config, run
from ccgins.scenarios import make_scenario
from ccgins.stream_learning import LearnedStreamingNavigator


def test_clean_whole_interval_and_sticky_rejection():
    h = GuardReferenceHealth(max_age=1.5)
    assert h.observe(0.1, accepted=True, q=1.) and h.healthy
    assert h.observe(1.1, accepted=True, q=1.) and h.healthy
    # One rejected reference permanently poisons this interval ...
    assert not h.observe(2.1, accepted=False, q=1.)
    assert h.reasons == ["rejected"]
    # ... and a later good fix can never erase that history.
    assert not h.observe(3.1, accepted=True, q=1.)
    assert h.reasons == ["rejected"]
    assert h.summary()["healthy"] is False


def test_interruption_stale_and_missing_intervals():
    h = GuardReferenceHealth(max_age=1.5)
    h.observe(0.1, accepted=True, q=1.)
    assert h.check_fresh(1.2)
    assert not h.check_fresh(2.0)
    assert "stale" in h.reasons
    h.begin(10.0)
    assert h.check_fresh(11.0)  # fresh but pending, no fix yet
    assert not h.check_fresh(12.0)
    assert "missing" in h.reasons
    assert h.unhealthy_intervals == 1 and "stale" in h.history_reasons


def test_forced_alarm_recovery_and_nonfinite_reasons():
    h = GuardReferenceHealth()
    assert not h.observe(1., accepted=True, q=1., forced=True)
    assert h.reasons == ["forced"]
    h.begin(10.)
    assert not h.observe(11., accepted=True, q=1., alarm=True)
    assert h.reasons == ["alarm"]
    h.begin(20.)
    assert not h.observe(21., accepted=False, q=1., alarm=True, recovery=True)
    assert h.reasons == ["alarm", "recovery", "rejected"]
    h.begin(30.)
    assert not h.observe(31., accepted=True, q=np.nan)
    assert h.reasons == ["nonfinite"]


def test_alarm_or_recovery_epoch_never_advances_health():
    h = GuardReferenceHealth()
    h.observe(1., accepted=True, q=1.)
    assert h.last_fix_t == 1.0
    h.observe(2., accepted=True, q=1., alarm=True)
    assert h.last_fix_t == 1.0
    h.begin(10.)
    h.observe(11., accepted=True, q=1., recovery=True)
    assert h.last_fix_t is None and h.accepted == 0


def test_new_scheduled_interval_clears_only_on_begin():
    h = GuardReferenceHealth()
    h.observe(1., accepted=False, q=1.)
    assert not h.healthy
    h.begin(30.)
    assert h.healthy and h.reset_skips == 0
    h.observe(31., accepted=True, q=1.)
    assert h.healthy and h.last_fix_t == 31.
    h.resets += 1
    assert h.summary()["resets"] == 1


@pytest.mark.parametrize("max_age", [0., -1., np.nan, np.inf])
def test_invalid_max_age(max_age):
    with pytest.raises(ValueError):
        GuardReferenceHealth(max_age=max_age)


def _feed_stream(sc, cfg, net):
    nav = LearnedStreamingNavigator.from_truth(sc.truth, sc.imu_spec, sc.gnss_spec,
                                               cfg, model=net)
    j = 0
    for k, t in enumerate(sc.truth.t):
        while j < len(sc.gnss.t) and sc.gnss.t[j] <= t + 1e-9:
            if sc.gnss.valid[j]:
                nav.push_gnss(sc.gnss.t[j], sc.gnss.pos[j], sc.gnss.sigma**2)
            j += 1
        nav.push_imu(t, sc.imu.accel[k], sc.imu.gyro[k])
    nav.flush()
    return nav


def _cfg(**kw):
    base = dict(pseudo='hybrid', aci=True, monitor='dual', recovery_epochs=3,
                recovery_q=3., corr_scale=(40., 40., 40.), robust_gnss_delta=1.5)
    base.update(kw)
    return Config(**base)


def test_loaded_checkpoint_defaults_and_healthy_noop():
    torch.set_num_threads(1)
    net = load('results/motionnet.pt')
    sc = make_scenario(500, duration=40.)
    off = run(sc.truth, sc.imu, sc.gnss, sc.imu_spec, sc.gnss_spec, _cfg(),
              model=net, log_covariance=True)
    on = run(sc.truth, sc.imu, sc.gnss, sc.imu_spec, sc.gnss_spec,
             _cfg(guard_ref_health=True), model=net, log_covariance=True)
    assert off.extra['guard_ref_health'] is None
    assert off.extra['guard_ref_fallback_epochs'] == 0
    # A wholly healthy clean interval changes nothing: same reseeds, no fallback.
    assert on.extra['guard_ref_fallback_epochs'] == 0
    assert on.extra['guard_ref_health']['reset_skips'] == 0
    assert on.extra['guard_ref_health']['resets'] == 1
    assert np.max(np.abs(off.p_est - on.p_est)) == 0.0
    assert np.max(np.abs(off.extra['covariance'][-1] -
                         on.extra['covariance'][-1])) == 0.0


def test_loaded_checkpoint_outage_batch_stream_parity():
    torch.set_num_threads(1)
    net = load('results/motionnet.pt')
    sc = make_scenario(500, duration=50., outages=[(10., 4.)])
    cfg = _cfg(guard_ref_health=True)
    batch = run(sc.truth, sc.imu, sc.gnss, sc.imu_spec, sc.gnss_spec, cfg,
                model=net, log_covariance=True)
    nav = _feed_stream(sc, cfg, net)
    summary = batch.extra['guard_ref_health']
    assert batch.extra['guard_ref_fallback_epochs'] > 0
    assert summary['reset_skips'] >= 1
    assert 'stale' in summary['history_reasons']
    assert nav.guard_ref_fallbacks == batch.extra['guard_ref_fallback_epochs']
    assert np.max(np.abs(nav.get_state().position - batch.p_est[-1])) < 1e-9
    assert np.max(np.abs(nav.get_state().covariance -
                         batch.extra['covariance'][-1])) < 1e-9


def test_loaded_checkpoint_alarmed_boundary_keeps_fixed_nhc():
    # Sensitive thresholds exercise latch semantics, not detection performance.
    torch.set_num_threads(1)
    net = load('results/motionnet.pt')
    sc = make_scenario(500, duration=33.)
    cfg = Config(guard_ref_health=True, cusum_h=.001, cusum_k=0.,
                 pseudo='learned_vel', monitor='dual')
    nav = _feed_stream(sc, cfg, net)
    window = [d for d in nav.decisions if 30. < d['t'] <= 31.]
    assert window and nav.alarm_time < 30.
    assert all(d['alarm'] and not d['guard_ref_healthy'] and
               d['guard_ref_fallback'] and len(d['effective_variance']) == 2
               for d in window)
    assert nav.guard_ref.resets == 0
    batch = run(sc.truth, sc.imu, sc.gnss, sc.imu_spec, sc.gnss_spec,
                cfg, model=net, log_covariance=True)
    assert batch.extra['guard_ref_health']['resets'] == 0
    assert batch.extra['guard_ref_fallback_epochs'] == nav.guard_ref_fallbacks
    np.testing.assert_allclose(nav.get_state().position, batch.p_est[-1],
                               rtol=0, atol=1e-9)
    np.testing.assert_allclose(nav.get_state().covariance,
                               batch.extra['covariance'][-1], rtol=0, atol=1e-9)


@pytest.mark.parametrize('engine', ['batch', 'stream'])
@pytest.mark.parametrize('forced', [False, True])
def test_rejected_boundary_stays_unhealthy(monkeypatch, engine, forced):
    original = GuardReferenceHealth.observe
    boundary = []

    def observe(self, t, **evidence):
        result = original(self, t, **evidence)
        if t == 30.:
            boundary.append((self.start_t, self.healthy, list(self.reasons)))
        return result

    monkeypatch.setattr(GuardReferenceHealth, 'observe', observe)
    torch.set_num_threads(1)
    sc = make_scenario(500, duration=30.2)
    cfg = Config(guard_ref_health=True, pseudo='learned_vel', monitor='dual',
                 cusum_h=1e10, gnss_gate=0.,
                 max_consecutive_rejects=0 if forced else 1000)
    net = load('results/motionnet.pt')
    if engine == 'batch':
        run(sc.truth, sc.imu, sc.gnss, sc.imu_spec, sc.gnss_spec, cfg, model=net)
    else:
        _feed_stream(sc, cfg, net)
    assert len(boundary) == 2
    assert boundary[-1][0] == 30.
    assert boundary[-1][1] is False
    assert ('forced' if forced else 'rejected') in boundary[-1][2]


@pytest.mark.parametrize('engine', ['batch', 'stream'])
def test_completing_recovery_at_boundary_stays_unhealthy(monkeypatch, engine):
    from ccgins.integrity import Cusum

    def step(self, q):
        self._test_epoch = getattr(self, '_test_epoch', 0) + 1
        # GNSS begins at 1 s. Complete recovery exactly on the 30 s boundary.
        self.alarm = self._test_epoch == 29
        self.g = 0.
        return self.alarm

    monkeypatch.setattr(Cusum, 'step', step)
    torch.set_num_threads(1)
    sc = make_scenario(500, duration=30.2)
    cfg = Config(guard_ref_health=True, pseudo='learned_vel', monitor='dual')
    net = load('results/motionnet.pt')
    if engine == 'batch':
        result = run(sc.truth, sc.imu, sc.gnss, sc.imu_spec, sc.gnss_spec,
                     cfg, model=net)
        summary = result.extra['guard_ref_health']
    else:
        nav = _feed_stream(sc, cfg, net)
        summary = nav.guard_ref.summary()
    assert summary['start_t'] == 30.
    assert not summary['healthy']
    assert 'recovery' in summary['reasons']
    assert summary['resets'] == 0
