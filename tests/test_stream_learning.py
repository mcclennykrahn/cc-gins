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
    np.testing.assert_allclose(nav.get_state().position, batch.p_est[-1], atol=1e-10, rtol=0)
    np.testing.assert_allclose(nav.get_state().covariance, batch.extra['covariance'][-1], atol=1e-10, rtol=0)
    np.testing.assert_allclose([r['q'] for r in nav.gnss_decisions], batch.stat_q, atol=1e-10, rtol=0)
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


def test_rejects_late_and_unsupported():
    sc = make_scenario(714, duration=2.)
    cfg = Config(pseudo='learned_vel')
    nav = drive(sc, cfg, model())
    with pytest.raises(ValueError, match='before its epoch'):
        nav.push_gnss(1., [0, 0, 0], 1.)
    nav.push_gnss(2.003, [0, 0, 0], 1.)
    for option in [dict(zupt=True), dict(adaptive_r=True)]:
        with pytest.raises(ValueError):
            LearnedStreamingNavigator.from_truth(sc.truth, sc.imu_spec, sc.gnss_spec,
                replace(cfg, **option), model=model())


def test_pending_delay_budget_and_duplicate_fix():
    sc = make_scenario(716, duration=1.)
    cfg = Config(pseudo='learned_vel')
    nav = LearnedStreamingNavigator.from_truth(sc.truth, sc.imu_spec, sc.gnss_spec,
                                               cfg, model=model(), delay=.05)
    nav.push_imu(0., sc.imu.accel[0], sc.imu.gyro[0])
    with pytest.raises(RuntimeError, match='no IMU'):
        nav.get_state()
    nav.push_gnss(0., [0, 0, 0], 1.)
    with pytest.raises(ValueError, match='duplicate'):
        nav.push_gnss(0., [0, 0, 0], 1.)
    for k in range(1, 7):
        nav.push_imu(sc.truth.t[k], sc.imu.accel[k], sc.imu.gyro[k])
    assert nav.get_state().t == .01
    with pytest.raises(ValueError, match='before its epoch'):
        nav.push_gnss(.01, [0, 0, 0], 1.)
    nav.flush()
    assert nav.get_state().t == .06


def test_prequeued_offgrid_is_split():
    sc = make_scenario(717, duration=1.)
    nav = LearnedStreamingNavigator.from_truth(sc.truth, sc.imu_spec, sc.gnss_spec,
            Config(pseudo='learned_vel'), model=model())
    nav.push_gnss(.003, [0, 0, 0], 1.)
    nav.push_imu(0., sc.imu.accel[0], sc.imu.gyro[0])
    nav.push_imu(.01, sc.imu.accel[1], sc.imu.gyro[1])
    assert nav.gnss_decisions[0]['t'] == .003
    assert nav.t == .01


@pytest.mark.parametrize('pseudo', ['hybrid', 'learned_vel'])
def test_wheel_layer_batch_parity(pseudo):
    from ccgins.zupt import OdometryData
    sc = make_scenario(718, duration=5.)
    sc.gnss.valid[sc.gnss.idx == 0] = False
    cfg = Config(pseudo=pseudo, aci=True, monitor='dual', cusum_h=1e12,
                 corr_scale=(40.,40.,40.), hybrid_max_std=.01)
    ii = np.arange(10, len(sc.truth.t), 10)
    vb = np.einsum('nji,nj->ni', sc.truth.R[ii], sc.truth.v[ii])
    odo = OdometryData(sc.truth.t[ii], vb[:,0], .1, np.eye(3), np.zeros(3))
    net = model()
    batch = run(sc.truth, sc.imu, sc.gnss, sc.imu_spec, sc.gnss_spec, cfg,
                model=net, odometry=odo, log_covariance=True)
    nav = LearnedStreamingNavigator.from_truth(sc.truth, sc.imu_spec, sc.gnss_spec,
                                               cfg, model=net, delay=.05)
    for j,t in enumerate(odo.t):
        nav.push_odometry(t,odo.speed[j],odo.sigma[j],odo.C_vb,odo.lever_b)
    drive(sc,cfg,net,nav=nav)
    np.testing.assert_allclose(nav.filter.p,batch.p_est[-1],atol=1e-10,rtol=0)
    np.testing.assert_allclose(nav.filter.P,batch.extra['covariance'][-1],atol=1e-10,rtol=0)
    assert nav.motion == batch.extra['motion']


def test_offgrid_out_of_order_delayed_and_guard_split():
    sc = make_scenario(719,duration=3.)
    cfg = Config(pseudo='learned_nhc', monitor='dual', cusum_h=1e12)
    net = model(True)
    a = LearnedStreamingNavigator.from_truth(sc.truth,sc.imu_spec,sc.gnss_spec,cfg,model=net,delay=.05)
    b = LearnedStreamingNavigator.from_truth(sc.truth,sc.imu_spec,sc.gnss_spec,cfg,model=net,delay=.05)
    fixes=[(.503,sc.truth.p[50]),(.507,sc.truth.p[50]),(1.405,sc.truth.p[140])]
    for t,p in fixes[::-1]: a.push_gnss(t,p,4.)
    for k,t in enumerate(sc.truth.t):
        for epoch,p in fixes[::-1]:
            if abs(t-(np.ceil(epoch*100)/100+.02))<1e-9: b.push_gnss(epoch,p,4.)
        a.push_imu(t,sc.imu.accel[k],sc.imu.gyro[k])
        b.push_imu(t,sc.imu.accel[k],sc.imu.gyro[k])
    a.flush(); b.flush()
    np.testing.assert_array_equal(a.filter.P,b.filter.P)
    np.testing.assert_array_equal(a.guard.P,b.guard.P)
    assert a.decisions == b.decisions
    assert [g['t'] for g in a.gnss_decisions] == [f[0] for f in fixes]


def test_buffer_explicit_error_and_replay(tmp_path):
    from ccgins.stream_replay import save_artifact,replay
    sc=make_scenario(720,duration=2.)
    cfg=Config(pseudo='learned_vel')
    net=model()
    with pytest.raises(ValueError,match='budget'):
        LearnedStreamingNavigator.from_truth(sc.truth,sc.imu_spec,sc.gnss_spec,cfg,model=net,delay=1.,max_buffer=2)
    nav=LearnedStreamingNavigator.from_truth(sc.truth,sc.imu_spec,sc.gnss_spec,cfg,model=net,max_buffer=2)
    nav.push_gnss(1.,[0,0,0],1.);nav.push_gnss(2.,[0,0,0],1.)
    with pytest.raises(BufferError): nav.push_gnss(3.,[0,0,0],1.)
    with pytest.raises(ValueError): nav.push_gnss(4.,[np.nan,0,0],1.)
    path=tmp_path/'full.npz'
    save_artifact(path,sc,cfg,lag=.03,delay=.05)
    events=[]
    replayed,timing=replay(path,net,events.append)
    expected=drive(sc,cfg,net)
    np.testing.assert_array_equal(replayed.filter.P,expected.filter.P)
    assert all(e['arrival_time']>=e.get('epoch',e.get('t')) for e in events)
    assert all(e['status'] in ('processed','accepted','rejected','excluded') for e in events)
    assert timing['hz']>0


def test_realistic_ramp_with_100_sample_latency():
    import runpy
    probe=runpy.run_path('results/proposal_alignment_stream_fifth/spoof_probe.py')['probe']
    clean,bad=probe(False),probe(True)
    assert clean['alarm_time'] is None
    assert bad['alarm_time']==8.  # 3 s after ramp onset at 5 s
    assert all(g['accepted'] for g in clean['gnss'])
    assert all(g['accepted'] for g in bad['gnss'] if g['t']<=5 or g['t']>13)
    assert all(g['arrival_time']>=g['t']+1. for g in bad['gnss'])
    assert all(g['g']<=7. for g in bad['gnss'] if g['t']<8.)
    excluded=probe(True,exclude=True)
    assert excluded['alarm_time']==8.
    assert all(g['excluded'] and not g['accepted'] for g in excluded['gnss'] if g['t']>=8.)


@pytest.mark.parametrize('bias_guard', [False, True])
def test_hybrid_warmup_calibration_and_bias_fallback(bias_guard):
    sc=make_scenario(721,duration=34.)
    sc.gnss.valid[sc.gnss.idx==0]=False
    cfg=Config(pseudo='hybrid',aci=True,monitor='dual',cusum_h=1e12,
               corr_scale=(40.,40.,40.),mount_py=(.05,.1),
               hybrid_bias_z=.1 if bias_guard else 0.,hybrid_bias_min=3,
               hybrid_bias_latch=bias_guard)
    net=model()
    batch=run(sc.truth,sc.imu,sc.gnss,sc.imu_spec,sc.gnss_spec,cfg,model=net,log_covariance=True)
    nav=drive(sc,cfg,net)
    np.testing.assert_allclose(nav.filter.p,batch.p_est[-1],atol=1e-10,rtol=0)
    np.testing.assert_allclose(nav.filter.P,batch.extra['covariance'][-1],atol=1e-10,rtol=0)
    assert nav.cal._n>0
    if bias_guard: assert nav.bias_bad and nav.bias_latched


def test_offgrid_split_matches_explicit_main_and_guard_propagation():
    sc=make_scenario(722,duration=1.)
    cfg=Config(pseudo='learned_vel',monitor='dual',cusum_h=1e12)
    nav=LearnedStreamingNavigator.from_truth(sc.truth,sc.imu_spec,sc.gnss_spec,cfg,model=model())
    main,guard=nav.filter.copy(),nav.guard.copy()
    gyro=np.array([.1,.2,.3]);acc=np.array([.4,.5,9.7]);z=np.array([0.,0.,0.])
    nav.push_gnss(.003,z,4.)
    nav.push_imu(0.,acc,gyro);nav.push_imu(.01,acc,gyro)
    main.propagate(gyro,acc,.003);guard.propagate(gyro,acc,.003)
    main.update_gnss(z,nav.lever_arm,np.eye(3)*4.,gate=nav.gate)
    main.propagate(gyro,acc,.007);guard.propagate(gyro,acc,.007)
    np.testing.assert_array_equal(nav.filter.P,main.P)
    np.testing.assert_array_equal(nav.guard.P,guard.P)
    np.testing.assert_array_equal(nav.guard.p,guard.p)


def test_near_grid_metadata_and_history_are_bounded():
    sc=make_scenario(723,duration=.1)
    nav=LearnedStreamingNavigator.from_truth(sc.truth,sc.imu_spec,sc.gnss_spec,
        Config(pseudo='learned_vel'),model=model(),max_buffer=2)
    nav.push_imu(0.,[0,0,9.81],[0,0,0])
    for k in range(1,11):
        nav.push_gnss(k*.01+1e-10,[0,0,0],4.,arrival_time=k*.01+.001)
        nav.push_imu(k*.01,[0,0,9.81],[0,0,0])
    assert len(nav.gnss_decisions)==2
    assert not nav._gnss_arrivals
    assert nav.gnss_decisions[-1]['arrival_time']==.101
