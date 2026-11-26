"""Matched report configurations: all state covariance blocks, not just position."""
from dataclasses import replace

import numpy as np
import pytest

from ccgins.pipeline import Config, run, mount_matrix
from ccgins.sim import ImuSpec, GnssSpec, generate_trajectory, simulate_imu, simulate_gnss
from ccgins.stream import StreamingNavigator
from ccgins.zupt import OdometryData


@pytest.mark.parametrize('vertical_sigma', [None, .03])
@pytest.mark.parametrize('kind', ['eskf', 'inekf'])
@pytest.mark.parametrize('mode', ['nhc_baseline', 'nhc_zupt', 'guarded', 'wheel', 'wheel_stop'])
def test_report_configurations_all_covariance(kind, mode, vertical_sigma):
    truth = generate_trajectory(8., seed=12)
    spec = ImuSpec()
    gs = GnssSpec(rate_hz=5.)
    imu = simulate_imu(truth, spec, seed=12)
    gnss = simulate_gnss(truth, gs, seed=12)
    # Non-binary interval at a large epoch exposes timestamp subtraction drift.
    truth.t += 12345.123
    gnss.t = truth.t[gnss.idx]
    gnss.valid &= (gnss.idx > 0) & ~((gnss.t > truth.t[0]+3) & (gnss.t < truth.t[0]+6))
    gnss.sigma_ep = np.tile([.01, .08, .03], (len(gnss.t), 1))
    cfg = Config(filter=kind, pseudo='nhc' if mode.startswith('nhc') else 'none',
                 zupt=mode != 'nhc_baseline', zupt_odometry_stop=mode == 'wheel_stop',
                 mount_py=(.13, -.21), nhc_sigma=.47, nhc_vertical_sigma=vertical_sigma, use_epoch_sigma=True,
                 gnss_sigma_floor=.05, earth_rate_enu=(0., 5e-5, 5e-5), local_gravity=9.8,
                 p0_bias=(.001, .02), zupt_accel_var=.1, zupt_gyro_var=.001)
    odo = None
    if mode in ('wheel', 'wheel_stop'):
        ii = np.arange(10, len(truth.t), 10)
        C = mount_matrix(*cfg.mount_py)
        vb = np.einsum('nji,nj->ni', truth.R[ii], truth.v[ii])
        odo = OdometryData(truth.t[ii], (vb @ C.T)[:, 0], .03, C, [.4, -.2, .1], ii % 30 != 0)
    log = run(truth, imu, gnss, spec, gs, cfg, odometry=odo, log_covariance=True)
    nav = StreamingNavigator.from_truth(truth, spec, gs, cfg)
    for j in np.flatnonzero(gnss.valid)[::-1]:
        nav.push_gnss(gnss.t[j], gnss.pos[j], np.maximum(gnss.sigma_ep[j], cfg.gnss_sigma_floor)**2)
    if odo is not None:
        for j, t in enumerate(odo.t):
            nav.push_odometry(t, odo.speed[j], odo.sigma[j], odo.C_vb, odo.lever_b, odo.valid[j])
    states = []
    for k, t in enumerate(truth.t):
        nav.push_imu(t, imu.accel[k], imu.gyro[k])
        if k > 0 and k % 10 == 0:
            states.append(nav.get_state())
    for actual, expected in (([s.position for s in states], log.p_est),
                             ([s.velocity for s in states], log.v_est),
                             ([s.position_covariance for s in states], log.P_pos),
                             ([s.covariance for s in states], log.extra['covariance'])):
        np.testing.assert_allclose(actual, expected, atol=1e-12, rtol=0)
    assert nav.motion == log.extra['motion']


@pytest.mark.parametrize('option', [dict(pseudo='hybrid'), dict(aci=True), dict(monitor='single'),
                                  dict(adaptive_r=True), dict(baseline_range_anchor=(0, 0, 0))])
def test_unsupported_batch_options_are_explicit(option):
    truth = generate_trajectory(1.)
    with pytest.raises(ValueError, match='streaming supports'):
        StreamingNavigator.from_truth(truth, ImuSpec(), GnssSpec(), replace(Config(), **option))


def test_start_gnss_and_off_grid_split_remain_explicit():
    truth = generate_trajectory(1.)
    spec, gs = ImuSpec(), GnssSpec()
    cfg = Config()
    nav = StreamingNavigator.from_truth(truth, spec, gs, cfg)
    ref = StreamingNavigator.from_truth(truth, spec, gs, cfg)
    acc, gyro = [0, 0, 9.81], [0, 0, 0]
    z = truth.p[0] + [1., 0, 0]
    nav.push_gnss(0., z, .01)
    nav.push_imu(0., acc, gyro)
    ref.push_imu(0., acc, gyro)
    assert not np.array_equal(nav.get_state().position, ref.get_state().position)
    ref.filter.update_gnss(z, ref.lever_arm, np.eye(3)*.01, gate=ref.gate)
    nav.push_gnss(.003, z, .01)
    nav.push_imu(.01, acc, gyro)
    ref.filter.propagate(gyro, acc, .003)
    ref.filter.update_gnss(z, ref.lever_arm, np.eye(3)*.01, gate=ref.gate)
    ref.filter.propagate(gyro, acc, .007)
    np.testing.assert_array_equal(nav.get_state().covariance, ref.filter.P)
    np.testing.assert_array_equal(nav.get_state().velocity, ref.filter.v)
