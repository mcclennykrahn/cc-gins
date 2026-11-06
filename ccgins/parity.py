"""Strict on-grid replay diagnostics; callers explicitly omit start-epoch GNSS.

No scoring/threshold selection: compare the same estimator state after the same
measurements. Off-grid GNSS splitting is deliberately outside this contract.
"""
import numpy as np
from .stream import StreamingNavigator


def replay(sc, cfg, log, odometry=None):
    nav = StreamingNavigator.from_truth(sc.truth, sc.imu_spec, sc.gnss_spec, cfg)
    skipped_start = 0
    for j in np.flatnonzero(sc.gnss.valid):
        k = int(sc.gnss.idx[j])
        if k == 0:
            skipped_start += 1
            continue
        t = sc.gnss.t[j]
        if abs(t-sc.truth.t[k]) > 1e-9:
            raise ValueError('parity requires GNSS on the IMU grid; no epoch snapping')
        sigma = sc.gnss.sigma
        if cfg.use_epoch_sigma and sc.gnss.sigma_ep is not None:
            sigma = np.maximum(sc.gnss.sigma_ep[j], cfg.gnss_sigma_floor)
        nav.push_gnss(t, sc.gnss.pos[j], sigma**2)
    if odometry is not None:
        for j, t in enumerate(odometry.t):
            nav.push_odometry(t, odometry.speed[j], odometry.sigma[j],
                              odometry.C_vb, odometry.lever_b, odometry.valid[j])
    maxima = dict(position_max_abs=0., velocity_max_abs=0., position_cov_max_abs=0.,
                  full_cov_max_abs=0., velocity_block_cov_max_abs=0.,
                  position_block_cov_max_abs=0., vel_pos_cross_cov_max_abs=0.)
    full = log.extra['covariance']
    if full is None:
        raise ValueError('batch must use log_covariance=True')
    j = 0
    for k, t in enumerate(sc.truth.t):
        nav.push_imu(t, sc.imu.accel[k], sc.imu.gyro[k])
        if k == 0 or k % 10:
            continue
        s = nav.get_state()
        assert abs(t-log.t[j]) < 1e-9
        diffs = dict(position_max_abs=s.position-log.p_est[j],
                     velocity_max_abs=s.velocity-log.v_est[j],
                     position_cov_max_abs=s.position_covariance-log.P_pos[j],
                     full_cov_max_abs=s.covariance-full[j],
                     velocity_block_cov_max_abs=s.covariance[3:6,3:6]-full[j,3:6,3:6],
                     position_block_cov_max_abs=s.covariance[6:9,6:9]-full[j,6:9,6:9],
                     vel_pos_cross_cov_max_abs=s.covariance[3:6,6:9]-full[j,3:6,6:9])
        for key, delta in diffs.items():
            maxima[key] = max(maxima[key], float(np.max(np.abs(delta))))
        j += 1
    assert j == len(log.t)
    assert all(x <= 1e-12 for x in maxima.values()), maxima
    assert nav.motion == log.extra['motion']
    return dict(**maxima, motion_equal=True, epochs=j, skipped_start_gnss=skipped_start)
