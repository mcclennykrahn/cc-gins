from dataclasses import replace

import numpy as np

from ccgins.pipeline import Config, run
from ccgins.sim import GnssSpec, ImuSpec, generate_trajectory, simulate_gnss, simulate_imu
from scripts.audit_nhc_outage import scheduled_nhc


def test_scheduled_nhc_exact_replay_and_causal_prefix():
    truth = generate_trajectory(duration=3., seed=900)
    imu_spec, gnss_spec = ImuSpec(), GnssSpec(outages=[(2., .5)])
    imu = simulate_imu(truth, imu_spec, seed=900)
    gnss = simulate_gnss(truth, gnss_spec, seed=900)
    cfg = Config(pseudo='nhc', nhc_sigma=.12, nhc_vertical_sigma=.07)
    native = run(truth, imu, gnss, imu_spec, gnss_spec, cfg)
    for skip in (None, (2., .5)):
        pred = scheduled_nhc(truth, cfg, skip)
        if skip:
            assert 190 in pred[0] and 250 in pred[0]
            assert not np.any((pred[0] >= 200) & (pred[0] < 250))
        replay = run(truth, imu, gnss, imu_spec, gnss_spec,
                     replace(cfg, pseudo='learned_nhc'), pred=pred)
        mask = native.t < 2. if skip else np.ones(len(native.t), bool)
        for field in ('p_est', 'v_est', 'P_pos'):
            np.testing.assert_array_equal(getattr(native, field)[mask],
                                          getattr(replay, field)[mask])
        if skip:
            assert np.linalg.norm(native.p_est[-1] - replay.p_est[-1]) > 0

    entry_off = run(truth, imu, gnss, imu_spec, gnss_spec,
                    replace(cfg, pseudo='learned_nhc'),
                    pred=scheduled_nhc(truth, cfg, (0., 2.)))
    both_off = run(truth, imu, gnss, imu_spec, gnss_spec,
                   replace(cfg, pseudo='learned_nhc'),
                   pred=scheduled_nhc(truth, cfg, (0., 2.5)))
    before_gap = entry_off.t < 2.
    for field in ('p_est', 'v_est', 'P_pos'):
        np.testing.assert_array_equal(getattr(entry_off, field)[before_gap],
                                      getattr(both_off, field)[before_gap])
    assert np.any(entry_off.p_est[~before_gap] != both_off.p_est[~before_gap])
