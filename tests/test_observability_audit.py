from types import SimpleNamespace

import numpy as np

from ccgins.filters import ESKF, mechanize
from ccgins.geometry import so3_exp, so3_log
from ccgins.pipeline import Config
from ccgins.sim import GnssSpec, ImuSpec
from scripts.audit_observability import analyze_window, bias_spectrum, discrete_tangent
from scripts.audit_scheme import budget


def test_bias_spectrum_removes_directions_explained_by_nuisance_state():
    design = np.eye(15)
    design[:, 9] = design[:, 0]
    spectrum = bias_spectrum([design])
    np.testing.assert_allclose(spectrum['bias_singular_values'], [1, 1, 1, 1, 1, 0])
    np.testing.assert_allclose(np.abs(spectrum['weakest_bias_direction']), [1, 0, 0, 0, 0, 0])


def test_level_straight_outage_has_four_unobservable_bias_combinations():
    t = np.arange(3001) * 0.01
    truth = SimpleNamespace(t=t, dt=0.01, v_body=np.tile([12., 0., 0.], (len(t), 1)))
    gnss = SimpleNamespace(idx=np.array([3000]), valid=np.array([True]))
    row = analyze_window(truth, gnss, ImuSpec(), GnssSpec(), Config(pseudo='nhc'),
                         1., 30., straight=True)
    assert row['measurement_rows'] == 2 * 290
    assert row['nuisance_rank'] == 3
    assert row['bias_singular_values'][1] > 1.
    assert row['bias_singular_values'][2] < 1e-8


def test_exact_replay_has_undefined_sse_fraction_and_zero_rmse():
    rows = budget(np.array([30., 31., 32.]), np.zeros((3, 3)), [(31., 1.)])
    assert rows['overall']['horizontal_sse_fraction'] is None
    assert rows['outage_31_32']['rmse_h'] == 0.


def test_discrete_tangent_preserves_global_yaw_during_turn():
    up = np.array([0., 0., 1.])
    R = so3_exp([0.2, -0.3, 1.1])
    v = np.array([12., 3., 2.])
    p = np.array([321., -84., 20.])
    gyro = np.array([0.05, 0.1, 0.25])
    accel = np.array([0.5, 1.8, 9.8])
    dt = 0.01
    earth = {"g": -9.81 * up, "w_ie": None}
    R_next, v_next, p_next = mechanize(R, v, p, gyro, accel, dt,
                                      earth=earth)
    yaw = np.r_[R.T @ up, np.cross(up, v), np.cross(up, p), np.zeros(6)]
    yaw_next = np.r_[R_next.T @ up, np.cross(up, v_next), np.cross(up, p_next), np.zeros(6)]
    tangent = discrete_tangent(R, gyro, accel, dt)
    np.testing.assert_allclose(tangent @ yaw, yaw_next, rtol=0., atol=1e-13)
    _, H = ESKF(R_next, v_next, p_next, np.eye(15), None).body_vel_model((1, 2))
    np.testing.assert_allclose(H @ tangent @ yaw, np.zeros(2), atol=1e-13)

    def propagated_error(delta):
        R1, v1, p1 = mechanize(R @ so3_exp(delta[:3]), v + delta[3:6], p + delta[6:9],
                               gyro - delta[9:12], accel - delta[12:15], dt, earth=earth)
        return np.r_[so3_log(R_next.T @ R1), v1 - v_next, p1 - p_next, delta[9:]]

    eps = 1e-4
    numeric = np.column_stack([(propagated_error(d) - propagated_error(-d)) / (2 * eps)
                               for d in eps * np.eye(15)])
    np.testing.assert_allclose(tangent, numeric, rtol=0., atol=1e-9)
