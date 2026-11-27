from types import SimpleNamespace

import numpy as np
import pytest

from scripts.audit_causal_course import causal_course, measured_rotation, wrap_angle


def test_course_is_causal_enu_and_ignores_invalid_outage_fixes():
    t = np.arange(12.)
    p = np.column_stack([3*t, 4*t, -t])
    valid = np.ones(len(t), dtype=bool)
    original = causal_course(t, p, valid, 5.)
    assert original['status'] == 'candidate'
    np.testing.assert_allclose(original['velocity_enu'], [3, 4, -1])
    assert original['course_rad'] == pytest.approx(np.arctan2(4, 3))
    p[6:] = np.nan
    assert causal_course(t, p, valid, 5.) == original
    valid[3:] = False
    p[3:] = 1e10
    assert causal_course(t, p, valid, 5.)['status'] == 'stale'


def test_stationary_rejects_heading_and_reacquisition_needs_history():
    t = np.arange(11.)
    p = np.zeros((len(t), 3))
    assert causal_course(t, p, np.ones(11, bool), 5.)['status'] == 'low_speed'
    valid = t >= 10
    row = causal_course(t, p, valid, 10.)
    assert row['status'] == 'insufficient_fixes'
    assert row['course_rad'] is None
    assert causal_course(t, p, ~np.ones(11, bool), 10.)['n_fixes'] == 0


def test_turning_course_is_not_endpoint_body_heading():
    t = np.arange(6.)
    theta = t*.2
    p = np.column_stack([20*np.sin(theta), 20*(1-np.cos(theta)), np.zeros(6)])
    row = causal_course(t, p, np.ones(6, bool), 5.)
    assert row['status'] == 'candidate'
    # A turn's chord/course estimates mid-window direction, not endpoint yaw.
    assert row['course_rad'] == pytest.approx(.5)
    assert abs(wrap_angle(row['course_rad']-theta[-1])) == pytest.approx(.5)


def test_course_rejects_bad_protocol_and_nonfinite_valid_fix():
    t = np.arange(6.)
    p = np.column_stack([4*t, t, t])
    valid = np.ones(6, bool)
    with pytest.raises(ValueError):
        causal_course(t[::-1], p, valid, 5.)
    with pytest.raises(ValueError):
        causal_course(t, p, valid, 5., min_speed=0)
    p[3, 0] = np.nan
    assert causal_course(t, p, valid, 5.)['status'] == 'invalid_position'


def test_rotation_gate_counts_complete_steps_and_reversals():
    gyro = np.deg2rad([[90., 0, 0], [0, 0, 4.], [0, 0, -4.], [90., 0, 0]])
    imu = SimpleNamespace(t=np.arange(4.) * .5, dt=.5, gyro=gyro)
    assert measured_rotation(imu, .5, 1.49) == pytest.approx(2.)
    assert measured_rotation(imu, .5, 1.5) == pytest.approx(4.)
