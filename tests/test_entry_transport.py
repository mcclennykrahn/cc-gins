import numpy as np
import pytest

from scripts.audit_entry_transport import decompose


def test_transport_cancellation_requires_signed_cross_terms():
    t = np.arange(6.)
    ep = np.zeros((6, 3))
    ep[:, 0] = 2 - t
    ev = np.tile([-1., 0., 0.], (6, 1))
    row = decompose(t, ep, ev, 0., 5., 0.)
    h = row['horizontal']
    assert h['remainder_rmse'] == 0
    assert h['sse'] == 10
    assert h['terms']['cross_position_velocity'] == -40
    assert h['reconstructed_sse'] == h['sse']
    assert row['vertical']['sse'] == 0


def test_off_grid_anchor_and_scoring_start_do_not_include_reacquisition():
    t = np.arange(6.)
    ev = np.tile([1., 2., 3.], (6, 1))
    ep = t[:, None]*ev
    ep[5] = 1000  # corrected reacquisition epoch is outside the outage
    row = decompose(t, ep, ev, 1.5, 3.5, 3.)
    assert row['anchor_t'] == 1
    assert row['first_scored_t'] == 3
    assert row['last_scored_t'] == 4
    assert row['n'] == 2
    for name in ('horizontal', 'vertical'):
        assert row[name]['remainder_rmse'] == 0
        assert row[name]['reconstructed_sse'] == row[name]['sse']


def test_nonlinear_remainder_identity():
    rng = np.random.default_rng(9)
    t = np.arange(100)*.1
    ep, ev = rng.normal(size=(2, 100, 3))
    row = decompose(t, ep, ev, 3., 5., 4.)
    for name in ('horizontal', 'vertical'):
        assert row[name]['reconstructed_sse'] == pytest.approx(row[name]['sse'])
        np.testing.assert_allclose(np.array(row[name]['endpoint_transport']) +
                                   row[name]['endpoint_remainder'], row[name]['endpoint_error'])


@pytest.mark.parametrize('t,start,duration', [([1., 2.], 0., 2.),
    ([1., 2.], 1., 0.), ([1., 2.], 3., 1.), ([2., 1.], 1., 1.)])
def test_invalid_or_uncovered_window_refused(t, start, duration):
    with pytest.raises(ValueError):
        decompose(t, np.zeros((2, 3)), np.zeros((2, 3)), start, duration, 0.)
