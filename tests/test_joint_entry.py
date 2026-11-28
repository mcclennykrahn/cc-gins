"""Do not confuse squared-error cancellation with dynamic vector interaction."""
import numpy as np
import pytest

from scripts.audit_joint_entry import interaction


def cells():
    native = np.zeros((4, 3))
    a = native + [1., 0., 0.]
    b = native + [2., 0., 0.]
    return dict(native=native, attitude=a, biases=b, attitude_and_biases=a+b)


def test_additive_vectors_can_have_nonzero_squared_error_contrast():
    r = interaction(np.arange(4.), cells(), 1., 2., 0.)
    assert r['n'] == 2
    assert r['horizontal']['vector_interaction_rms'] == 0.
    assert r['horizontal']['mse_contrast'] == 4.
    assert r['horizontal']['sse_contrast'] == 8.


def test_vector_interaction_and_scoring_intersection():
    e = cells()
    e['attitude_and_biases'][2] += [0., 4., 3.]
    e['attitude_and_biases'][3] += 1000.  # outage endpoint excluded
    r = interaction(np.arange(4.), e, 1., 2., 2.)
    assert r['n'] == 1
    assert r['horizontal']['vector_interaction_rms'] == 4.
    assert r['vertical']['vector_interaction_rms'] == 3.


def test_integer_errors_do_not_overflow_factorial_arithmetic():
    errors = {arm: (200 * values).astype(np.int16) for arm, values in cells().items()}
    result = interaction(np.arange(4.), errors, 1., 2., 0.)
    assert result['horizontal']['sse']['attitude'] == 80_000.
    assert result['horizontal']['mse_contrast'] == 160_000.
    assert result['horizontal']['vector_interaction_rms'] == 0.


@pytest.mark.parametrize('bad', ['missing', 'shape', 'nan', 'time', 'empty', 'duration'])
def test_invalid_factorials_refused(bad):
    t, e, start, duration = np.arange(4.), cells(), 1., 2.
    if bad == 'missing':
        del e['biases']
    elif bad == 'shape':
        e['biases'] = np.zeros((3, 3))
    elif bad == 'nan':
        e['native'][0, 0] = np.nan
    elif bad == 'time':
        t[1] = t[0]
    elif bad == 'empty':
        start = 5.
    else:
        duration = 0.
    with pytest.raises(ValueError):
        interaction(t, e, start, duration, 0.)
