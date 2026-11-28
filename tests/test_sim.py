import subprocess
import sys

import pytest

from ccgins.sim import generate_trajectory


def test_coarse_grid_segments_include_initial_stop_and_finish():
    # Bound the test even if a segment loop stops advancing again.
    subprocess.run([sys.executable, "-c", """
import numpy as np
from ccgins.sim import generate_trajectory

truth = generate_trajectory(duration=60., dt=30., seed=900)
np.testing.assert_array_equal(truth.t, [0., 30., 60.])
np.testing.assert_array_equal(truth.v[0], [0., 0., 0.])
assert np.isfinite(truth.gyro).all()
assert np.isfinite(truth.accel).all()
"""], check=True, timeout=10)


def test_trajectory_requires_one_full_grid_step():
    with pytest.raises(ValueError, match="at least one IMU interval"):
        generate_trajectory(duration=.01, dt=1.)


@pytest.mark.parametrize("kwargs", [
    {"duration": 0.},
    {"dt": 0.},
    {"dt": float("nan")},
    {"vmax": -1.},
    {"vmax": float("inf")},
])
def test_trajectory_rejects_invalid_grid_inputs(kwargs):
    with pytest.raises(ValueError, match="trajectory requires"):
        generate_trajectory(**kwargs)
