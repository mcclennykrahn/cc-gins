"""Uncertainty configuration must preserve legacy behavior and reject invalid noise."""
import numpy as np
import pytest

from ccgins.pipeline import Config


def test_legacy_and_anisotropic_nhc():
    np.testing.assert_array_equal(Config().nhc_variances(), np.full(2, .1**2))
    np.testing.assert_array_equal(Config(nhc_sigma=.2, nhc_vertical_sigma=.03).nhc_variances(),
                                  np.array([.2**2, .03**2]))


@pytest.mark.parametrize('value', [0., -.1, np.nan, np.inf])
@pytest.mark.parametrize('field', ['nhc_sigma', 'nhc_vertical_sigma'])
def test_invalid_nhc_noise(field, value):
    with pytest.raises(ValueError, match='NHC sigmas'):
        Config(**{field: value}).nhc_variances()
