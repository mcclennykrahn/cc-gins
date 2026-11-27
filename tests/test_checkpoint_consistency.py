import numpy as np
from scipy.stats import chi2

from scripts.audit_checkpoint_consistency import aggregate


def test_checkpoint_aggregate_full_horizontal_covariance_and_run_count():
    covariance = [[2., 1., 0.], [1., 2., 0.], [0., 0., 4.]]
    samples = [dict(error_enu=[2., 2., 2.], covariance=covariance),
               dict(error_enu=[-2., -2., -2.], covariance=covariance)]
    result = aggregate(samples)
    assert result['n'] == 2
    np.testing.assert_allclose(result['mean_error_enu'], 0.)
    np.testing.assert_allclose(result['horizontal_nees_per_dim'], 4/3)
    np.testing.assert_allclose(result['vertical_nees'], 1.)
    np.testing.assert_allclose(result['mean_error_outer'], np.full((3, 3), 4.))
    np.testing.assert_allclose(result['ideal_gaussian_mean_h_nees_per_dim_95_interval'],
                               chi2.ppf([.025, .975], 4)/4)
    np.testing.assert_allclose(result['rmse_h'], np.sqrt(8))
