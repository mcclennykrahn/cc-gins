import numpy as np

from scripts.audit_innovation_serial import capture_innovations, serial_moments
from ccgins.filters import NavFilter, UpdateInfo


def test_serial_moments_excludes_gaps_and_keeps_rejected_fixes():
    rows = [dict(t=t, whitened=u, accepted=t != 31, forced=False)
            for t, u in [(29, [99, 99, 99]), (30, [1, 2, 0]),
                         (31, [3, 4, 0]), (90, [9, 9, 9]), (91, [5, 6, 0])]]
    result = serial_moments(rows, 30, 91)
    assert result['n'] == 3 and result['pairs'] == 1
    assert result['rejected'] == 1
    np.testing.assert_array_equal(result['lag_one_cross_moment'],
                                 np.outer([1, 2, 0], [3, 4, 0]))
    assert serial_moments(rows, 40, 50) == dict(n=0, pairs=0)


def test_capture_uses_preupdate_full_covariance_and_restores_method(monkeypatch):
    class Fake:
        P = np.array([[2., 1., 0.], [1., 2., 0.], [0., 0., 3.]])
        def gnss_model(self, lever):
            return np.ones(3), np.eye(3)

    def original(filt, z, lever, Rm, **kwargs):
        filt.P = np.eye(3)*100
        return UpdateInfo(2., False, 3)

    monkeypatch.setattr(NavFilter, 'update_gnss', original)
    with capture_innovations([30.]) as rows:
        NavFilter.update_gnss(Fake(), np.array([2., 3., 4.]), None, np.eye(3), force=True)
    assert NavFilter.update_gnss is original
    S = np.array(rows[0]['covariance'])
    np.testing.assert_allclose(np.linalg.cholesky(S) @ rows[0]['whitened'], [1, 2, 3])
    assert S[0, 1] == 1 and S[0, 0] == 3
    assert not rows[0]['accepted'] and rows[0]['forced']
