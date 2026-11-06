import numpy as np
import pytest
from ccgins.filters import ESKF, InEKF, NoiseParams
from ccgins.range_coupling import baseline_range_model, BaselineRangeCoupler

def make(cls):
    return cls(np.eye(3),np.zeros(3),np.array([10.,3.,2.]),np.eye(15)*1e-5,
               NoiseParams(0,0,0,0))

@pytest.mark.parametrize('cls',[ESKF,InEKF])
def test_range_jacobian_covariance(cls):
    f=make(cls); lever=np.array([.5,.2,1.]); z=np.array([11.,4.,3.])
    cov=np.array([[.04,.01,0],[.01,.09,0],[0,0,.16]])
    r,H,R=baseline_range_model(f,z,lever,cov,np.zeros(3))
    assert R[0,0] == pytest.approx(z@cov@z/(z@z))
    for j in range(15):
        b=f.copy(); dx=np.zeros(15); dx[j]=1e-7; b._inject(dx)
        rb,_,_=baseline_range_model(b,z,lever,cov,np.zeros(3))
        assert (r-rb)[0]/1e-7 == pytest.approx(H[0,j],abs=2e-6)

@pytest.mark.parametrize('cls',[ESKF,InEKF])
@pytest.mark.parametrize('drift',[False,True])
def test_growing_lever_error(cls,drift):
    f=make(cls); coupler=BaselineRangeCoupler(np.zeros(3),threshold=20)
    lever=np.array([.5,0,0]); z=f.p+lever
    for epoch in range(100):
        wrong=lever+np.array([.005*epoch if drift else 0,0,0])
        coupler.update(f,z,wrong,np.eye(3)*.01)
    assert coupler.monitor.alarm == drift
    if drift:
        assert abs(coupler.last_residual) > .3

def test_range_singular_geometry():
    f=make(ESKF)
    with pytest.raises(ValueError):
        baseline_range_model(f,np.zeros(3),np.zeros(3),np.eye(3),np.zeros(3))


def test_adis16465_pilot(tmp_path, monkeypatch):
    import json
    from ccgins import i2nav_scenarios as I
    from ccgins.range_experiment import main
    if not (I.DATA/'ADIS16465.npz').exists():
        pytest.skip('i2Nav ADIS16465 data not prepared')
    monkeypatch.chdir(tmp_path)
    main()
    result=json.loads((tmp_path/'results/range_coupling/metrics.json').read_text())
    assert result['0.0']['alarm_time'] is None
    assert 120 < result['0.05']['alarm_time'] < 160
    assert len(result['0.05']['trace']) == 40
