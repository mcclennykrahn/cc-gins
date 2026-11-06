import json
from types import SimpleNamespace
import numpy as np
import pytest
from ccgins.metrics import summarize
from ccgins.comparison import write_report

@pytest.mark.parametrize('error,coverage,anees', [(0,1,0), (1,1,1), (4,0,16)])
def test_consistency_definitions(error, coverage, anees):
    log = SimpleNamespace(t=np.arange(40.), p_est=np.full((40,3), error),
                          p_true=np.zeros((40,3)), P_pos=np.tile(np.eye(3),(40,1,1)),
                          runtime_per_step_us=0)
    m = summarize(log, [(31,5)], t_start=30)
    assert m['coverage95_h'] == coverage
    assert m['anees_pos'] == anees
    assert m['anees_pos_outage'] == anees
    r = dict(imu='i300', duration_s=40, outages=[(31,5)],
             aggregate={'eskf':m, 'inekf':m})
    # Covered separately by each analytically known covariance/error pair.
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        write_report(r,d)
        assert json.load(open(d+'/metrics.json'))['aggregate']['inekf']['anees_pos'] == anees
        assert 'Outage ANEES' in open(d+'/report.md').read()
