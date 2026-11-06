import runpy
import numpy as np
import pytest
from ccgins.integrity import Cusum


def test_recovery_interruption_and_default_latch():
    latch = Cusum(h=1)
    recovering = Cusum(h=1, recovery_epochs=3, recovery_q=3)
    for c in (latch, recovering):
        assert c.step(100)
        assert c.step(1)
        assert c.step(4)  # breaks streak even though below CUSUM drift
        assert c.step(1)
        assert c.step(1)
    assert not recovering.step(1)
    assert recovering.g == 0 and recovering.recoveries == 1
    assert latch.step(1)
    assert recovering.step(100)  # new attack re-arms


@pytest.mark.parametrize('q', [np.nan, np.inf, -1])
def test_invalid_statistic_never_recovers(q):
    c = Cusum(h=1, recovery_epochs=2)
    c.step(100)
    c.step(1)
    with pytest.raises(ValueError):
        c.step(q)
    assert c.alarm and c.healthy_streak == 0


@pytest.mark.parametrize('epochs', [-1, 1.5, True])
def test_invalid_recovery_config(epochs):
    with pytest.raises(ValueError):
        Cusum(recovery_epochs=epochs)


def test_loaded_legacy_recovery_batch_stream():
    evaluate = runpy.run_path('results/v08_stage1/evaluate.py')['evaluate']
    row = evaluate(500, duration=40.)
    assert row['alarm_time'] == row['batch_alarm_time'] == 3.
    assert row['recoveries'] >= 1
    assert row['aci_observations'] > 0 and row['batch_aci_updates'] > 0
    assert len(row['batch_recovery_times']) == row['recoveries']
    assert row['position_max_abs'] < 1e-9
    assert row['covariance_max_abs'] < 1e-9
    decisions = row['gnss']
    assert all(g['excluded'] and not g['eligible'] for g in decisions if 3 <= g['t'] <= 5)
    completed = next(g for g in decisions if g['recoveries'])
    assert completed['excluded'] and not completed['eligible']
    assert any(g['accepted'] for g in decisions if g['t'] > completed['t'])
