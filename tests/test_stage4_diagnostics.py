"""Independent checks of cumulative instrumentation and first-alarm accounting."""
import importlib.util
from pathlib import Path

from ccgins.integrity import GuardReferenceHealth


def test_counts_survive_clean_interval_and_include_open_interval():
    h = GuardReferenceHealth()
    h.observe(1., accepted=False, alarm=True)
    h.observe(2., accepted=False, alarm=True)
    assert h.summary()['reason_events']['alarm'] == 2
    assert h.summary()['reason_intervals']['alarm'] == 1
    h.begin(30.)
    h.observe(31., accepted=True)
    assert not any(h.reason_codes().values())
    assert h.summary()['reason_events']['rejected'] == 2
    assert h.summary()['reason_intervals']['rejected'] == 1
    h.begin(60.)
    h.observe(61., accepted=False)
    assert h.summary()['reason_events']['rejected'] == 3
    assert h.summary()['reason_intervals']['rejected'] == 2


def test_freshness_counts_calls_not_independent_gnss_events_and_copies():
    h = GuardReferenceHealth()
    h.check_fresh(2.)
    h.check_fresh(3.)
    snapshot = h.summary()
    assert snapshot['reason_events']['missing'] == 2
    assert snapshot['reason_intervals']['missing'] == 1
    snapshot['reason_events']['missing'] = 999
    snapshot['reason_intervals']['missing'] = 999
    h.begin(30.)
    h.check_fresh(32.)
    assert h.summary()['reason_events']['missing'] == 3
    assert h.summary()['reason_intervals']['missing'] == 2


def test_first_alarm_miss_partition_and_pooled_counts():
    path = Path(__file__).resolve().parents[1] / 'results/v08_stage4/evaluate.py'
    spec = importlib.util.spec_from_file_location('stage4_evaluate', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    rows = []
    for alarm, delay in [(None, None), (299., None), (300., 0.), (310., 10.)]:
        rows.append(dict(guard_ref_health=True, rate=.2, alarm_time=alarm,
                         delay_s=delay, final_horizontal_error_m=1., resets=0,
                         reset_skips=0, fallback_epochs=0, reason_codes={'alarm': 0},
                         reason_events={'alarm': 5}, reason_intervals={'alarm': 2}))
    result = module.summarize(rows, True)
    attack = result['attacks']['0.2']
    assert attack == dict(runs=4, detected=2, misses=2, no_alarm_misses=1,
                          preattack_alarms=1, delays_s=[0., 10.])
    assert attack['misses'] == attack['no_alarm_misses'] + attack['preattack_alarms']
    assert result['navigation']['reason_events'] == {'alarm': 20}
    assert result['navigation']['reason_intervals'] == {'alarm': 8}
    assert result['navigation']['reason_codes'] == {'alarm': 0}
