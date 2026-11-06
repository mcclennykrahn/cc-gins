"""Campaign accounting must not count early alarms as ramp detections."""
import runpy


def test_detection_accounting_separates_clean_early_and_missed():
    protocol = runpy.run_path('results/v08_stage2/evaluate.py')
    rows = [dict(rate=0., alarm_time=None, gnss_epochs=600)]
    for rate, alarm in [(0.2, 299.), (0.5, None), (1., 310.)]:
        rows.append(dict(rate=rate, alarm_time=alarm,
                         delay_s=alarm-300 if alarm is not None and alarm>=300 else None))
    summary = protocol['summarize'](rows)
    assert summary['clean_exposure_s'] == 600
    assert summary['clean_gnss_epochs'] == 600
    assert summary['attacks']['0.2']['preattack_alarms'] == 1
    assert summary['attacks']['0.2']['detected'] == 0
    assert summary['attacks']['0.5']['misses'] == 1
    assert summary['attacks']['1.0']['delays_s'] == [10.]
    assert not protocol['eligible'](summary)


def test_clean_alarm_blocks_even_complete_attack_detection():
    protocol = runpy.run_path('results/v08_stage2/evaluate.py')
    rows = [dict(rate=0., alarm_time=10., gnss_epochs=600)]
    rows += [dict(rate=r, alarm_time=301., delay_s=1.) for r in protocol['RATES'][1:]]
    assert not protocol['eligible'](protocol['summarize'](rows))
