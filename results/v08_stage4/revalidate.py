"""Audit stored campaign aggregates without importing evaluator or model code."""
import json
import math
from collections import Counter
from pathlib import Path

root = Path(__file__).resolve().parent
metrics = json.loads((root / 'metrics.json').read_text())
checked_rows = 0
for split in ('validation', 'held_out_diagnostic'):
    rows = metrics[split]['rows']
    checked_rows += len(rows)
    for summary in metrics[split]['summaries']:
        selected = [r for r in rows if r['guard_ref_health'] == summary['guard_ref_health']]
        clean = [r for r in selected if r['rate'] == 0]
        assert summary['clean']['runs'] == len(clean)
        assert summary['clean']['alarms'] == sum(r['alarm_time'] is not None for r in clean)
        for rate, counts in summary['attacks'].items():
            attacks = [r for r in selected if r['rate'] == float(rate)]
            detected = preattack = absent = 0
            for row in attacks:
                alarm = row['alarm_time']
                if alarm is None:
                    absent += 1
                    assert row['delay_s'] is None
                elif alarm < 300:
                    preattack += 1
                    assert row['delay_s'] is None
                else:
                    detected += 1
                    assert row['delay_s'] == alarm - 300
            assert counts['runs'] == len(attacks)
            assert counts['detected'] == detected
            assert counts['preattack_alarms'] == preattack
            assert counts['no_alarm_misses'] == absent
            assert counts['misses'] == preattack + absent
        nav = summary['navigation']
        for field in ('reason_events', 'reason_intervals'):
            total = Counter()
            for row in selected:
                total.update(row[field])
            assert dict(total) == nav[field]
        for field in ('resets', 'reset_skips', 'fallback_epochs'):
            assert nav[field] == sum(r[field] for r in selected)
        errors = [r['final_horizontal_error_m'] for r in selected]
        assert math.isclose(nav['mean_final_horizontal_error_m'], math.fsum(errors) / len(errors), rel_tol=1e-12)
        assert nav['max_final_horizontal_error_m'] == max(errors)
print(json.dumps({'verdict': 'PASS', 'rows_audited': checked_rows,
                  'scope': 'Independent arithmetic audit of stored rows; no fresh simulation or held-out evidence',
                  'detection': 'FAIL', 'confirmed_v08': False}, indent=2))
