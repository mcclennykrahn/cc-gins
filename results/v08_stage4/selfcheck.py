"""Post-run evidence audit, separate from evaluator aggregation."""
import json
from pathlib import Path

root = Path(__file__).resolve().parent
metrics = json.loads((root / 'metrics.json').read_text())
old = json.loads((root.parent / 'v08_stage3/metrics.json').read_text())
checks = []
historical_differences = {}
for split, seeds in [('validation', range(500, 504)),
                     ('held_out_diagnostic', range(900, 910))]:
    rows = metrics[split]['rows']
    assert len(rows) == len(seeds) * 8
    assert {(r['seed'], r['rate'], r['guard_ref_health']) for r in rows} == {
        (s, rate, flag) for s in seeds for rate in (0., .2, .5, 1.) for flag in (False, True)}
    historical = {(r['seed'], r['rate'], r['guard_ref_health']): r
                  for r in old[split]['rows']}
    for r in rows:
        prev = historical[r['seed'], r['rate'], r['guard_ref_health']]
        for key in ('alarm_time', 'delay_s', 'final_horizontal_error_m', 'resets',
                    'reset_skips', 'fallback_epochs', 'reason_codes'):
            if r[key] != prev[key]:
                label = split + ':' + key
                historical_differences[label] = historical_differences.get(label, 0) + 1
        if r['guard_ref_health']:
            assert set(r['reason_events']) == set(r['reason_intervals'])
            for reason, count in r['reason_intervals'].items():
                assert r['reason_events'][reason] >= count >= r['reason_codes'][reason]
                assert 0 <= count <= r['intervals']
    for summary in metrics[split]['summaries']:
        for attack in summary['attacks'].values():
            assert attack['runs'] == attack['detected'] + attack['misses']
            assert attack['misses'] == attack['preattack_alarms'] + attack['no_alarm_misses']
    checks.append(split + ': complete unique matrix; counters/partitions valid; historical differences recorded')
for case in metrics['parity']:
    for flag in ('False', 'True'):
        assert case[flag]['max_position_delta'] == 0
        assert case[flag]['max_covariance_delta'] == 0
checks.append('batch/stream final state and covariance parity')
print(json.dumps(dict(verdict='PASS', checks=checks, historical_differences=historical_differences, historical_note='Stage3 campaign predates S3-01; differences are not attributed to Stage4 counters', detection='FAIL', confirmed_v08=False), indent=2))
