"""Render the measured summaries without editing historical evidence."""
import json
from pathlib import Path
root = Path(__file__).resolve().parent
m = json.loads((root/'metrics.json').read_text())
lines = ['# Stage 4 DETECTION diagnostics', '',
         '**DETECTION FAIL; confirmed_v08=false. CUSUM and TCN unpromoted.**', '',
         'Independent source review PASS; full regression 177 passed in 100.24s; new diagnostics included in independent focused regression (21 passed in 18.73s).', '',
         'Frozen Stage3 scenarios and checkpoint reused. Seeds 900–909 were already exposed; this is not fresh held-out evidence. No tuning or navigation change.', '',
         '| Split | Health flag | Clean alarm runs | Rate | Detected | Misses | Preattack alarms | No-alarm misses |',
         '|---|---|---|---|---|---|---|---|']
for split in ('validation','held_out_diagnostic'):
    for s in m[split]['summaries']:
        c=s['clean']
        for rate,a in s['attacks'].items():
            lines.append(f"| {split} | {s['guard_ref_health']} | {c['alarms']}/{c['runs']} | {rate} | {a['detected']}/{a['runs']} | {a['misses']} | {a['preattack_alarms']} | {a['no_alarm_misses']} |")
lines += ['', '## Cumulative bookkeeping', '',
          'Pooled across clean and three attack rates. Events count invalidate calls, including repeated IMU freshness checks and boundary observations; intervals count unique affected intervals including current. Causes overlap. Disabled counters are absent. Counts are not independent GNSS failures, rates or durations.', '',
          '| Split (enabled only) | Reason | Calls | Affected intervals |', '|---|---|---|---|']
for split in ('validation','held_out_diagnostic'):
    s=next(s for s in m[split]['summaries'] if s['guard_ref_health'])
    for reason,count in s['navigation']['reason_events'].items():
        lines.append(f"| {split} | {reason} | {count} | {s['navigation']['reason_intervals'][reason]} |")
lines += ['', '## Limits and checks', '',
          'See selfcheck.json: complete scenario matrices, historical Stage3 differences recorded (campaign predates S3-01), nonnegative cumulative counts, miss partitions and batch/stream final-state/full-covariance parity. Existing reason_codes remain interval-local and can be zero despite earlier failures.', '',
          'Misses include latched preattack alarms; an onset-time alarm counts as zero-delay detection. Clean counts measure runs with any alarm, not transitions. Nominal clean exposure is 2400 s per validation flag and 6000 s per reused diagnostic flag.', '',
          'Post-S3-01 enabled reused-seed fallback count is 146,100 (historical 141,540); pooled final horizontal error is 21.164331 m (historical 20.870568 m). These historical differences do not isolate a Stage4 effect.', '',
          'This closes cumulative cause observability only. Coverage/kappa/reseed-age/DR covariance growth remain unmeasured; no detection, navigation improvement, source authentication, fresh-seed generalization or V0.8 claim. Prior Stage3 artifacts remain intact.', '',
          'Reproduce: PYTHONPATH=. .venv/bin/python results/v08_stage4/evaluate.py all --workers 4; then .venv/bin/python results/v08_stage4/selfcheck.py. Always use all to avoid phase overwrite. Checkpoint SHA256 e2495238df1cf3093155f26cec929d2341f03abb489f04b8acb9dd6a9f515445.', '']
(root/'report.md').write_text('\n'.join(lines))
