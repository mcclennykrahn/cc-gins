"""Validate fresh parent/candidate runs; consolidate prior unpromoted ensemble evidence."""
import argparse
import json
from pathlib import Path
import numpy as np
from scripts.audit_segment_ensemble import summarize

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('evidence', type=Path)
root = parser.parse_args().evidence
parents = json.loads((root/'current_before/budget.json').read_text())
candidates = json.loads((root/'current_after/budget.json').read_text())
assert parents['config'] == candidates['config']
assert [r['seed'] for r in parents['rows']] == [900, 117001, 117002]
assert [r['seed'] for r in candidates['rows']] == [900, 117001, 117002]
paired = []
for a, b in zip(parents['rows'], candidates['rows']):
    name = f"errors_{a['seed']}.csv"
    x = np.loadtxt(root/'current_before'/name, delimiter=',', skiprows=1)
    y = np.loadtxt(root/'current_after'/name, delimiter=',', skiprows=1)
    np.testing.assert_array_equal(x[:,0], y[:,0])
    np.testing.assert_allclose(x[:,1:], y[:,1:], atol=1e-9, rtol=0)
    assert a['gnss_only'] == b['gnss_only']
    for parent_ins, candidate_ins in zip(a['pure_ins_truth_initialized'],
                                         b['pure_ins_truth_initialized'], strict=True):
        assert parent_ins.keys() == candidate_ins.keys()
        np.testing.assert_allclose(list(parent_ins.values()), list(candidate_ins.values()),
                                   atol=1e-9, rtol=0)
    paired.append(dict(seed=a['seed'], before=a['segments'], after=b['segments'],
        max_abs_enu_delta_m=float(np.max(np.abs(x[:,1:] - y[:,1:])))))
rows = [r for p in sorted((root/'ensemble_final').glob('part*/ensemble.json'))
        for r in json.loads(p.read_text())['rows']]
assert [r['seed'] for r in rows] == list(range(117101,117133))
for r in rows:
    assert r['init_seed'] == r['seed'] + 1000000
result = dict(candidate_id='scheme_so3_half_turn_completion', paired=paired,
    inherited_ensemble_summary=summarize(rows),
    inherited_ensemble_scope='Prior pending candidate evidence, descriptive only; not rerun on final source. Fresh three-seed comparison above validates final source.',
    audit_complete=False, confirmed_v08=False, detection='FAIL')
output = json.dumps(result, indent=2, allow_nan=False)+'\n'
(root/'current_validation.json').write_text(output)
print('PASS: fresh paired traces, segment budgets and inherited 32-seed summary')
