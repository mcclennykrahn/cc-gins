"""Check paired default traces and independently measure noiseless Earth closure."""
import json
from pathlib import Path
import subprocess
import sys
import numpy as np

root = Path('/workspace/ccgins-v08/results/cc_scheme_audit/earth_model')
checks = {}
for seed in (900, 118001, 118002):
    a = np.loadtxt(root/'default_before'/f'errors_{seed}.csv', delimiter=',', skiprows=1)
    b = np.loadtxt(root/'default_after'/f'errors_{seed}.csv', delimiter=',', skiprows=1)
    checks[str(seed)] = dict(default_trace_identical=bool(np.array_equal(a, b)),
                            max_abs_delta=float(np.max(np.abs(a-b))))
    assert np.array_equal(a, b)
code = '''
import json
import numpy as np
from ccgins.cli import load_config, _build_inputs, _filter_config
from ccgins.filters import mechanize
raw=load_config('/workspace/ccgins-v08/results/cc_scheme_audit/earth_model/config.yaml')
truth,*_= _build_inputs(raw)
cfg=_filter_config(raw)
earth={'w_ie':np.asarray(cfg.earth_rate_enu), 'g':np.array([0.,0.,-cfg.local_gravity])}
R,v,p=truth.R[0].copy(),truth.v[0].copy(),truth.p[0].copy()
for k in range(len(truth.t)-1):
 R,v,p=mechanize(R,v,p,truth.gyro[k],truth.accel[k],truth.dt,earth)
print(json.dumps({'rotation_frobenius':float(np.linalg.norm(R-truth.R[-1])),
 'velocity_mps':float(np.linalg.norm(v-truth.v[-1])),
 'position_m':float(np.linalg.norm(p-truth.p[-1]))}))
'''
for name, directory in [('parent','/tmp/ccgins-earth-parent'), ('candidate','/workspace/cc-gins')]:
    checks[name] = json.loads(subprocess.check_output([sys.executable,'-c',code], cwd=directory))
assert checks['candidate']['position_m'] < 1e-6
(root/'verification.json').write_text(json.dumps(checks, indent=2)+'\n')
print(json.dumps(checks, indent=2))
