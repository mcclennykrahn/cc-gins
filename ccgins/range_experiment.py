"""ADIS16465 real RTK-derived pseudo-range coupling pilot."""
import json
from pathlib import Path
import numpy as np
from . import i2nav_scenarios as I
from .pipeline import Config, run
from .metrics import summarize


def main():
    seq = I.segment('ADIS16465', 0, 160)
    results = {}
    for rate in (0., .05):
        sc = I.make_scenario(seq)
        # Physical antenna displacement in body forward direction; estimator lever stays nominal.
        drift = np.maximum(sc.gnss.t-120., 0)*rate
        sc.gnss.pos[:] += np.einsum('nij,nj->ni', seq.truth.R[seq.gnss_idx],
                                    np.c_[drift, np.zeros((len(drift),2))])
        cfg = Config(filter='inekf', baseline_range_anchor=(-100.,-100.,0.), baseline_range_start_s=120.,
                     **I.REAL_CFG, **I.earth_config(seq))
        log = run(sc.truth,sc.imu,sc.gnss,sc.imu_spec,sc.gnss_spec,cfg)
        metrics = summarize(log, [], t_start=30)
        metrics.pop('runtime_us')
        results[str(rate)] = dict(metrics=metrics, alarm_time=log.extra['range_alarm_time'],
                                  trace=log.extra['range_trace'])
    out = Path('results/range_coupling'); out.mkdir(parents=True,exist_ok=True)
    (out/'metrics.json').write_text(json.dumps(results,indent=2,allow_nan=False)+'\n')
    lines = ['# ADIS16465 RTK-baseline pseudo-range pilot', '',
             '160 s real IMU + RTK, identical seed 0; Earth compensation enabled.',
             'Fixed ENU anchor (-100, -100, 0) m. After 120 s vector-RTK warmup, one scalar update replaces each RTK vector update.',
             'Injection: body-forward antenna displacement starts at 120 s, grows at 0.05 m/s.', '',
             '| Lever drift (m/s) | First CUSUM alarm (s) | Horizontal RMSE (m) |',
             '|---|---:|---:|']
    for rate,r in results.items():
        lines.append(f"| {rate} | {r['alarm_time']} | {r['metrics']['rmse_h']:.3f} |")
    lines += ['', 'This scalar geometry is rank one; tangential drift is not constrained.',
              'An alarm indicates residual inconsistency and is not a diagnosis of lever error.',
              'Baseline false alarms must be considered alongside the injected run.',
              'RTK-derived pseudo-ranges are not raw satellite observations; receiver clock is not estimated.',
              'Reproduce: `OPENBLAS_NUM_THREADS=1 python -m ccgins.range_experiment`', '']
    (out/'report.md').write_text('\n'.join(lines))

if __name__ == '__main__':
    main()
