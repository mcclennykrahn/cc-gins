"""Paired, reproducible real i300 position-consistency comparison."""
import argparse
import json
from pathlib import Path

from . import filters
from . import i2nav_scenarios as I
from .metrics import summarize
from .pipeline import Config, run


def compare(seq, outages, seeds=(0, 1)):
    """Identical IMU, GNSS, outage mask and initialization seeds for both filters."""
    old_w, old_g = filters.EARTH['w_ie'], filters.EARTH['g'].copy()
    rows = []
    try:
        I.enable_earth(seq)
        for seed in seeds:
            sc = I.make_scenario(seq, seed, outages)
            row = {'seed': seed, 'filters': {}}
            for kind in ('eskf', 'inekf'):
                cfg = Config(filter=kind, pseudo='none', init_seed=seed, **I.REAL_CFG, **I.earth_config(seq))
                log = run(sc.truth, sc.imu, sc.gnss, sc.imu_spec, sc.gnss_spec, cfg)
                metrics = summarize(log, outages, t_start=30.0)
                metrics.pop('runtime_us')  # machine timing is not a reproducible metric
                row['filters'][kind] = metrics
            rows.append(row)
    finally:
        filters.EARTH.update(w_ie=old_w, g=old_g)
    return {'imu': seq.name, 'duration_s': float(seq.truth.t[-1]),
            'outages': outages, 'runs': rows,
            'definitions': {'coverage95_h': 'fraction with horizontal NEES <= 5.991464547',
                            'anees_pos': 'mean 3D position NEES / 3 (ideal 1)'},
            'aggregate': {kind: {key: sum(r['filters'][kind][key] for r in rows)/len(rows)
                                 for key in rows[0]['filters'][kind]}
                          for kind in ('eskf', 'inekf')}}


def write_report(result, out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    (out/'metrics.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    lines = ['# i300: InEKF vs ESKF', '',
             f"Real {result['imu']} IMU + RTK + reference, {result['duration_s']:.2f} s.",
             f"Injected outages (start, duration), seconds: {result['outages']}.",
             'Paired initial seeds; no learned or motion pseudo-observations. Earth compensation enabled.',
             'Receiver epoch sigma floor: 0.05 m. Initialization excluded for first 30 s.', '',
             '| Filter | Horizontal RMSE (m) | 95% horizontal coverage | Outage coverage | Position ANEES | Outage ANEES |',
             '|---|---:|---:|---:|---:|---:|']
    for kind, m in result['aggregate'].items():
        keys = ('rmse_h', 'coverage95_h', 'coverage95_h_outage', 'anees_pos', 'anees_pos_outage')
        lines.append('| '+kind+' | '+' | '.join(f'{m[k]:.4f}' for k in keys)+' |')
    lines += ['', 'Coverage uses the 2D position covariance ellipse and chi-square threshold 5.9915.',
              'ANEES is mean 3D position NEES divided by 3 (ideal 1).',
              'These are time-correlated samples from one drive, not independent Monte Carlo trials;',
              'coverage is descriptive, not a confidence claim or proof of consistency.',
              'Both filters use the same gating and forced reacquisition policy. No tuning to truth.', '',
              'Reproduce: `OPENBLAS_NUM_THREADS=1 python -m ccgins.comparison`', '']
    (out/'report.md').write_text('\n'.join(lines))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='results/inekf_vs_eskf')
    ap.add_argument('--duration', type=float, default=600.0)
    args = ap.parse_args()
    seq = I.segment('i300', 0, args.duration)
    outages = I.outage_schedule(seq.truth.t[-1], durations=(30., 120., 60.))
    if not outages:
        ap.error('duration must allow at least one outage')
    write_report(compare(seq, outages), args.out)


if __name__ == '__main__':
    main()
