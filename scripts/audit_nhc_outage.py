"""Isolate in-gap NHC effects with identical aiding before each ablated gap.

Use the existing precomputed-prediction interface to schedule constant zero
lateral/vertical observations. No learned model, truth observation or new online
configuration is introduced. Always verify the full-schedule replay against NHC.
"""
import argparse
from dataclasses import replace
import json
from pathlib import Path

import numpy as np

from ccgins.cli import _build_inputs, _filter_config, load_config
from ccgins.learning import WINDOW
from ccgins.pipeline import run
from scripts.audit_calibration import position_consistency
from scripts.audit_scheme import budget


def scheduled_nhc(truth, cfg, skip=None):
    if (cfg.pseudo != 'nhc' or cfg.aci or cfg.monitor != 'none'
            or cfg.mount_py is not None or cfg.corr_scale is not None):
        raise ValueError('requires fixed unmounted NHC without ACI or monitoring')
    indices = np.arange(cfg.pseudo_every, len(truth.t), cfg.pseudo_every)
    indices = indices[indices >= WINDOW]
    if skip is not None:
        start, duration = skip
        times = truth.t[indices]
        indices = indices[(times < start) | (times >= start + duration)]
    mean = np.zeros((len(indices), 3))
    variance = np.zeros_like(mean)
    variance[:, 1:] = cfg.nhc_variances()
    return indices, mean, variance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--seeds', type=int, nargs='+', default=[900, 80301, 80302])
    parser.add_argument('--init-seeds', type=int, nargs='+', default=[0, 1701])
    parser.add_argument('--factorial', action='store_true',
                        help='also cross in-gap aiding with NHC history before entry')
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    raw = load_config('configs/example_sim.yaml')
    result = dict(config=raw, factorial=args.factorial, rows=[], replay_checks=[])
    for seed in args.seeds:
        truth, imu, gnss, imu_spec, gnss_spec = _build_inputs(raw, seed_override=seed)
        for init_seed in args.init_seeds:
            cfg = replace(_filter_config(raw), init_seed=init_seed)
            native = run(truth, imu, gnss, imu_spec, gnss_spec, cfg)
            cases = [('replay', None, 'native', None)]
            for start, duration in gnss_spec.outages:
                end = start + duration
                entry_label = f'entry_off_{start:g}_{end:g}'
                cases.append((f'skip_{start:g}_{end:g}', (start, duration), 'native', start))
                if args.factorial:
                    cases.extend([
                        (entry_label, (0., start), None, None),
                        (f'both_off_{start:g}_{end:g}', (0., end), entry_label, start),
                    ])
            logs = {'native': native}
            for label, skip, reference_label, comparison_end in cases:
                pred = scheduled_nhc(truth, cfg, skip)
                log = run(truth, imu, gnss, imu_spec, gnss_spec,
                          replace(cfg, pseudo='learned_nhc'), pred=pred)
                # Exact equivalence is required before interpreting ablations.
                logs[label] = log
                if reference_label is not None:
                    reference = logs[reference_label]
                    before = (np.ones(len(log.t), bool) if comparison_end is None
                              else log.t < comparison_end)
                    for field in ('t', 'p_est', 'v_est', 'P_pos'):
                        np.testing.assert_array_equal(getattr(log, field)[before],
                                                      getattr(reference, field)[before])
                    result['replay_checks'].append(dict(
                        seed=seed, init_seed=init_seed, case=label,
                        reference=reference_label,
                        exact_prefix=True, samples=int(before.sum())))
                error = log.p_est - log.p_true
                row = dict(seed=seed, init_seed=init_seed, case=label,
                           skipped_interval=skip, update_epochs=len(pred[0]),
                           **position_consistency(log, gnss_spec.outages, raw['metrics']['t_start'],
                                              scenario_end=truth.t[-1]), bins=[])
                for start in range(30, 180, 10):
                    mask = (log.t >= start) & (log.t < start + 10)
                    row['bins'].append(dict(start=start, **budget(
                        log.t[mask], error[mask], [], start)['overall']))
                np.savez_compressed(args.out / f'trace_{seed}_{init_seed}_{label}.npz',
                                    t=log.t, error=error, velocity=log.v_est,
                                    position_covariance=log.P_pos)
                result['rows'].append(row)
                (args.out / 'metrics.json').write_text(
                    json.dumps(result, indent=2, allow_nan=False) + '\n')
                print(seed, init_seed, label, row['segments']['overall']['rmse_h'], flush=True)


if __name__ == '__main__':
    main()
