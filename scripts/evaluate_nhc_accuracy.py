"""Paired accuracy check; fixed candidate, development and held-out seeds.

Run from repo root with PYTHONPATH=. and one BLAS thread for reproducibility.
"""
import json
from dataclasses import replace
from pathlib import Path

from ccgins.cli import load_config, _build_inputs, _filter_config
from ccgins.metrics import summarize
from ccgins.pipeline import run


def main():
    raw = load_config('configs/example_sim.yaml')
    cfg = _filter_config(raw)
    result = dict(config=raw, candidate={'nhc_vertical_sigma': .03},
                  development_seeds=[900, 901, 902], held_out_seeds=list(range(19101, 19106)), rows=[])
    target = Path('results/cc_accuracy_s1/paired_metrics.json')
    target.parent.mkdir(parents=True, exist_ok=True)
    for seed in result['development_seeds'] + result['held_out_seeds']:
        truth, imu, gnss, spec, gs = _build_inputs(raw, seed_override=seed)
        for name, config in [('baseline', cfg), ('candidate', replace(cfg, nhc_vertical_sigma=.03))]:
            log = run(truth, imu, gnss, spec, gs, config)
            metrics = summarize(log, gs.outages, t_start=raw['metrics']['t_start'])
            result['rows'].append(dict(seed=seed, variant=name, metrics=metrics))
            target.write_text(json.dumps(result, indent=2)+'\n')
            print(seed, name, metrics['rmse_h'], flush=True)


if __name__ == '__main__':
    main()
