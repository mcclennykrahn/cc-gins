"""V0.8 Stage 3: frozen guard-reference-health diagnostics.

This script only runs the mechanism/semantics preregistered in protocol.md.  It
does not select detector thresholds, retune attacks/noise, or promote CUSUM.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
import json
from pathlib import Path

import numpy as np
import torch

from ccgins.learning import load, predict, WINDOW
from ccgins.pipeline import Config, run
from ccgins.scenarios import make_scenario
from ccgins.stream_learning import LearnedStreamingNavigator

ROOT = Path(__file__).resolve().parent
RATES = [0., .2, .5, 1.]
CONFIGS = [False, True]  # guard_ref_health off / on
# Historical Stage 2 base and its frozen diagnostic candidate.  Neither is
# promoted here; the candidate only supplies a fixed scenario-level monitor.
BASE = dict(filter='inekf', pseudo='hybrid', corr_scale=(38.02, 61.09, 39.1),
            aci=True, aci_gamma=.05, aci_window=100, hybrid_max_kappa=10.,
            hybrid_bias_z=4., hybrid_bias_latch=True, monitor='dual',
            dr_reset_period=30., recovery_epochs=0, cusum_k=3., cusum_h=50.)
PARITY_BASE = dict(pseudo='hybrid', aci=True, monitor='dual', recovery_epochs=3,
                   recovery_q=3., corr_scale=(40., 40., 40.), robust_gnss_delta=1.5)


def _num(v):
    return None if v is None else float(v)


def summarize(rows, flag):
    rr = [r for r in rows if r['guard_ref_health'] == flag]
    clean = [r for r in rr if r['rate'] == 0]
    out = dict(guard_ref_health=flag, clean=dict(
        runs=len(clean), exposure_s=600 * len(clean),
        alarms=sum(r['alarm_time'] is not None for r in clean),
        alarm_times=[r['alarm_time'] for r in clean if r['alarm_time'] is not None]),
        attacks={})
    for rate in RATES[1:]:
        a = [r for r in rr if r['rate'] == rate]
        out['attacks'][str(rate)] = dict(
            runs=len(a), detected=sum(r['delay_s'] is not None for r in a),
            misses=sum(r['delay_s'] is None for r in a),
            preattack_alarms=sum(r['alarm_time'] is not None and r['alarm_time'] < 300
                                 for r in a),
            delays_s=[r['delay_s'] for r in a if r['delay_s'] is not None])
    out['navigation'] = dict(
        mean_final_horizontal_error_m=float(np.mean([r['final_horizontal_error_m'] for r in rr])),
        max_final_horizontal_error_m=float(np.max([r['final_horizontal_error_m'] for r in rr])),
        resets=sum(r['resets'] for r in rr),
        reset_skips=sum(r['reset_skips'] for r in rr),
        fallback_epochs=sum(r['fallback_epochs'] for r in rr),
        reason_codes=_merge_reason_codes(rr))
    return out


def _merge_reason_codes(rows):
    out = {}
    for r in rows:
        for k, v in r['reason_codes'].items():
            out[k] = out.get(k, 0) + int(v > 0)
    return out


def worker(args):
    split, seed = args
    torch.set_num_threads(1)
    net = load('results/motionnet.pt')
    rows = []
    for rate in RATES:
        sc = make_scenario(seed, spoof_start=300. if rate else None, spoof_rate=rate)
        idx = np.arange(WINDOW, len(sc.truth.t), 10)
        with torch.no_grad():
            mu, var = predict(net, sc.imu6(), idx)
        for flag in CONFIGS:
            cfg = Config(init_seed=seed, guard_ref_health=flag, **BASE)
            log = run(sc.truth, sc.imu, sc.gnss, sc.imu_spec, sc.gnss_spec, cfg,
                      pred=(idx, mu, var))
            alarm = _num(log.alarm_time)
            grh = log.extra['guard_ref_health'] or {}
            rows.append(dict(
                split=split, seed=seed, rate=rate, guard_ref_health=flag,
                alarm_time=alarm,
                delay_s=None if not rate or alarm is None or alarm < 300 else alarm - 300,
                final_horizontal_error_m=float(np.linalg.norm(
                    log.p_est[-1, :2] - log.p_true[-1, :2])),
                resets=int(grh.get('resets', 0)),
                reset_skips=int(grh.get('reset_skips', 0)),
                fallback_epochs=int(log.extra['guard_ref_fallback_epochs']),
                first_fallback=_num(log.extra['guard_ref_first_fallback']),
                intervals=int(grh.get('intervals', 0)),
                reason_codes=grh.get('reason_codes', {}),
                history_reasons=grh.get('history_reasons', [])))
    print(f'completed {split} seed {seed}', flush=True)
    return rows


def _feed_stream(sc, cfg, net):
    nav = LearnedStreamingNavigator.from_truth(sc.truth, sc.imu_spec, sc.gnss_spec,
                                               cfg, model=net)
    j = 0
    for k, t in enumerate(sc.truth.t):
        while j < len(sc.gnss.t) and sc.gnss.t[j] <= t + 1e-9:
            if sc.gnss.valid[j]:
                nav.push_gnss(sc.gnss.t[j], sc.gnss.pos[j], sc.gnss.sigma**2)
            j += 1
        nav.push_imu(t, sc.imu.accel[k], sc.imu.gyro[k])
    nav.flush()
    return nav


def parity():
    torch.set_num_threads(1)
    net = load('results/motionnet.pt')
    cases = []
    for name, duration, outages in [('clean', 40., None), ('outage', 50., [(10., 4.)])]:
        sc = make_scenario(500, duration=duration, outages=outages)
        case = dict(case=name, duration_s=duration, outages=outages)
        for flag in CONFIGS:
            cfg = Config(guard_ref_health=flag, **PARITY_BASE)
            batch = run(sc.truth, sc.imu, sc.gnss, sc.imu_spec, sc.gnss_spec, cfg,
                        model=net, log_covariance=True)
            nav = _feed_stream(sc, cfg, net)
            case[str(flag)] = dict(
                batch_extra_guard=batch.extra['guard_ref_health'],
                batch_fallback_epochs=batch.extra['guard_ref_fallback_epochs'],
                stream_fallback_epochs=nav.guard_ref_fallbacks,
                max_position_delta=float(np.max(np.abs(
                    nav.get_state().position - batch.p_est[-1]))),
                max_covariance_delta=float(np.max(np.abs(
                    nav.get_state().covariance - batch.extra['covariance'][-1]))))
        cases.append(case)
    # Clean flag-on must be a no-op relative to the historical default.
    clean = next(c for c in cases if c['case'] == 'clean')
    off, on = clean['False'], clean['True']
    clean['flag_on_is_noop'] = bool(
        on['batch_fallback_epochs'] == 0 and on['batch_extra_guard']['reset_skips'] == 0
        and on['max_position_delta'] == off['max_position_delta'] == 0.0
        and on['max_covariance_delta'] == off['max_covariance_delta'] == 0.0)
    outage = next(c for c in cases if c['case'] == 'outage')
    outage['flag_on_observable_health'] = bool(
        outage['True']['batch_fallback_epochs'] > 0
        and outage['True']['batch_extra_guard']['reset_skips'] >= 1)
    return cases


def write(path, data):
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('phase', choices=['parity', 'validation', 'heldout', 'all'])
    ap.add_argument('--workers', type=int, default=8)
    args = ap.parse_args()
    out = {}
    if args.phase in ('parity', 'all'):
        out['parity'] = parity()
    for phase, split, seeds in [('validation', 'validation', list(range(500, 504))),
                                ('heldout', 'held_out_diagnostic', list(range(900, 910)))]:
        if args.phase not in (phase, 'all'):
            continue
        with ProcessPoolExecutor(max_workers=args.workers) as ex:
            nested = list(ex.map(worker, [(split, s) for s in seeds]))
        rows = sorted((r for rr in nested for r in rr),
                      key=lambda r: (r['seed'], r['rate'], r['guard_ref_health']))
        out[split] = dict(rows=rows,
                          summaries=[summarize(rows, f) for f in CONFIGS])
    write(ROOT / 'metrics.json', out)
    for key, val in out.items():
        if key == 'parity':
            print('parity', [(c['case'], c.get('flag_on_is_noop'),
                              c.get('flag_on_observable_health')) for c in val])
        else:
            for s in val['summaries']:
                print(key, s['guard_ref_health'], 'clean alarms',
                      s['clean']['alarms'], 'fallbacks', s['navigation']['fallback_epochs'])


if __name__ == '__main__':
    main()
