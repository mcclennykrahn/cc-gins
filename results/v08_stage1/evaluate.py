"""Locked mechanism evidence; no detection threshold tuning or promotion."""
import json
from pathlib import Path
import numpy as np
import torch
from ccgins.learning import load
from ccgins.pipeline import Config, run
from ccgins.scenarios import make_scenario
from ccgins.stream_learning import LearnedStreamingNavigator


def evaluate(seed, attacked=True, duration=12.):
    sc = make_scenario(seed, duration=duration)
    sc.gnss.valid[sc.gnss.idx == 0] = False
    # Separate gross transient mechanism attack; never replaces existing ramp definitions.
    if attacked:
        sc.gnss.pos[(sc.gnss.t >= 3.) & (sc.gnss.t <= 5.)] += [10000., 0., 0.]
    cfg = Config(pseudo='hybrid', aci=True, monitor='dual', recovery_epochs=3,
                 recovery_q=3., corr_scale=(40., 40., 40.), robust_gnss_delta=1.5)
    net = load('results/motionnet.pt')
    batch = run(sc.truth, sc.imu, sc.gnss, sc.imu_spec, sc.gnss_spec, cfg,
                model=net, log_covariance=True)
    nav = LearnedStreamingNavigator.from_truth(sc.truth, sc.imu_spec, sc.gnss_spec,
                                               cfg, model=net)
    j = 0
    for k, t in enumerate(sc.truth.t):
        while j < len(sc.gnss.t) and sc.gnss.t[j] <= t+1e-9:
            if sc.gnss.valid[j]:
                nav.push_gnss(sc.gnss.t[j], sc.gnss.pos[j], sc.gnss.sigma**2)
            j += 1
        nav.push_imu(t, sc.imu.accel[k], sc.imu.gyro[k])
    nav.flush()
    return dict(seed=seed, attacked=attacked, alarm_time=nav.alarm_time,
                batch_alarm_time=batch.alarm_time, recoveries=nav.cusum.recoveries,
                batch_recovery_times=batch.extra['recovery_times'],
                aci_observations=nav.cal._n, batch_aci_updates=len(batch.kappa_hist),
                position_max_abs=float(np.max(np.abs(nav.get_state().position-batch.p_est[-1]))),
                covariance_max_abs=float(np.max(np.abs(nav.get_state().covariance-batch.extra['covariance'][-1]))),
                gnss=nav.gnss_decisions)


if __name__ == '__main__':
    torch.set_num_threads(1)
    rows = {split: [evaluate(s, a) for s in seeds for a in (False, True)]
            for split, seeds in [('validation_mechanism', range(500, 504)),
                                 ('held_out_mechanism', range(900, 910))]}
    Path(__file__).with_name('metrics.json').write_text(json.dumps(rows, indent=2)+'\n')
    for split, rr in rows.items():
        print(split, 'clean alarms', sum(r['alarm_time'] is not None for r in rr if not r['attacked']),
              'attack detections', sum(r['alarm_time'] is not None for r in rr if r['attacked']),
              'attack recoveries', [r['recoveries'] for r in rr if r['attacked']],
              'max position/covariance delta', max(r['position_max_abs'] for r in rr),
              max(r['covariance_max_abs'] for r in rr))
