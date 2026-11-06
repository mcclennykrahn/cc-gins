"""Local reproducible synthetic demo; no training or model selection."""
import copy
import json
import platform
import time
from pathlib import Path

import numpy as np
import torch

from ccgins.learning import load
from ccgins.pipeline import Config, run
from ccgins.scenarios import make_scenario
from ccgins.stream_learning import LearnedStreamingNavigator

OUT = Path(__file__).parent

def drive(sc, cfg, net):
    nav = LearnedStreamingNavigator.from_truth(sc.truth, sc.imu_spec, sc.gnss_spec,
                                               cfg, model=net, delay=.05)
    js = list(np.flatnonzero(sc.gnss.valid & (sc.gnss.idx > 0)))
    j = 0
    for k, t in enumerate(sc.truth.t):
        while j < len(js) and sc.gnss.t[js[j]]+.03 <= t+1e-9:
            m = js[j]
            nav.push_gnss(sc.gnss.t[m], sc.gnss.pos[m], sc.gnss.sigma**2)
            j += 1
        nav.push_imu(t, sc.imu.accel[k], sc.imu.gyro[k])
    while j < len(js):
        m = js[j]
        nav.push_gnss(sc.gnss.t[m], sc.gnss.pos[m], sc.gnss.sigma**2)
        j += 1
    nav.flush()
    return nav


def main():
    torch.set_num_threads(1)
    sc = make_scenario(715, duration=40., outages=[(34., 5.)])
    sc.gnss.valid[sc.gnss.idx == 0] = False
    # Diagnostic no-alarm threshold isolates inference/ACI/reset parity. This is
    # not a calibrated integrity threshold; attack checks use h=50 in pytest.
    cfg = Config(pseudo='learned_vel', aci=True, gnss_aci=True, monitor='dual',
                 cusum_h=1e12, corr_scale=(40., 40., 40.), gnss_gate=1e20)
    measurements = []
    for arch in ('legacy_cnn', 'proposal_tcn'):
        net = load(f'results/proposal_alignment_full/{arch}.pt')
        start = time.perf_counter()
        nav = drive(sc, cfg, net)
        elapsed = time.perf_counter()-start
        batch = run(sc.truth, sc.imu, sc.gnss, sc.imu_spec, sc.gnss_spec, cfg,
                    model=net, log_covariance=True)
        dp = float(np.max(np.abs(nav.filter.p-batch.p_est[-1])))
        dc = float(np.max(np.abs(nav.filter.P-batch.extra['covariance'][-1])))
        dq = float(np.max(np.abs(np.array([r['q'] for r in nav.gnss_decisions])-batch.stat_q)))
        assert dp < 1e-8 and dc < 1e-8 and dq < 1e-8
        frozen = [r for r in nav.decisions if 35.6 <= r['t'] < 39.]
        assert len({r['observations'] for r in frozen}) == 1
        assert not any(r['eligible'] for r in frozen)
        assert nav.cal._n > 30 and nav.gnss_cal.n > 0
        measurements.append(dict(architecture=arch, imu_samples=len(sc.truth.t),
            wall_seconds=elapsed, processed_hz=len(sc.truth.t)/elapsed,
            duration_seconds=40., pseudo_decisions=len(nav.decisions),
            gnss_decisions=len(nav.gnss_decisions), aci_observations=nav.cal._n,
            gnss_aci_observations=nav.gnss_cal.n, frozen_decisions=len(frozen),
            position_max_abs_difference=dp, covariance_max_abs_difference=dc,
            statistic_max_abs_difference=dq, alarm_time=nav.alarm_time))
        (OUT/f'{arch}_trace.json').write_text(json.dumps(dict(
            pseudo=nav.decisions, gnss=nav.gnss_decisions), indent=2)+'\n')
    result = dict(data='synthetic IMU and synthetic GNSS; seed 715',
        python=platform.python_version(), torch=torch.__version__, torch_threads=1,
        delay_seconds=.05, gnss_arrival_lag_seconds=.03, config=vars(cfg), runs=measurements)
    (OUT/'metrics.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))

if __name__ == '__main__':
    main()
