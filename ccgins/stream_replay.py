"""One-file offline artifact replay. No receiver, clock or hardware I/O support."""
import argparse
import json
import time
from dataclasses import asdict
from pathlib import Path

import numpy as np

from .filters import NoiseParams
from .pipeline import Config, initial_state
from .stream_learning import LearnedStreamingNavigator


def save_artifact(path, sc, cfg, lag=1., delay=1.01):
    """NPZ v1: full IMU, valid GNSS, arrival epochs, explicit initialization/config."""
    R, v, p, P = initial_state(sc.truth, cfg.init_seed, cfg.p0_bias)
    ii = np.flatnonzero(sc.gnss.valid & (sc.gnss.idx > 0))
    meta = dict(version=1, source='synthetic IMU and synthetic GNSS', config=asdict(cfg),
                noise=asdict(NoiseParams.from_spec(sc.imu_spec, dt=sc.truth.dt)), dt=sc.truth.dt, delay=delay)
    np.savez_compressed(path, metadata=json.dumps(meta), t=sc.truth.t,
        accel=sc.imu.accel, gyro=sc.imu.gyro, gnss_t=sc.gnss.t[ii],
        arrival=sc.gnss.t[ii]+lag, pos=sc.gnss.pos[ii],
        cov=np.tile(np.diag(sc.gnss.sigma**2), (len(ii), 1, 1)),
        R=R, v=v, p=p, P=P, lever=sc.gnss_spec.lever_arm)


def replay(path, model, callback=None):
    with np.load(path, allow_pickle=False) as archive:
        # Materialize once: NpzFile indexing decompresses the entire member.
        a = {key: archive[key] for key in archive.files}
        meta = json.loads(str(a['metadata']))
        if meta['version'] != 1:
            raise ValueError('unsupported replay version')
        nav = LearnedStreamingNavigator(a['R'], a['v'], a['p'], a['P'],
            NoiseParams(**meta['noise']), model=model, motion_cfg=Config(**meta['config']),
            dt=meta['dt'], delay=meta['delay'], lever_arm=a['lever'], callback=callback,
            kind=meta['config']['filter'], gate=meta['config']['gnss_gate'],
            max_consecutive_rejects=meta['config']['max_consecutive_rejects'],
            robust_gnss_delta=meta['config']['robust_gnss_delta'],
            earth=None if meta['config']['earth_rate_enu'] is None and meta['config']['local_gravity'] is None else
            dict(w_ie=meta['config']['earth_rate_enu'], g=np.array([0.,0.,-(meta['config']['local_gravity'] or 9.81)])))
        order = np.argsort(a['arrival'], kind='stable')
        j = 0
        timings = []
        start = time.perf_counter()
        for k, t in enumerate(a['t']):
            before = time.perf_counter()
            while j < len(order) and a['arrival'][order[j]] <= t+1e-9:
                m = order[j]
                nav.push_gnss(a['gnss_t'][m], a['pos'][m], a['cov'][m], arrival_time=float(a['arrival'][m]))
                j += 1
            nav.push_imu(t, a['accel'][k], a['gyro'][k])
            timings.append(time.perf_counter()-before)
        while j < len(order):
            m = order[j]
            nav.push_gnss(a['gnss_t'][m], a['pos'][m], a['cov'][m], arrival_time=float(a['arrival'][m]))
            j += 1
        before = time.perf_counter()
        nav.flush()
        tail = time.perf_counter()-before
        elapsed = time.perf_counter()-start
        if nav._gnss:
            raise ValueError("replay GNSS exceeds final IMU epoch")
    return nav, dict(imu_samples=len(timings), wall_seconds=elapsed,
        mean_ms=float(np.mean(timings)*1000), p95_ms=float(np.percentile(timings,95)*1000),
        flush_ms=tail*1000, hz=len(timings)/elapsed, buffering_delay_s=meta['delay'])


def main():
    from .learning import load
    import torch
    torch.set_num_threads(1)
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('artifact'); ap.add_argument('--checkpoint', default='results/motionnet.pt')
    a = ap.parse_args()
    nav, timing = replay(a.artifact, load(a.checkpoint))
    print(json.dumps(dict(offline_only=True, timing=timing, final_position=nav.filter.p.tolist())))

if __name__ == '__main__':
    main()
