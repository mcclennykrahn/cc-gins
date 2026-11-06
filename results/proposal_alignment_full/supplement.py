"""Post-training calibration-only diagnostics; never retunes primary comparison."""
import json
import hashlib
from dataclasses import replace
from pathlib import Path
import numpy as np
import torch
import yaml
from ccgins import learning as L
from ccgins.scenarios import make_scenario
from ccgins.pipeline import Config, run
from ccgins.metrics import summarize

out = Path('results/proposal_alignment_full')
c = yaml.safe_load(Path('configs/proposal_alignment_full.yaml').read_text())
torch.set_num_threads(1)
calibration = {}
for arch in ['legacy_cnn', 'proposal_tcn']:
    net = L.load(out/'training'/f'{arch}_smooth.pt')
    groups, records = [], []
    for seed in c['calibration_seeds']:
        sc = make_scenario(seed, duration=c['duration'])
        idx = np.arange(L.WINDOW, len(sc.truth.t), 10)
        mu, var = L.predict(net, sc.imu6(), idx, attitude=L.roll_pitch(sc.truth.R),
            bias=np.c_[sc.imu.gyro_bias, sc.imu.accel_bias], batch=512)
        z = (mu-sc.truth.v_body[idx-1])/np.sqrt(var)
        groups.append(z)
        records.append(dict(seed=seed, samples=len(z), corr_scale=L.correlation_factor(z).tolist(),
                            mean_standardized_residual=z.mean(0).tolist()))
    calibration[arch] = dict(checkpoint_class=type(net).__name__, variance_activation=net.variance_activation,
        parameter_count=sum(p.numel() for p in net.parameters()),
        checkpoint_sha256=hashlib.sha256((out/'training'/f'{arch}_smooth.pt').read_bytes()).hexdigest(), groups=records, grouped_corr_scale=L.grouped_correlation_factor(groups).tolist(),
        aggregation='equal-drive mean; each drive independently centered; no boundary lag pairs',
        diagnostic_only=True, primary_corr_scale=c['filter']['corr_scale'])
(out/'grouped_calibration.json').write_text(json.dumps(calibration, indent=2)+'\n')
print('grouped calibration', calibration, flush=True)
# Independent calibration seed pilot, no held-out test selection; supported batch path.
sc = make_scenario(700, duration=120., outages=[(80., 20.)])
net = L.load(out/'training'/'legacy_cnn_smooth.pt')
rows = []
for monitor in ['none', 'dual']:
    for enabled in [False, True]:
        cfg = Config(**{**c['filter'], 'monitor':monitor, 'gnss_aci':enabled})
        log = run(sc.truth, sc.imu, sc.gnss, sc.imu_spec, sc.gnss_spec, cfg, model=net)
        rows.append(dict(seed=700, duration=120., outages=[[80.,20.]], monitor=monitor,
            gnss_aci=enabled, navigation=summarize(log, [(80.,20.)], t_start=30),
            alarm_time=log.alarm_time, gnss_aci_trace=log.extra['gnss_aci']))
        print('GNSS ACI pilot', monitor, enabled, rows[-1]['navigation']['rmse_h'], flush=True)
(out/'gnss_aci_pilot.json').write_text(json.dumps(dict(data_source='synthetic IMU and synthetic 1 Hz GNSS',
    diagnostic_only=True, records=rows), indent=2)+'\n')
