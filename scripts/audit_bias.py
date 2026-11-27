"""Offline bias/consistency diagnostic; never supplies truth to an update."""
import argparse
import json
from pathlib import Path
import numpy as np
from ccgins.cli import load_config, _build_inputs, _filter_config
from ccgins.pipeline import run
from scripts.audit_scheme import budget


def analyze(truth, imu, log, outages):
    idx = np.rint(log.t / truth.dt).astype(int)
    bias_error = log.extra['bias_est'] - np.c_[imu.gyro_bias[idx], imu.accel_bias[idx]]
    cov = log.extra['covariance']
    pe = log.p_est - log.p_true
    # Marginal NEES/dimension: correlations within each 3-vector are retained.
    pnees = np.einsum('ni,ni->n', pe, np.linalg.solve(log.P_pos, pe[..., None])[..., 0])/3
    bnees = []
    for j in (9, 12):
        e = bias_error[:, j-9:j-6]
        bnees.append(np.einsum('ni,ni->n', e, np.linalg.solve(cov[:,j:j+3,j:j+3], e[...,None])[...,0])/3)
    segments = budget(log.t, pe, outages)
    checkpoints = []
    for t in (30., 59.9, 89.9, 119.9, 179.9, 180.):
        k = int(np.argmin(abs(log.t-t)))
        if abs(log.t[k]-t) > 1e-6:
            continue
        checkpoints.append(dict(t=t, gyro_bias_error_deg_h=(bias_error[k,:3]*180/np.pi*3600).tolist(),
            accel_bias_error_mg=(bias_error[k,3:]/9.81*1000).tolist(),
            gyro_bias_sigma_deg_h=(np.sqrt(np.diag(cov[k])[9:12])*180/np.pi*3600).tolist(),
            accel_bias_sigma_mg=(np.sqrt(np.diag(cov[k])[12:15])/9.81*1000).tolist(),
            position_nees_per_dim=float(pnees[k]), gyro_bias_nees_per_dim=float(bnees[0][k]),
            accel_bias_nees_per_dim=float(bnees[1][k])))
    for name, a, b in [('available_30_60',30,60),('outage_60_90',60,90),
                       ('available_90_120',90,120),('outage_120_180',120,180)]:
        m=(log.t>=a)&(log.t<b)
        segments[name]['mean_position_nees_per_dim']=float(pnees[m].mean())
        segments[name]['position_95pct_coverage']=float(np.mean(pnees[m]*3<=7.814727903251179))
        segments[name]['mean_gyro_bias_nees_per_dim']=float(bnees[0][m].mean())
        segments[name]['mean_accel_bias_nees_per_dim']=float(bnees[1][m].mean())
    return dict(segments=segments, checkpoints=checkpoints)


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    args=ap.parse_args()
    out=Path(args.out)
    out.mkdir(parents=True,exist_ok=True)
    raw=load_config('configs/example_sim.yaml')
    result=dict(config=raw, diagnostic_only=True, rows=[])
    for seed in (900,49501,49502):
        truth,imu,gnss,spec,gs=_build_inputs(raw,seed_override=seed)
        for init in (0,17):
            cfg=_filter_config(raw)
            cfg.init_seed=init
            log=run(truth,imu,gnss,spec,gs,cfg,log_covariance=True)
            row=dict(seed=seed,init_seed=init,**analyze(truth,imu,log,gs.outages))
            result['rows'].append(row)
            (out/'bias_budget.json').write_text(json.dumps(result,indent=2)+'\n')
            print(seed,init,row['segments']['overall']['rmse_h'],flush=True)

if __name__=='__main__':
    main()
