"""Frozen-checkpoint diagnostics, no fitting/selection and no test reranking."""
import json
import re
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import torch
from torch import nn

from ccgins import learning as L
from ccgins.scenarios import make_scenario

OUT=Path(__file__).parent

def stats(mu, var, y, times):
    e=mu-y
    trend=np.c_[np.ones(len(times)), times-times.mean()]
    fit=trend@np.linalg.lstsq(trend, e, rcond=None)[0]
    return dict(finite=bool(np.isfinite(mu).all() and np.isfinite(var).all()),
        max_abs_mu=np.abs(mu).max(0).tolist(), mean_error=e.mean(0).tolist(),
        rmse=np.sqrt((e**2).mean(0)).tolist(),
        linear_detrended_rmse=np.sqrt(((e-fit)**2).mean(0)).tolist(),
        error_slope_per_second=np.linalg.lstsq(trend,e,rcond=None)[0][1].tolist(),
        opening_1_to_5s_rmse=np.sqrt((e[times<=5]**2).mean(0)).tolist(),
        later_rmse=np.sqrt((e[times>5]**2).mean(0)).tolist(),
        variance_min=var.min(0).tolist(), variance_max=var.max(0).tolist())

def predict(net, sc, idx, raw=None, rp=None):
    return L.predict(net, sc.imu6() if raw is None else raw, idx,
        attitude=L.roll_pitch(sc.truth.R) if rp is None else rp,
        bias=np.c_[sc.imu.gyro_bias,sc.imu.accel_bias] if raw is None else None,
        batch=256)

def main():
    torch.set_num_threads(1)
    result=dict(data='synthetic IMU + synthetic GNSS; reference-conditioned offline probes',
                decision='available, not default; no retraining or model selection', architectures={}, seeds=[], probes={})
    nets={a:L.load(f'results/proposal_alignment_full/{a}.pt') for a in ['legacy_cnn','proposal_tcn']}
    for a,net in nets.items():
        result['architectures'][a]=dict(parameters=sum(p.numel() for p in net.parameters()),
            layers=[dict(kernel=c.kernel_size[0], dilation=c.dilation[0], stride=c.stride[0],
                         in_channels=c.in_channels,out_channels=c.out_channels)
                    for c in net.conv if isinstance(c,nn.Conv1d)])
    for seed in list(range(100,120))+list(range(500,504)):
        sc=make_scenario(seed, duration=600.)
        idx=np.arange(L.WINDOW,len(sc.truth.t),100)
        for arch,net in nets.items():
            mu,var=predict(net,sc,idx)
            result['seeds'].append(dict(seed=seed,split='train' if seed<120 else 'val',
                architecture=arch, **stats(mu,var,sc.truth.v_body[idx-1], sc.truth.t[idx-1])))
        if seed==500:
            idx=np.arange(L.WINDOW+1,len(sc.truth.t)-1,20)
            net=nets['proposal_tcn']; rp=L.roll_pitch(sc.truth.R)
            raw=sc.imu6()-np.c_[sc.imu.gyro_bias,sc.imu.accel_bias]
            mu,var=predict(net,sc,idx)
            y=sc.truth.v_body[idx-1]
            tests={}
            def probe(name,x,att,transform):
                m,v=predict(net,sc,idx,raw=x,rp=att)
                tests[name]=dict(equivariance_rmse=np.sqrt(((m-mu*transform)**2).mean(0)).tolist(),
                    transformed_label_rmse=np.sqrt(((m-y*transform)**2).mean(0)).tolist(),
                    finite=bool(np.isfinite(m).all() and np.isfinite(v).all()))
            probe('left_right_physical_reflection',raw*np.array([-1,1,-1,1,-1,1]),rp*np.array([-1,1]),np.array([1,-1,1]))
            probe('body_axis_yaw_pi',raw*np.array([-1,-1,1,-1,-1,1]),rp*np.array([-1,-1]),np.array([-1,-1,1]))
            probe('roll_pitch_sign_only_inconsistent',raw,rp*-1,np.ones(3))
            probe('gyro_accel_swapped_invalid',raw[:,[3,4,5,0,1,2]],rp,np.ones(3))
            jitter={}
            for shift in [-1,0,1]:
                m,v=predict(net,sc,idx+shift)
                jitter[str(shift)]=dict(rmse=np.sqrt(((m-sc.truth.v_body[idx+shift-1])**2).mean(0)).tolist(),
                    prediction_change_rmse=np.sqrt(((m-mu)**2).mean(0)).tolist())
            fuzz_idx=idx+np.random.default_rng(42).integers(-10,11,len(idx))
            fuzz_idx=np.clip(fuzz_idx,L.WINDOW,len(sc.truth.t)-1)
            fm,fv=predict(net,sc,fuzz_idx)
            jitter['random_plus_minus_10_samples']=dict(
                rmse=np.sqrt(((fm-sc.truth.v_body[fuzz_idx-1])**2).mean(0)).tolist(),
                prediction_change_rmse=np.sqrt(((fm-mu)**2).mean(0)).tolist())
            vb=np.einsum('nji,nj->ni',sc.truth.R,sc.truth.v)
            result['probes']=dict(seed=500,split='val',transforms=tests,adjacency_jitter=jitter,
                label_rotation_max_abs_difference=float(np.abs(vb-sc.truth.v_body).max()),
                channel_order='gyro xyz, accel xyz, roll pitch; target body forward left up at idx-1',
                split_adjacency='Independent seeds 100–119 versus 500–503; no within-drive boundary split. Jitter stays within validation 500.')
    prior=json.loads(Path('results/proposal_alignment_full/metrics.json').read_text())
    result['preexisting_test_channel_diagnosis']=[dict(seed=r['seed'], architecture=r['architecture'],
        rmse=r['network']['rmse_fwd_lat_up'], nll=r['network']['nll_channels'])
        for r in prior['records'] if r['split']=='test' and 'network' in r]
    text=Path('results/proposal_alignment_full/run.log').read_text()
    curves=[]; curve=[]
    for line in text.splitlines():
        m=re.match(r'epoch (\d+)/(\d+) beta-NLL ([-.\d]+)',line)
        if m:
            if int(m[1])==1 and curve: curves.append(curve); curve=[]
            curve.append(float(m[3]))
    if curve: curves.append(curve)
    result['preexisting_training_curves']=curves
    fig,ax=plt.subplots(figsize=(7,4))
    for a,c in zip(nets,curves): ax.plot(np.arange(1,len(c)+1),c,marker='o',label=a)
    ax.set(xlabel='Epoch',ylabel='Training beta-NLL',title='Original-budget frozen training log (pre-existing)')
    ax.legend(); fig.tight_layout(); fig.savefig(OUT/'training_curves.png',dpi=150); plt.close(fig)
    (OUT/'metrics.json').write_text(json.dumps(result,indent=2)+'\n')
    print('Architecture:',result['architectures'])
    print('Seed records:',len(result['seeds']),'finite:',sum(r['finite'] for r in result['seeds']))
    print('Probes:',result['probes'])

if __name__=='__main__': main()
