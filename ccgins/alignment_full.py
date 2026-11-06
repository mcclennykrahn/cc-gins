"""Original twenty-seed training followed by locked nominal paired evaluation."""
from concurrent.futures import ProcessPoolExecutor, as_completed
import multiprocessing
import hashlib
import json
from pathlib import Path
from dataclasses import asdict
import numpy as np
import torch
import yaml
from scipy.stats import chi2
from . import learning as L
from .scenarios import make_scenario
from .pipeline import Config, run
from .metrics import summarize

OUT = Path('results/proposal_alignment_full')
DEBUG = OUT / 'training'


def diagnostic(net, sc):
    idx = np.arange(L.WINDOW, len(sc.truth.t), 10)
    mu, var = L.predict(net, sc.imu6(), idx, attitude=L.roll_pitch(sc.truth.R),
                       bias=np.c_[sc.imu.gyro_bias, sc.imu.accel_bias], batch=512)
    e = mu - sc.truth.v_body[idx-1]
    nll = .5*(np.log(2*np.pi*var)+e**2/var)
    return dict(gaussian_nll=float(nll.mean()), nll_channels=nll.mean(0).tolist(),
                rmse_fwd_lat_up=np.sqrt((e**2).mean(0)).tolist(),
                coverage95_fwd_lat_up=(e**2/var <= chi2.ppf(.95, 1)).mean(0).tolist(),
                var_quantiles=np.quantile(var,[0,.01,.5,.99,1],axis=0).tolist())


def navigation_record(job):
    arch,split,seed,c,selected=job
    torch.set_num_threads(1)
    net=None if arch=='nhc' else L.load(DEBUG/f'{arch}_{selected}.pt')
    sc=make_scenario(seed,duration=c['duration'],outages=c['outages'])
    cfg=Config(**(c['filter'] if net is not None else {**c['filter'],'pseudo':'nhc','aci':False,'monitor':'none','corr_scale':None}))
    log=run(sc.truth,sc.imu,sc.gnss,sc.imu_spec,sc.gnss_spec,cfg,model=net)
    record=dict(architecture=arch,split=split,seed=seed,navigation=summarize(log,c['outages'],t_start=30))
    if net is not None: record['network']=diagnostic(net,sc)
    return record


def main():
    path=Path('configs/proposal_alignment_full.yaml'); c=yaml.safe_load(path.read_text())
    torch.set_num_threads(c['threads']); OUT.mkdir(exist_ok=True); DEBUG.mkdir(exist_ok=True)
    frozen=DEBUG/'protocol.yaml'
    if frozen.exists() and frozen.read_bytes() != path.read_bytes():
        raise ValueError('cached training protocol differs; use a new output directory')
    frozen.write_bytes(path.read_bytes())
    train=[make_scenario(s,duration=c['duration']) for s in c['train_seeds']]
    vals=[make_scenario(s,duration=c['duration'],outages=c['outages']) for s in c['val_seeds']]
    pilots=[]
    for activation in c['variance_candidates']:
        for arch in ['legacy_cnn','proposal_tcn']:
            checkpoint=DEBUG/f'{arch}_{activation}.pt'
            if checkpoint.exists():
                net=L.load(checkpoint)
            else:
                net=L.train_motionnet([s.imu6() for s in train],[s.truth.v_body for s in train],
                    epochs=c['epochs'],batch=c['batch'],lr=c['lr'],stride=c['stride'],seed=c['training_seed'],
                    architecture=arch,variance_activation=activation,
                    attitude_list=[L.roll_pitch(s.truth.R) for s in train] if arch=='proposal_tcn' else None,
                    bias_list=[np.c_[s.imu.gyro_bias,s.imu.accel_bias] for s in train] if arch=='proposal_tcn' else None)
                L.save(net,checkpoint)
            (DEBUG/f'{arch}_{activation}_completed.json').write_text(json.dumps(dict(train_seeds=c['train_seeds'], duration=c['duration'], epochs=c['epochs'], checkpoint_sha256=hashlib.sha256(checkpoint.read_bytes()).hexdigest()),indent=2)+'\n')
            for sc in vals:
                pilots.append(dict(architecture=arch,activation=activation,seed=sc.seed,split='val',
                                   network=diagnostic(net,sc)))
            (DEBUG/'validation_pilots.json').write_text(json.dumps(pilots,indent=2)+'\n')
            print('PILOT',arch,activation, np.mean([p['network']['gaussian_nll'] for p in pilots if p['architecture']==arch and p['activation']==activation]),flush=True)
    scores={a:float(np.mean([p['network']['gaussian_nll'] for p in pilots if p['activation']==a])) for a in c['variance_candidates']}
    selected=min(scores,key=scores.get)
    selection=dict(activation=selected,criterion='mean reference-conditioned validation Gaussian NLL over both architectures and all validation seeds',scores=scores,config_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    (OUT/'selection.json').write_text(json.dumps(selection,indent=2)+'\n')
    print('LOCKED SELECTION',selection,flush=True)
    payload=dict(config=c,selection=selection,resolved_filter=asdict(Config(**c['filter'])),records=[],aggregates={},data_source='synthetic simulation IMU + synthetic GNSS',training='original 20 seeds 100–119 x 600 s completed for both architectures; not full M3 acceptance')
    if (OUT/'metrics.json').exists():
        previous=json.loads((OUT/'metrics.json').read_text())
        if previous['config'] != c or previous['selection'] != selection:
            raise ValueError('output contains a different protocol; use a new directory')
        payload['records']=previous['records']
    completed={(r['architecture'],r['split'],r['seed']) for r in payload['records']}
    jobs=[]
    for arch in ['legacy_cnn','proposal_tcn','nhc']:
        if arch != 'nhc': L.save(L.load(DEBUG/f'{arch}_{selected}.pt'),OUT/f'{arch}.pt')
        for split in (['test'] if arch=='nhc' else ['val','test']):
            for seed in c[f'{split}_seeds']:
                if (arch,split,seed) not in completed: jobs.append((arch,split,seed,c,selected))
    # Independent local processes only; each uses one Torch/BLAS thread.
    with ProcessPoolExecutor(max_workers=3,mp_context=multiprocessing.get_context('spawn')) as pool:
        futures=[pool.submit(navigation_record,job) for job in jobs]
        for future in as_completed(futures):
            record=future.result()
            payload['records'].append(record)
            payload['records'].sort(key=lambda r:(r['architecture'],r['split'],r['seed']))
            (OUT/'metrics.json').write_text(json.dumps(payload,indent=2)+'\n')
            message=f"NAV {record['architecture']} {record['split']} {record['seed']} RMSE_h={record['navigation']['rmse_h']}"
            print(message,flush=True)
            with open('/workspace/document_alignment_ccgins_third.log','a') as f: f.write(message+'\n')
    for arch in ['legacy_cnn','proposal_tcn','nhc']:
        rows=[r for r in payload['records'] if r['architecture']==arch and r['split']=='test']
        payload['aggregates'][arch]={key:float(np.mean([r['navigation'][key] for r in rows])) for key in ['rmse_h','coverage95_h','coverage95_h_outage']}
        payload['aggregates'][arch]['gaussian_nll']=None if arch=='nhc' else float(np.mean([r['network']['gaussian_nll'] for r in rows]))
    (OUT/'metrics.json').write_text(json.dumps(payload,indent=2)+'\n')
    print(json.dumps(payload['aggregates'],indent=2),flush=True)

if __name__=='__main__': main()
