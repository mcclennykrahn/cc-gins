import json
import numpy as np
import torch
from ccgins import learning as L
from ccgins.scenarios import make_scenario
torch.set_num_threads(1)
out={}
for arch in ['legacy_cnn','proposal_tcn']:
 net=L.load('results/proposal_alignment/'+arch+'.pt'); rows=[]
 for seed in [500,900,901]:
  s=make_scenario(seed,duration=80); idx=np.arange(100,len(s.truth.t),10)
  mu,v=L.predict(net,s.imu6(),idx,attitude=L.roll_pitch(s.truth.R),bias=np.c_[s.imu.gyro_bias,s.imu.accel_bias]); e=mu-s.truth.v_body[idx-1]
  rows.append(dict(seed=seed,nll_channels=np.mean(.5*(np.log(2*np.pi*v)+e*e/v),0).tolist(),var_quantiles=np.quantile(v,[0,.01,.5,.99,1],axis=0).tolist(),worst_forward=dict(nll=float(np.max(.5*(np.log(2*np.pi*v[:,0])+e[:,0]**2/v[:,0]))),mu=float(mu[np.argmax(e[:,0]**2/v[:,0]),0]),var=float(v[np.argmax(e[:,0]**2/v[:,0]),0]))))
 out[arch]=rows
print(json.dumps(out,indent=2))
