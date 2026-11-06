"""Localize published smoke tails; diagnostics only, never selection."""
import json
import numpy as np
import torch
from ccgins import learning as L
from ccgins.scenarios import make_scenario
torch.set_num_threads(1)
net=L.load('results/proposal_alignment/proposal_tcn.pt'); rows=[]
for seed in [100,101,500,900,901]:
 sc=make_scenario(seed,duration=80); raw=[]
 hook=net.head.register_forward_hook(lambda m,a,out:raw.append(out.detach().numpy()))
 idx=np.arange(100,len(sc.truth.t),10)
 mu,var=L.predict(net,sc.imu6(),idx,attitude=L.roll_pitch(sc.truth.R),bias=np.c_[sc.imu.gyro_bias,sc.imu.accel_bias],batch=512)
 hook.remove(); raw=np.concatenate(raw)[:,3:]
 features=L.proposal_inputs(sc.imu6(),L.roll_pitch(sc.truth.R),np.c_[sc.imu.gyro_bias,sc.imu.accel_bias])
 z=(features-net.x_mean.numpy())/net.x_std.numpy()
 rows.append(dict(seed=seed,raw_logvar_min=raw.min(0).tolist(),fraction_raw_below_lower=(raw < -12).mean(0).tolist(),fraction_raw_above_upper=(raw > 4).mean(0).tolist(),normalized_channel_absmax=np.abs(z).max(0).tolist()))
open('results/proposal_alignment_debug/tail_probe.json','w').write(json.dumps(rows,indent=2)+'\n')
print(json.dumps(rows,indent=2))
