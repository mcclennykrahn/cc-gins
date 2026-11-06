"""Debug only: frozen smoke train/validation, no test-based selection."""
import json
import numpy as np
import torch
from ccgins import learning as L
from ccgins.scenarios import make_scenario
from ccgins.alignment_next import diagnostic
torch.set_num_threads(1)
train=[make_scenario(s,duration=80) for s in [100,101]]
net=L.train_motionnet([s.imu6() for s in train],[s.truth.v_body for s in train],epochs=6,batch=128,stride=20,architecture='proposal_tcn',variance_activation='smooth',attitude_list=[L.roll_pitch(s.truth.R) for s in train],bias_list=[np.c_[s.imu.gyro_bias,s.imu.accel_bias] for s in train])
sc=make_scenario(500,duration=80)
out={a:diagnostic(n,sc) for a,n in [('hard',L.load('results/proposal_alignment/proposal_tcn.pt')),('smooth',net)]}
print(json.dumps(out,indent=2))
open('results/proposal_alignment_debug/smooth_pilot.json','w').write(json.dumps(out,indent=2)+'\n')
