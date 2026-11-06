"""Stationary synthetic mechanism fixture: 1 m/s ramp, 100-sample arrival lag."""
import json
from pathlib import Path
import numpy as np
import torch
from ccgins.learning import load
from ccgins.pipeline import Config
from ccgins.filters import NoiseParams
from ccgins.sim import ImuSpec
from ccgins.stream_learning import LearnedStreamingNavigator

def probe(attacked, h=7., exclude=False):
 net=load('results/motionnet.pt')
 cfg=Config(pseudo='learned_nhc',monitor='dual',cusum_h=h,cusum_k=.5,
            exclude_on_alarm=exclude,gnss_gate=1e12,dr_reset_period=30.)
 nav=LearnedStreamingNavigator(np.eye(3),np.zeros(3),np.zeros(3),np.eye(15)*1e-6,
     NoiseParams(1e-5,1e-3,1e-7,1e-5),motion_cfg=cfg,model=net,dt=.01,delay=1.01,gate=cfg.gnss_gate)
 for k in range(2001):
  t=k*.01
  if k>=100 and k%100==0:
   epoch=t-1.
   shift=max(0.,epoch-5.) if attacked and epoch<=13 else 0.
   nav.push_gnss(epoch,[shift,0,0],.25,arrival_time=t)
  nav.push_imu(t,[0,0,9.81],[0,0,0])
 nav.flush()
 return dict(attacked=attacked,exclude_on_alarm=exclude,alarm_time=nav.alarm_time,gnss=nav.gnss_decisions)

if __name__=='__main__':
 torch.set_num_threads(1)
 rows=[probe(False),probe(True),probe(True,exclude=True)]
 Path(__file__).with_name('spoof.json').write_text(json.dumps(rows,indent=2)+'\n')
 for r in rows: print(r['attacked'],r['alarm_time'],[(g['t'],round(g['q'],2),round(g['g'],2),g['accepted']) for g in r['gnss']])
