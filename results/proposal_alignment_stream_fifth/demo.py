"""Short legacy stream-learning demo; synthetic data, offline replay only."""
import json
from pathlib import Path
import torch
import yaml
from ccgins.learning import load
from ccgins.pipeline import Config
from ccgins.scenarios import make_scenario
from ccgins.stream_replay import save_artifact, replay

if __name__ == '__main__':
    torch.set_num_threads(1)
    out=Path(__file__).parent
    cfg=Config(**yaml.safe_load(Path('configs/proposal_alignment_full.yaml').read_text())['filter'])
    sc=make_scenario(715,duration=40.,outages=[(34.,5.)])
    save_artifact(out/'short_demo.npz',sc,cfg)
    nav,timing=replay(out/'short_demo.npz',load('results/motionnet.pt'))
    report=dict(source='synthetic IMU and synthetic GNSS; offline only',checkpoint='results/motionnet.pt',
        timing=timing,final_position=nav.filter.p.tolist(),aci_references=nav.cal._n,
        alarm_time=nav.alarm_time,recent_gnss=nav.gnss_decisions[-5:])
    (out/'short_demo.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
