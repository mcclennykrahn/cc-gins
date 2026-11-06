from pathlib import Path
import numpy as np
import pytest
import torch
from ccgins import learning
from ccgins.pipeline import Config, run
from ccgins.scenarios import make_scenario
from ccgins.cli import main


def test_tcn_heads_dilations_attitude_and_roundtrip(tmp_path):
    torch.manual_seed(2)
    net = learning.MotionNetTCN().eval()
    convs = [m for m in net.conv if isinstance(m, torch.nn.Conv1d)]
    assert [m.dilation[0] for m in convs] == [1, 2, 4]
    assert net.conv(torch.zeros(1,8,100)).shape == (1,64,100)
    x = torch.randn(2,100,8)
    mu, lv = net(x)
    assert mu.shape == lv.shape == (2,3)
    y=x.clone(); y[:,:,6:] += .5
    assert not torch.allclose(mu,net(y)[0])
    loss=learning.beta_nll(mu,lv,torch.ones_like(mu));loss.backward()
    assert all(p.grad is not None and torch.isfinite(p.grad).all() for p in net.parameters())
    learning.save(net,tmp_path/'tcn.pt')
    loaded=learning.load(tmp_path/'tcn.pt',expected_architecture='proposal_tcn')
    assert torch.equal(mu,loaded(x)[0])
    with pytest.raises(ValueError,match='mismatch'):
        learning.load(tmp_path/'tcn.pt',expected_architecture='legacy_cnn')


def test_inputs_and_trailing_window():
    x=np.ones((120,6));att=np.tile([.2,-.3],(120,1));bias=np.arange(6)*.1
    feat=learning.proposal_inputs(x,att,bias)
    assert np.allclose(feat[:,:6],x-bias)
    assert np.array_equal(feat[:,6:],att)
    net=learning.MotionNetTCN()
    before=learning.predict(net,x,np.array([100]),attitude=att,bias=bias)
    x[100:]=1e6;att[100:]=1e6
    after=learning.predict(net,x,np.array([100]),attitude=att,bias=bias)
    assert all(np.array_equal(a,b) for a,b in zip(before,after))
    with pytest.raises(ValueError,match='roll/pitch'):
        learning.predict(net,x,np.array([100]))
    with pytest.raises(ValueError,match='attitude_list'):
        learning.train_motionnet([x],[x[:,:3]],epochs=1,architecture='proposal_tcn')


def test_historical_checkpoints_load():
    paths=[Path('results/motionnet.pt'),Path('results/kitti/motionnet_kitti.pt'),
           *[Path(f'results/i2nav/motionnet_fold{f}.pt') for f in 'AB']]
    for path in paths:
        net=learning.load(path,expected_architecture='legacy_cnn')
        assert type(net) is learning.MotionNet
        assert net(torch.zeros(1,100,6))[0].shape == (1,3)


def test_pipeline_wires_filter_attitude_and_current_bias(monkeypatch):
    sc=make_scenario(12,duration=2)
    net=learning.MotionNetTCN()
    seen=[]
    orig=learning.predict
    def spy(model,imu6,idx,**kwargs):
        seen.append((imu6.copy(),kwargs['attitude'].copy(),kwargs['bias'].copy()))
        return orig(model,imu6,idx,**kwargs)
    monkeypatch.setattr(learning,'predict',spy)
    log=run(sc.truth,sc.imu,sc.gnss,sc.imu_spec,sc.gnss_spec,
            Config(pseudo='hybrid',aci=True,monitor='dual'),model=net)
    assert seen and np.isfinite(log.p_est).all()
    # Every A inference has a separate guard inference; GNSS updates make their
    # input histories differ. Reusing A-conditioned predictions would leak GNSS.
    assert len(seen) % 2 == 0
    assert any(not np.array_equal(seen[i][1],seen[i+1][1])
               for i in range(0,len(seen),2))
    assert seen[0][1].shape == (100,2)
    # Initialization contains attitude error, proving it is not truth conditioning.
    assert not np.allclose(seen[0][1],learning.roll_pitch(sc.truth.R[:100]))
    assert seen[0][2].shape == (6,)
    assert np.array_equal(seen[0][0],sc.imu6()[:100])


def test_cli_tcn_primary_path(tmp_path):
    import yaml,json
    model=tmp_path/'tcn.pt';learning.save(learning.MotionNetTCN(),model)
    cfg={'scenario':{'duration':2},'filter':{'pseudo':'hybrid','aci':True,
         'monitor':'dual','robust_gnss_delta':1.5},'metrics':{'t_start':0},
         'learning':{'model':str(model),'architecture':'proposal_tcn'}}
    path=tmp_path/'config.yaml';path.write_text(yaml.safe_dump(cfg))
    assert main(['run','--config',str(path),'--out',str(tmp_path/'out'),'--architecture','proposal_tcn','--quiet']) == 0
    assert np.isfinite(json.loads((tmp_path/'out/metrics.json').read_text())['metrics']['rmse_h'])
    cfg['learning']['architecture']='legacy_cnn';path.write_text(yaml.safe_dump(cfg))
    assert main(['run','--config',str(path),'--out',str(tmp_path/'bad'),'--quiet']) == 2


def test_tcn_training_normalizes_attitude_and_imu():
    sc=make_scenario(100,duration=3)
    att=learning.roll_pitch(sc.truth.R)
    bias=np.c_[sc.imu.gyro_bias,sc.imu.accel_bias]
    net=learning.train_motionnet([sc.imu6()],[sc.truth.v_body],epochs=1,batch=8,
        stride=20,architecture='proposal_tcn',attitude_list=[att],bias_list=[bias],log=lambda _:None)
    assert isinstance(net,learning.MotionNetTCN)
    idx=np.arange(learning.WINDOW,len(sc.truth.t),20)
    X=learning.make_windows(learning.proposal_inputs(sc.imu6(),att,bias),idx)
    assert np.allclose(net.x_mean.numpy(),X.reshape(-1,8).mean(0),atol=1e-5)
    assert np.allclose(net.x_std.numpy(),X.reshape(-1,8).std(0,ddof=1)+1e-6,rtol=1e-5)
    mu,var=learning.predict(net,sc.imu6(),idx,attitude=att,bias=bias)
    assert np.isfinite(mu).all() and np.all(var>0)


def test_roll_pitch_zyx_yaw_invariance():
    from ccgins.geometry import euler_to_rot
    for roll, pitch in [(0.,0.),(.3,-.4),(-.7,.6)]:
        rotations=np.stack([euler_to_rot(roll,pitch,yaw) for yaw in [-2.,0.,2.]])
        assert np.allclose(learning.roll_pitch(rotations),[roll,pitch],atol=1e-12)


def test_time_varying_bias_and_window_label_alignment():
    raw=np.arange(120*6).reshape(120,6).astype(float)
    bias=raw*.01; att=np.zeros((120,2))
    features=learning.proposal_inputs(raw,att,bias)
    assert np.array_equal(features[:,:6],raw-bias)
    idx=np.array([100,110])
    windows=learning.make_windows(features,idx)
    # Exclusive endpoint means last IMU/attitude and supervised label are idx-1.
    labels=np.arange(120)
    assert np.array_equal(windows[:,-1,0],features[idx-1,0])
    assert np.array_equal(labels[idx-1],[99,109])


def test_channel_normalization_unit_invariance():
    rng=np.random.default_rng(3)
    x=torch.tensor(rng.normal(size=(10,100,8)),dtype=torch.float64)
    scale=torch.tensor([.01,.02,.03,1.,2.,3.,.001,.002])
    shift=torch.arange(8,dtype=torch.float64)
    y=x*scale+shift
    # Independent channel standardization removes SI/radian scale differences.
    zx=(x-x.reshape(-1,8).mean(0))/x.reshape(-1,8).std(0)
    zy=(y-y.reshape(-1,8).mean(0))/y.reshape(-1,8).std(0)
    assert torch.allclose(zx,zy,atol=1e-10)
    assert torch.allclose(zx.reshape(-1,8).std(0),torch.ones(8,dtype=zx.dtype),atol=1e-6)


def test_beta_nll_toy_value_and_detached_weight_gradient():
    mu=torch.tensor([[0.]],requires_grad=True)
    lv=torch.tensor([[np.log(4.)]],requires_grad=True)
    loss=learning.beta_nll(mu,lv,torch.tensor([[2.]]))
    assert loss.item()==pytest.approx(np.log(4.)+1)
    loss.backward()
    assert mu.grad.item()==pytest.approx(-1.)
    assert lv.grad.item()==pytest.approx(0.)
    # The omitted Gaussian constant changes reporting, not gradients.


@pytest.mark.parametrize('arch',['legacy_cnn','proposal_tcn'])
def test_smooth_variance_restores_saturated_head_gradient_and_roundtrip(tmp_path,arch):
    ctor=learning.MotionNetTCN if arch=='proposal_tcn' else learning.MotionNet
    x=torch.zeros(1,100,8 if arch=='proposal_tcn' else 6)
    grads={}
    for activation in ['hard','smooth']:
        net=ctor(variance_activation=activation)
        with torch.no_grad():
            net.head[-1].weight.zero_()
            net.head[-1].bias.copy_(torch.tensor([0.,0.,0.,-13.,5.,0.]))
        mu,lv=net(x)
        learning.beta_nll(mu,lv,torch.ones_like(mu)).backward()
        grads[activation]=net.head[-1].bias.grad[3:5]
        if activation=='smooth':
            learning.save(net,tmp_path/'net.pt')
            loaded=learning.load(tmp_path/'net.pt')
            assert loaded.variance_activation=='smooth'
            assert torch.equal(lv,loaded(x)[1])
    assert torch.equal(grads['hard'],torch.zeros(2))
    assert torch.all(grads['smooth'].abs()>0)
