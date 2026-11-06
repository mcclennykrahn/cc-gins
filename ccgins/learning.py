"""MotionNet: learned body-frame velocity pseudo-measurement with heteroscedastic variance."""
from __future__ import annotations

import numpy as np
import torch
from torch import nn

WINDOW = 100  # 1 s @ 100 Hz


class MotionNet(nn.Module):
    def __init__(self, ch: int = 6, hidden: int = 64, variance_activation: str = "hard"):
        if variance_activation not in ("hard", "smooth"):
            raise ValueError("unknown variance activation")
        super().__init__()
        self.variance_activation = variance_activation
        self.conv = nn.Sequential(
            nn.Conv1d(ch, 32, 5, padding=2), nn.GELU(),
            nn.Conv1d(32, hidden, 5, stride=2, padding=2), nn.GELU(),
            nn.Conv1d(hidden, hidden, 5, stride=2, padding=2), nn.GELU(),
        )
        self.head = nn.Sequential(nn.Linear(hidden + 2 * ch, 64), nn.GELU(), nn.Linear(64, 6))
        self.register_buffer("x_mean", torch.zeros(ch))
        self.register_buffer("x_std", torch.ones(ch))

    def forward(self, x):  # x: (B, T, 6) raw IMU
        xn = (x - self.x_mean) / self.x_std
        feat = self.conv(xn.transpose(1, 2)).mean(dim=2)
        stats = torch.cat([xn.mean(dim=1), xn.std(dim=1)], dim=1)
        out = self.head(torch.cat([feat, stats], dim=1))
        mu, raw = out[:, :3], out[:, 3:]
        # Smooth clipping retains a restoring gradient outside the bounds.
        # Historical checkpoints retain hard clipping and identical predictions.
        logvar = (raw.clamp(-12.0, 4.0) if self.variance_activation == "hard" else
                  -12.0 + torch.nn.functional.softplus(raw + 12.0)
                  - torch.nn.functional.softplus(raw - 4.0))
        return mu, logvar


class MotionNetTCN(MotionNet):
    """Proposal §3.4: three same-padded temporal layers, dilations 1/2/4.

    Eight channels = bias-corrected gyro/accel + roll/pitch radians. Symmetric
    padding is confined to the supplied trailing window; no future samples are
    consumed. Internal features are not claimed causal at each window position.
    Normalization/statistical pooling and dual heads reuse the legacy interface.
    """
    def __init__(self, variance_activation: str = "hard"):
        super().__init__(ch=8, variance_activation=variance_activation)
        self.conv = nn.Sequential(
            nn.Conv1d(8, 32, 5, padding=2, dilation=1), nn.GELU(),
            nn.Conv1d(32, 64, 5, padding=4, dilation=2), nn.GELU(),
            nn.Conv1d(64, 64, 5, padding=8, dilation=4), nn.GELU(),
        )


def roll_pitch(R):
    """Body→ENU rotation(s) to roll/pitch radians (ZYX convention)."""
    R = np.asarray(R)
    return np.stack([np.arctan2(R[..., 2, 1], R[..., 2, 2]),
                     np.arcsin(np.clip(-R[..., 2, 0], -1, 1))], axis=-1)


def proposal_inputs(imu6, attitude, bias=None):
    """Explicit attitude (N,2) and optional gyro/accel bias (6,) or (N,6).

    Offline supervised training may use reference attitude/bias; deployment must
    supply filter estimates. Never silently synthesize missing attitude.
    """
    imu6, attitude = np.asarray(imu6), np.asarray(attitude)
    if imu6.ndim != 2 or imu6.shape[1] != 6 or attitude.shape != (len(imu6), 2):
        raise ValueError("proposal_tcn requires IMU (N,6) and roll/pitch (N,2)")
    if bias is not None and np.asarray(bias).shape not in ((6,), imu6.shape):
        raise ValueError("bias must be (6,) or (N,6)")
    x = np.concatenate([imu6 if bias is None else imu6 - bias, attitude], axis=1)
    if not np.isfinite(x).all():
        raise ValueError("non-finite learning inputs")
    return x


def architecture(net):
    return "proposal_tcn" if isinstance(net, MotionNetTCN) else "legacy_cnn"


def make_windows(imu6: np.ndarray, idx: np.ndarray) -> np.ndarray:
    """imu6: (N,6) [gyro, accel]; idx: end indices (exclusive) -> (M, WINDOW, 6)."""
    off = np.arange(-WINDOW, 0)
    return imu6[idx[:, None] + off[None, :]]


def beta_nll(mu, logvar, y, beta: float = 0.5):
    var = logvar.exp()
    nll = 0.5 * (logvar + (y - mu) ** 2 / var)
    return (nll * var.detach() ** beta).mean()


def train_motionnet(imu_list, vb_list, epochs: int = 6, batch: int = 512, lr: float = 2e-3,
                    stride: int = 10, seed: int = 0, log=print, init: "MotionNet | None" = None,
                    architecture: str | None = None, attitude_list=None, bias_list=None,
                    variance_activation: str = "hard"):
    """Train from scratch, or fine-tune `init`, retaining its normalization/variance activation."""
    torch.manual_seed(seed)
    np.random.seed(seed)
    # Omitted architecture preserves the historical six-channel Python API.
    arch = architecture or ("proposal_tcn" if attitude_list is not None else "legacy_cnn")
    if arch not in ("legacy_cnn", "proposal_tcn"):
        raise ValueError("unknown architecture")
    if init is not None and ("proposal_tcn" if isinstance(init, MotionNetTCN) else "legacy_cnn") != arch:
        raise ValueError("initial checkpoint architecture mismatch")
    if len(imu_list) != len(vb_list) or not imu_list:
        raise ValueError("nonempty paired training sequences required")
    if arch == "proposal_tcn":
        if attitude_list is None or len(attitude_list) != len(imu_list):
            raise ValueError("proposal_tcn requires attitude_list")
        if bias_list is not None and len(bias_list) != len(imu_list):
            raise ValueError("bias_list length mismatch")
        imu_list = [proposal_inputs(x, attitude_list[i], None if bias_list is None else bias_list[i])
                    for i, x in enumerate(imu_list)]
    X, Y = [], []
    for imu6, vb in zip(imu_list, vb_list):
        idx = np.arange(WINDOW, len(imu6), stride)
        X.append(make_windows(imu6, idx).astype(np.float32))
        Y.append(vb[idx - 1].astype(np.float32))
    X = torch.from_numpy(np.concatenate(X))
    Y = torch.from_numpy(np.concatenate(Y))
    if init is not None:
        import copy
        net = copy.deepcopy(init)
    else:
        net = (MotionNetTCN(variance_activation=variance_activation) if arch == "proposal_tcn"
               else MotionNet(variance_activation=variance_activation))
        flat = X.reshape(-1, X.shape[-1])
        net.x_mean.copy_(flat.mean(0))
        net.x_std.copy_(flat.std(0) + 1e-6)
    opt = torch.optim.Adam(net.parameters(), lr=lr)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=epochs)
    n = len(X)
    for ep in range(epochs):
        perm = torch.randperm(n)
        tot = 0.0
        net.train()
        for i in range(0, n, batch):
            b = perm[i:i + batch]
            mu, lv = net(X[b])
            loss = beta_nll(mu, lv, Y[b])
            opt.zero_grad()
            loss.backward()
            opt.step()
            tot += loss.item() * len(b)
        sched.step()
        log(f"epoch {ep + 1}/{epochs} beta-NLL {tot / n:.4f}")
    return net


@torch.no_grad()
def predict(net: MotionNet, imu6: np.ndarray, idx: np.ndarray, batch: int = 4096,
            attitude=None, bias=None):
    net.eval()
    if isinstance(net, MotionNetTCN):
        imu6 = proposal_inputs(imu6, attitude, bias)
    mus, vs = [], []
    for i in range(0, len(idx), batch):
        x = torch.from_numpy(make_windows(imu6, idx[i:i + batch]).astype(np.float32))
        mu, lv = net(x)
        mus.append(mu.numpy())
        vs.append(lv.exp().numpy())
    return np.concatenate(mus).astype(float), np.concatenate(vs).astype(float)


def save(net, path):
    if isinstance(net, MotionNetTCN) or net.variance_activation != "hard":
        torch.save(dict(architecture=architecture(net), variance_activation=net.variance_activation,
                        state_dict=net.state_dict()), path)
    else:
        torch.save(net.state_dict(), path)


def load(path, expected_architecture=None):
    payload = torch.load(path, map_location="cpu", weights_only=True)
    arch = payload.get("architecture", "legacy_cnn")
    if arch not in ("legacy_cnn", "proposal_tcn"):
        raise ValueError("unknown checkpoint architecture")
    if expected_architecture is not None and arch != expected_architecture:
        raise ValueError("checkpoint architecture mismatch")
    activation = payload.get("variance_activation", "hard")
    net = (MotionNetTCN(variance_activation=activation) if arch == "proposal_tcn"
           else MotionNet(variance_activation=activation))
    net.load_state_dict(payload["state_dict"] if "state_dict" in payload else payload)
    net.eval()
    return net


def correlation_factor(z: np.ndarray, max_lag: int = 100) -> np.ndarray:
    """Effective-sample-size inflation n_eff = 1 + 2*sum_k rho_k of normalized errors z (T, C).

    Learned pseudo-measurements at 10 Hz have strongly autocorrelated errors; a Kalman filter that
    treats them as white becomes over-confident by roughly this factor. Sum is truncated at the
    first non-positive autocorrelation (initial positive sequence) or max_lag.
    """
    z = z - z.mean(0)
    out = []
    for c in range(z.shape[1]):
        x = z[:, c]
        v = x.var()
        tot = 1.0
        for L in range(1, max_lag + 1):
            r = float(np.mean(x[:-L] * x[L:]) / v)
            if r <= 0:
                break
            tot += 2 * r
        out.append(tot)
    return np.array(out)


def grouped_correlation_factor(groups, max_lag=100):
    """Equal-drive mean of within-drive correlation inflations (legacy n_eff).

    Never concatenate independent drives: center each drive independently, and
    never create cross-boundary lag pairs. Effective independent count is N/factor;
    this factor itself is NOT a sample count. Constant channels have factor one.
    """
    groups = [np.asarray(g, float) for g in groups]
    if not groups or any(g.ndim != 2 or len(g) < 2 or not np.isfinite(g).all() for g in groups):
        raise ValueError('finite groups of at least two samples required')
    if len({g.shape[1] for g in groups}) != 1:
        raise ValueError('same channels required')
    factors = []
    for g in groups:
        active = g.var(0) > 0
        f = np.ones(g.shape[1])
        f[active] = correlation_factor(g[:, active], min(max_lag, len(g)-1))
        factors.append(f)
    return np.mean(factors, axis=0)
