"""MotionNet: learned body-frame velocity pseudo-measurement with heteroscedastic variance."""
from __future__ import annotations

import numpy as np
import torch
from torch import nn

WINDOW = 100  # 1 s @ 100 Hz


class MotionNet(nn.Module):
    def __init__(self, ch: int = 6, hidden: int = 64):
        super().__init__()
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
        mu, logvar = out[:, :3], out[:, 3:].clamp(-12.0, 4.0)
        return mu, logvar


def make_windows(imu6: np.ndarray, idx: np.ndarray) -> np.ndarray:
    """imu6: (N,6) [gyro, accel]; idx: end indices (exclusive) -> (M, WINDOW, 6)."""
    off = np.arange(-WINDOW, 0)
    return imu6[idx[:, None] + off[None, :]]


def beta_nll(mu, logvar, y, beta: float = 0.5):
    var = logvar.exp()
    nll = 0.5 * (logvar + (y - mu) ** 2 / var)
    return (nll * var.detach() ** beta).mean()


def train_motionnet(imu_list, vb_list, epochs: int = 6, batch: int = 512, lr: float = 2e-3,
                    stride: int = 10, seed: int = 0, log=print):
    torch.manual_seed(seed)
    np.random.seed(seed)
    X, Y = [], []
    for imu6, vb in zip(imu_list, vb_list):
        idx = np.arange(WINDOW, len(imu6), stride)
        X.append(make_windows(imu6, idx).astype(np.float32))
        Y.append(vb[idx - 1].astype(np.float32))
    X = torch.from_numpy(np.concatenate(X))
    Y = torch.from_numpy(np.concatenate(Y))
    net = MotionNet()
    flat = X.reshape(-1, 6)
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
def predict(net: MotionNet, imu6: np.ndarray, idx: np.ndarray, batch: int = 4096):
    net.eval()
    mus, vs = [], []
    for i in range(0, len(idx), batch):
        x = torch.from_numpy(make_windows(imu6, idx[i:i + batch]).astype(np.float32))
        mu, lv = net(x)
        mus.append(mu.numpy())
        vs.append(lv.exp().numpy())
    return np.concatenate(mus).astype(float), np.concatenate(vs).astype(float)


def save(net, path):
    torch.save(net.state_dict(), path)


def load(path):
    net = MotionNet()
    net.load_state_dict(torch.load(path, map_location="cpu"))
    net.eval()
    return net
