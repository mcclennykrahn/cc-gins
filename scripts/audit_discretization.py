"""Isolate Euler transition and process-covariance errors in a frozen model.

This is an offline linear-system diagnostic, not a navigation candidate. No truth
is supplied to a filter update. Exactness is relative to constant F and G only.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.linalg import expm

from ccgins.filters import InEKF, NoiseParams
from ccgins.geometry import skew
from ccgins.sim import ImuSpec


def exact_step(F, W, dt):
    """Van Loan integral Q = integral exp(Fs) W exp(Fs).T ds."""
    n = len(F)
    block = np.zeros((2*n, 2*n))
    block[:n, :n] = F
    block[:n, n:] = W
    block[n:, n:] = -F.T
    E = expm(block * dt)
    phi = E[:n, :n]
    q = E[:n, n:] @ phi.T
    return phi, (q + q.T) / 2


def accumulated_covariance(phi, q, steps):
    """Compose identical steps by squaring, starting with zero covariance."""
    total_q = np.zeros_like(q)
    while steps:
        if steps & 1:
            total_q = phi @ total_q @ phi.T + q
        q = phi @ q @ phi.T + q
        phi = phi @ phi
        steps //= 2
    return (total_q + total_q.T) / 2


def experiment():
    rows = []
    # Large p tests cancellation in invariant coordinates, not another motion.
    for position in ([0., 0., 0.], [2000., 1000., 0.]):
        noise = NoiseParams.from_spec(ImuSpec(), dt=.01)
        filt = InEKF(np.eye(3), [20., 0., 0.], position, np.zeros((15,15)), noise,
                     earth={'w_ie': None, 'g': np.array([0., 0., -9.81])})
        F, G = filt._jacobians(np.zeros(3), np.array([0.,0.,9.81]))
        densities = np.repeat([noise.gyro, noise.accel, noise.gyro_bias_rw,
                               noise.accel_bias_rw], 3)
        W = (G * densities**2) @ G.T
        J = np.zeros((3,15))
        J[:, :3] = -skew(filt.p)
        J[:, 6:9] = np.eye(3)
        for duration in (30., 60.):
            # Repeated short exact steps avoid a poorly conditioned long Van Loan exponential.
            ep, eq = exact_step(F, W, .01)
            reference = accumulated_covariance(ep, eq, round(duration/.01))
            ref = J @ reference @ J.T
            for dt in (.02, .01, .005):
                ep, eq = exact_step(F, W, dt)
                for mode, phi, q in (
                    ('euler_phi_euler_q', np.eye(15)+F*dt, W*dt),
                    ('exact_phi_euler_q', ep, W*dt),
                    ('euler_phi_exact_q', np.eye(15)+F*dt, eq),
                    ('exact_phi_exact_q', ep, eq),
                ):
                    P = accumulated_covariance(phi, q, round(duration/dt))
                    physical = J @ P @ J.T
                    rows.append(dict(position=position, duration_s=duration, dt=dt, mode=mode,
                        reference_position_sigma_m=np.sqrt(np.diag(ref)).tolist(),
                        position_sigma_m=np.sqrt(np.maximum(0,np.diag(physical))).tolist(),
                        relative_position_covariance_error=float(np.linalg.norm(physical-ref)/np.linalg.norm(ref)),
                        relative_horizontal_variance_error=float(np.trace(physical[:2,:2])/np.trace(ref[:2,:2])-1)))
    return dict(scope='Frozen F/G, zero initial covariance, process-noise accumulation only; '
                'not trajectory replay, Monte Carlo, NEES attribution, or online before/after.', rows=rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out/'covariance.json').write_text(json.dumps(experiment(), indent=2)+'\n')


if __name__ == '__main__':
    main()
