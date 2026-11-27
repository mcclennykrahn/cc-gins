"""Hold the state and covariance at 120 s fixed while changing Q after entry."""
import json
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

import numpy as np

from ccgins.cli import _build_inputs, _filter_config, load_config
from ccgins.filters import NavFilter, NoiseParams
from ccgins.pipeline import run
from scripts.audit_scheme import budget

out = Path(__file__).parent
raw = load_config("configs/example_sim.yaml")
start, duration = raw["gnss"]["outages"][-1].values()
rows = []

for seed in (900, 93001, 93002, 93003, 93004):
    truth, imu, gnss, imu_spec, gnss_spec = _build_inputs(raw, seed_override=seed)
    entry_index = round(start / imu.dt)
    for alignment in (0, 1701):
        config = replace(_filter_config(raw), init_seed=alignment)
        assert config.filter == "inekf" and config.monitor == "none"
        reference_entry = None
        for mode in ("fixed", "oracle", "prior_state"):
            original = NavFilter.propagate
            step = 0
            entry = None

            def propagate(filt, gyro, accel, dt):
                global step, entry
                if step == entry_index:
                    entry = tuple(value.copy() for value in
                                  (filt.R, filt.v, filt.p, filt.bg, filt.ba, filt.P))
                if step >= entry_index and mode != "fixed":
                    body_speed = (truth.v_body[step, 0] if mode == "oracle"
                                  else (filt.R.T @ filt.v)[0])
                    filt.noise = NoiseParams.from_spec(imu_spec, vmax_eff=abs(body_speed), dt=dt)
                step += 1
                return original(filt, gyro, accel, dt)

            with patch.object(NavFilter, "propagate", propagate):
                log = run(truth, imu, gnss, imu_spec, gnss_spec, config)
            assert step == len(truth.t) - 1 and entry is not None
            error = log.p_est - log.p_true
            if mode == "fixed":
                reference_entry = entry
                reference_error = error
            else:
                for expected, actual in zip(reference_entry, entry, strict=True):
                    np.testing.assert_array_equal(expected, actual)
                np.testing.assert_array_equal(reference_error[log.t <= start], error[log.t <= start])
            rows.append(dict(seed=seed, alignment=alignment, mode=mode,
                             entry_state_and_covariance_identical=True,
                             segments=budget(log.t, error, gnss_spec.outages),
                             bins=[dict(start=s, **budget(
                                 log.t[(log.t >= s) & (log.t < s + 10)],
                                 error[(log.t >= s) & (log.t < s + 10)], [], s)["overall"])
                                 for s in range(30, 180, 10)]))
            (out / "fixed_entry.json").write_text(json.dumps(rows, indent=2, allow_nan=False) + "\n")
