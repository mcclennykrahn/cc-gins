"""Smoke tests for the configuration-driven CLI."""
from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from ccgins.cli import CliError, _build_inputs, _extend_metrics, _filter_config, load_config

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "configs" / "example_sim.yaml"


@pytest.mark.parametrize("vmax", [1.0, 4.0])
def test_simulated_forward_speed_respects_low_scenario_ceiling(vmax):
    truth, *_ = _build_inputs({"scenario": {"seed": 900, "duration": 60, "vmax": vmax}})
    speed = truth.v_body[:, 0]
    assert np.all(speed >= 0.0)
    assert speed.max() == pytest.approx(vmax)
    assert np.all(speed <= vmax + 1e-12)


def _smoke_config(path: Path) -> Path:
    raw = {
        "scenario": {"seed": 3, "duration": 30.0, "dt": 0.01, "vmax": 15.0},
        "gnss": {
            "rate_hz": 1.0,
            "sigma_h": 1.0,
            "sigma_v": 1.5,
            "outages": [{"start": 12.0, "duration": 6.0}],
        },
        "filter": {"name": "inekf", "pseudo": "nhc", "monitor": "none"},
        "metrics": {"t_start": 5.0},
    }
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    return path


def test_cli_run_writes_report_and_metrics(tmp_path):
    config = _smoke_config(tmp_path / "smoke.yaml")
    out = tmp_path / "out"
    env = os.environ.copy()
    env["PYTHONPATH"] = str(ROOT) + os.pathsep + env.get("PYTHONPATH", "")
    proc = subprocess.run(
        [sys.executable, "-m", "ccgins.cli", "run", "--config", str(config), "--out", str(out)],
        cwd=ROOT, env=env, capture_output=True, text=True, timeout=120,
    )
    assert proc.returncode == 0, proc.stderr
    assert "wrote" in proc.stdout

    report_path, metrics_path = out / "report.md", out / "metrics.json"
    assert report_path.is_file() and metrics_path.is_file()

    report = report_path.read_text(encoding="utf-8")
    assert "Horizontal RMSE" in report
    assert "Maximum horizontal error" in report
    assert "ANEES" in report
    assert "Runtime per IMU step" in report

    payload = json.loads(metrics_path.read_text(encoding="utf-8"))
    metrics = payload["metrics"]
    assert payload["scenario"]["outages"] == [[12.0, 6.0]]
    assert payload["filter"]["pseudo"] == "nhc"
    assert metrics["rmse_h"] >= 0.0
    assert metrics["max_err_h"] >= metrics["rmse_h"]
    assert 0.0 <= metrics["coverage95_h"] <= 1.0
    assert metrics["runtime_us"] > 0.0
    assert len(metrics["outage_metrics"]) == 1
    assert metrics["outage_metrics"][0]["start_s"] == 12.0


def test_example_config_parses_and_builds_inputs():
    raw = load_config(CONFIG)
    cfg = _filter_config(raw)
    assert cfg.filter == "inekf"
    assert cfg.pseudo == "nhc"
    truth, imu, gnss, _, gnss_spec = _build_inputs(raw, duration_override=15.0)
    assert len(truth.t) == len(imu.t)
    assert truth.dt == 0.01
    assert gnss_spec.outages == [(60.0, 30.0), (120.0, 60.0)]
    assert len(gnss.t) >= 1


def test_outage_details_use_same_scoring_window_as_summary():
    t = np.arange(1., 11.)
    error = np.zeros((len(t), 3))
    error[:5, 0] = [100., 50., 2., 3., 200.]
    log = SimpleNamespace(t=t, p_est=error, p_true=np.zeros_like(error),
                          P_pos=np.repeat(np.eye(3)[None], len(t), axis=0),
                          runtime_per_step_us=0.)
    metrics = _extend_metrics(log, [(1., 4.), (1., 1.)], t_start=3.)
    assert metrics["max_err_outage_4s"] == 3.
    assert metrics["max_err_outage_1s"] is None
    assert len(metrics["outage_metrics"]) == 1
    outage = metrics["outage_metrics"][0]
    assert outage["max_err_h"] == outage["end_err_h"] == 3.
    assert outage["anees_pos"] == pytest.approx((4. + 9.) / 6.)
    assert outage["coverage95_h"] == .5
    with pytest.raises(CliError, match="after the last logged epoch"):
        _extend_metrics(log, [], t_start=11.)
