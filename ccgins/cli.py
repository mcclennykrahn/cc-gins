"""Command-line interface for configuration-driven simulation and outage runs.

Usage::

    python -m ccgins.cli run --config configs/example_sim.yaml --out results/cli_demo

The command writes ``report.md`` (RMSE, maximum error, NEES consistency and runtime) and
``metrics.json`` (all metric values plus the resolved configuration) to ``--out``.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict, fields
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from .metrics import CHI2_2_95, summarize
from .timing import at_or_after, in_window
from .pipeline import Config, run
from .sim import GnssSpec, ImuSpec, generate_trajectory, simulate_gnss, simulate_imu


class CliError(Exception):
    """Invalid configuration or scenario."""


def _as_mapping(value: Any, where: str) -> dict:
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise CliError(f"{where} must be a mapping, got {type(value).__name__}")
    return value


def _unknown_keys(raw: dict, allowed: set[str], where: str) -> None:
    unknown = sorted(set(raw) - allowed)
    if unknown:
        raise CliError(f"unknown {where} option(s): {', '.join(unknown)}")


def load_config(path: str | Path) -> dict:
    """Load a YAML config (``.json`` files are also accepted)."""
    p = Path(path)
    if not p.is_file():
        raise CliError(f"config file not found: {p}")
    text = p.read_text(encoding="utf-8")
    try:
        if p.suffix.lower() == ".json":
            cfg = json.loads(text)
        else:
            cfg = yaml.safe_load(text)
    except (json.JSONDecodeError, yaml.YAMLError) as exc:
        raise CliError(f"cannot parse {p}: {exc}") from exc
    return _as_mapping(cfg, str(p))


def _parse_outages(raw: Any) -> list[tuple[float, float]]:
    if raw is None:
        return []
    if not isinstance(raw, (list, tuple)):
        raise CliError("outages must be a list")
    outages: list[tuple[float, float]] = []
    for i, item in enumerate(raw):
        if isinstance(item, dict):
            if "start" in item and "duration" in item:
                start, duration = item["start"], item["duration"]
            elif "start" in item and "end" in item:
                start, duration = item["start"], float(item["end"]) - float(item["start"])
            else:
                raise CliError(f"outage[{i}] needs start+duration (or start+end)")
        elif isinstance(item, (list, tuple)) and len(item) == 2:
            start, duration = item
        else:
            raise CliError(f"outage[{i}] must be {{start, duration}} or [start, duration]")
        start, duration = float(start), float(duration)
        if start < 0 or duration <= 0:
            raise CliError(f"outage[{i}] must have start >= 0 and duration > 0")
        outages.append((start, duration))
    return outages


def _sim_config(raw: dict) -> dict:
    sim = raw.get("scenario", raw.get("simulation", {}))
    return _as_mapping(sim, "scenario")


def _build_inputs(raw: dict, *, seed_override: int | None = None,
                  duration_override: float | None = None):
    sim = _sim_config(raw)
    _unknown_keys(sim, {"seed", "duration", "dt", "vmax", "outages", "spoof"}, "scenario")
    seed = int(seed_override if seed_override is not None else sim.get("seed", 0))
    duration = float(duration_override if duration_override is not None
                     else sim.get("duration", 120.0))
    dt = float(sim.get("dt", 0.01))
    vmax = float(sim.get("vmax", 20.0))
    if duration <= 0:
        raise CliError("scenario.duration must be positive")
    if dt <= 0:
        raise CliError("scenario.dt must be positive")
    if vmax <= 0:
        raise CliError("scenario.vmax must be positive")

    imu_raw = _as_mapping(raw.get("imu", {}), "imu")
    _unknown_keys(imu_raw, {f.name for f in fields(ImuSpec)}, "imu")
    imu_spec = ImuSpec(**imu_raw)

    gnss_raw = _as_mapping(raw.get("gnss", {}), "gnss")
    _unknown_keys(gnss_raw, {"rate_hz", "sigma_h", "sigma_v", "mp_sigma", "mp_tau",
                             "lever_arm", "outages", "spoof", "spoof_start",
                             "spoof_rate", "spoof_dir_deg"}, "gnss")
    outages = _parse_outages(gnss_raw.get("outages", sim.get("outages", raw.get("outages"))))
    spoof = _as_mapping(gnss_raw.get("spoof", sim.get("spoof", {})), "spoof")
    _unknown_keys(spoof, {"start", "rate", "direction_deg"}, "spoof")
    spoof_start = spoof.get("start", gnss_raw.get("spoof_start"))
    spoof_rate = float(spoof.get("rate", gnss_raw.get("spoof_rate", 0.0)))
    spoof_dir = float(spoof.get("direction_deg", gnss_raw.get("spoof_dir_deg", 45.0)))
    if spoof_rate < 0:
        raise CliError("spoof.rate must be non-negative")

    gnss_kwargs = {k: v for k, v in gnss_raw.items()
                   if k in {"rate_hz", "sigma_h", "sigma_v", "mp_sigma", "mp_tau", "lever_arm"}}
    if "lever_arm" in gnss_kwargs:
        gnss_kwargs["lever_arm"] = np.asarray(gnss_kwargs["lever_arm"], dtype=float)
        if gnss_kwargs["lever_arm"].shape != (3,):
            raise CliError("gnss.lever_arm must contain 3 numbers")
    gnss_spec = GnssSpec(outages=outages,
                         spoof_start=None if spoof_start is None else float(spoof_start),
                         spoof_rate=spoof_rate, spoof_dir_deg=spoof_dir, **gnss_kwargs)

    truth = generate_trajectory(duration, dt=dt, seed=seed, vmax=vmax)
    imu = simulate_imu(truth, imu_spec, seed=seed)
    gnss = simulate_gnss(truth, gnss_spec, seed=seed)
    return truth, imu, gnss, imu_spec, gnss_spec


def _filter_config(raw: dict) -> Config:
    filt = _as_mapping(raw.get("filter", {}), "filter")
    filt = dict(filt)
    if "name" in filt:
        filt["filter"] = filt.pop("name")
    allowed = {f.name for f in fields(Config)}
    _unknown_keys(filt, allowed, "filter")
    if "corr_scale" in filt and filt["corr_scale"] is not None:
        filt["corr_scale"] = tuple(float(x) for x in filt["corr_scale"])
    if "mount_py" in filt and filt["mount_py"] is not None:
        filt["mount_py"] = tuple(float(x) for x in filt["mount_py"])
        if len(filt["mount_py"]) != 2:
            raise CliError("filter.mount_py must be [pitch, yaw]")
    if "p0_bias" in filt and filt["p0_bias"] is not None:
        filt["p0_bias"] = tuple(tuple(float(x) for x in v) for v in filt["p0_bias"])
    cfg = Config(**filt)
    if cfg.filter not in {"eskf", "inekf"}:
        raise CliError(f"filter.filter must be 'eskf' or 'inekf', got {cfg.filter!r}")
    if cfg.pseudo not in {"none", "nhc", "learned_nhc", "learned_vel", "hybrid"}:
        raise CliError(f"unknown filter.pseudo: {cfg.pseudo!r}")
    if cfg.monitor not in {"none", "single", "dual"}:
        raise CliError(f"unknown filter.monitor: {cfg.monitor!r}")
    if cfg.pseudo_every <= 0:
        raise CliError("filter.pseudo_every must be positive")
    return cfg


def _resolve_model(model: str, config_path: Path) -> Path:
    p = Path(model).expanduser()
    if p.is_file():
        return p
    candidate = config_path.parent / p
    if candidate.is_file():
        return candidate
    raise CliError(f"learning model not found: {model}")


def _predictions(raw: dict, truth, imu, cfg: Config, config_path: Path):
    """Run MotionNet for learned pseudo-measurement modes (imports torch lazily)."""
    if cfg.pseudo not in {"learned_nhc", "learned_vel", "hybrid"}:
        return None
    learn_raw = _as_mapping(raw.get("learning", {}), "learning")
    _unknown_keys(learn_raw, {"model", "architecture"}, "learning")
    model = learn_raw.get("model")
    if not model:
        raise CliError(f"filter.pseudo={cfg.pseudo!r} requires learning.model")
    from . import learning

    if len(truth.t) <= learning.WINDOW:
        raise CliError("scenario is shorter than one MotionNet window (100 samples)")
    arch = learn_raw.get("architecture")
    if arch not in (None, "legacy_cnn", "proposal_tcn"):
        raise CliError("learning.architecture must be legacy_cnn or proposal_tcn")
    net = learning.load(_resolve_model(str(model), config_path), expected_architecture=arch)
    if learning.architecture(net) == "proposal_tcn":
        return net  # filter attitude/bias are available inside pipeline.run only
    idx = np.arange(learning.WINDOW, len(truth.t), cfg.pseudo_every)
    imu6 = np.concatenate([imu.gyro, imu.accel], axis=1)
    import torch

    with torch.no_grad():
        mu, var = learning.predict(net, imu6, idx)
    return idx, mu, var


def _extend_metrics(log, outages, t_start: float) -> dict:
    mask = at_or_after(log.t, t_start)
    if not np.any(mask):
        raise CliError(f"metrics.t_start={t_start} is after the last logged epoch")
    metrics = summarize(log, outages, t_start=t_start)
    err = log.p_est - log.p_true
    eh = np.linalg.norm(err[:, :2], axis=1)
    nees_h = np.einsum("ni,nij,nj->n", err[:, :2],
                       np.linalg.inv(log.P_pos[:, :2, :2]), err[:, :2])
    nees_3d = np.einsum("ni,nij,nj->n", err, np.linalg.inv(log.P_pos), err)
    metrics.update({
        "max_err_h": float(eh[mask].max()),
        "max_err_3d": float(np.linalg.norm(err[mask], axis=1).max()),
        "nees_2d_mean": float(nees_h[mask].mean()),
        "nees_3d_mean": float(nees_3d[mask].mean()),
        "nees_2d_p95": float(np.percentile(nees_h[mask], 95)),
        "nees_3d_p95": float(np.percentile(nees_3d[mask], 95)),
        "nees_95_threshold_2d": CHI2_2_95,
        "n_metrics_epochs": int(mask.sum()),
    })
    segs = []
    for start, duration in outages:
        seg = mask & in_window(log.t, start, start + duration)
        if not np.any(seg):
            continue
        segs.append({
            "start_s": float(start),
            "duration_s": float(duration),
            "max_err_h": float(eh[seg].max()),
            "end_err_h": float(eh[seg][-1]),
            "anees_pos": float(nees_3d[seg].mean() / 3.0),
            "coverage95_h": float(np.mean(nees_h[seg] <= CHI2_2_95)),
        })
    metrics["outage_metrics"] = segs
    return metrics


def _write_report(path: Path, metrics: dict, cfg: Config, scenario: dict,
                  config_path: Path, wall_time_s: float, t_start: float) -> None:
    def row(name: str, value: float, fmt: str = ".3f") -> str:
        return f"| {name} | {format(value, fmt)} |"

    outages = scenario["outages"]
    outage_text = ", ".join(f"{s:g}s+{d:g}s" for s, d in outages) if outages else "none"
    lines = [
        "# cc-gins simulation report",
        "",
        f"- Config: `{config_path}`",
        f"- Scenario: seed={scenario['seed']}, duration={scenario['duration']:g}s, "
        f"dt={scenario['dt']:g}s, IMU samples={scenario['n_imu']}, GNSS epochs={scenario['n_gnss']}",
        f"- Outages: {outage_text}",
        f"- Filter: `{cfg.filter}`, pseudo=`{cfg.pseudo}`, monitor=`{cfg.monitor}`, "
        f"ACI={'on' if cfg.aci else 'off'}",
        f"- Metrics window: t >= {t_start:g}s",
        "",
        "## Metrics",
        "",
        "| Metric | Value |",
        "| --- | --- |",
        row("Horizontal RMSE [m]", metrics["rmse_h"]),
        row("3D RMSE [m]", metrics["rmse_3d"]),
        row("Maximum horizontal error [m]", metrics["max_err_h"]),
        row("Maximum 3D error [m]", metrics["max_err_3d"]),
        row("ANEES (position, 3D)", metrics["anees_pos"]),
        row("Mean 2D NEES", metrics["nees_2d_mean"]),
        row("Mean 3D NEES", metrics["nees_3d_mean"]),
        row("2D NEES 95th percentile", metrics["nees_2d_p95"]),
        row("95% NEES coverage (2D)", metrics["coverage95_h"]),
        row("Runtime per IMU step [us]", metrics["runtime_us"], ".1f"),
        row("Total run wall time [s]", wall_time_s),
        "",
    ]
    lines += [
        "## Outages",
        "",
    ]
    if metrics["outage_metrics"]:
        lines += [
            "| Start [s] | Duration [s] | Max horizontal error [m] | End error [m] | ANEES | 95% coverage |",
            "| --- | --- | --- | --- | --- | --- |",
        ]
        for seg in metrics["outage_metrics"]:
            lines.append(
                f"| {seg['start_s']:.1f} | {seg['duration_s']:.1f} | {seg['max_err_h']:.3f} | "
                f"{seg['end_err_h']:.3f} | {seg['anees_pos']:.3f} | {seg['coverage95_h']:.3f} |")
    else:
        lines.append("No configured outages.")
    lines += [
        "",
        "NEES consistency: ANEES is the mean 3D position NEES divided by 3 (target 1.0); "
        f"the 95% coverage uses the 2D threshold chi2_2(0.95) = {CHI2_2_95:.3f}.",
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


def _jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    return value


def run_command(args: argparse.Namespace) -> int:
    config_path = Path(args.config)
    out_dir = Path(args.out)
    try:
        raw = load_config(config_path)
        if getattr(args, "architecture", None) is not None:
            raw["learning"] = {**_as_mapping(raw.get("learning", {}), "learning"),
                               "architecture": args.architecture}
        cfg = _filter_config(raw)
        truth, imu, gnss, imu_spec, gnss_spec = _build_inputs(
            raw, seed_override=args.seed, duration_override=args.duration)
        pred = _predictions(raw, truth, imu, cfg, config_path)
        t_start = float(_as_mapping(raw.get("metrics", {}), "metrics").get("t_start", 60.0))
        if t_start < 0:
            raise CliError("metrics.t_start must be non-negative")

        t0 = time.perf_counter()
        from .learning import MotionNetTCN
        log = run(truth, imu, gnss, imu_spec, gnss_spec, cfg,
                  model=pred if isinstance(pred, MotionNetTCN) else None,
                  pred=None if isinstance(pred, MotionNetTCN) else pred)
        wall_time_s = time.perf_counter() - t0
        metrics = _extend_metrics(log, gnss_spec.outages, t_start)
        metrics["runtime_total_s"] = wall_time_s
        metrics["n_steps"] = int(len(truth.t) - 1)

        sim = _sim_config(raw)
        scenario = {
            "seed": int(args.seed if args.seed is not None else sim.get("seed", 0)),
            "duration": float(truth.t[-1]),
            "dt": float(truth.dt),
            "n_imu": int(len(truth.t)),
            "n_gnss": int(len(gnss.t)),
            "outages": [[float(s), float(d)] for s, d in gnss_spec.outages],
        }

        out_dir.mkdir(parents=True, exist_ok=True)
        report_path = out_dir / "report.md"
        metrics_path = out_dir / "metrics.json"
        _write_report(report_path, metrics, cfg, scenario, config_path, wall_time_s, t_start)
        payload = {
            "schema_version": 1,
            "config_path": str(config_path),
            "scenario": scenario,
            "filter": asdict(cfg),
            "metrics": metrics,
            "wall_time_s": wall_time_s,
        }
        metrics_path.write_text(
            json.dumps(_jsonable(payload), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    except CliError as exc:
        print(f"ccgins: error: {exc}", file=sys.stderr)
        return 2
    except (OSError, ValueError, ImportError) as exc:
        print(f"ccgins: error: {exc}", file=sys.stderr)
        return 2

    if not args.quiet:
        print(f"wrote {report_path}")
        print(f"wrote {metrics_path}")
        print(f"horizontal RMSE {metrics['rmse_h']:.3f} m, "
              f"max error {metrics['max_err_h']:.3f} m, ANEES {metrics['anees_pos']:.3f}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="ccgins", description="Configuration-driven cc-gins simulation and outage runs")
    sub = parser.add_subparsers(dest="command", required=True)
    run_p = sub.add_parser("run", help="run a simulated GNSS/INS scenario from a config file")
    run_p.add_argument("--config", required=True, help="YAML (or JSON) scenario config")
    run_p.add_argument("--out", required=True, help="output directory for report.md and metrics.json")
    run_p.add_argument("--seed", type=int, help="override scenario.seed")
    run_p.add_argument("--duration", type=float, help="override scenario.duration [s]")
    run_p.add_argument("--architecture", choices=["legacy_cnn", "proposal_tcn"],
                       help="require this learning checkpoint architecture (overrides config)")
    run_p.add_argument("--quiet", action="store_true", help="suppress summary output")
    run_p.set_defaults(func=run_command)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
