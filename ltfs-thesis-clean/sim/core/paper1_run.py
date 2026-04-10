"""
paper1_run.py

Configuration-driven runner for Paper 1 scenarios (S0-S3) and ablations (A0-A8).
This script does not change controller formulas; it only standardizes run selection
and command patterns for reproducibility.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path


CORE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = CORE_DIR.parents[1]
RAW_OUTPUT_DIR = PROJECT_ROOT / "outputs" / "raw"
RUN_MATRIX_JSON = PROJECT_ROOT / "notes" / "paper1_run_matrix.json"

BASELINE_SCRIPT = CORE_DIR / "mp_single_baseline_wt.py"
LTFS_SCRIPT = CORE_DIR / "mp_ltfs1.0.1_wt.py"
FREEFLOW_SCRIPT = CORE_DIR / "mp_freeflow.py"
METRICS_SCRIPT = CORE_DIR / "metrics_wt_all.py"


def _load_matrix() -> dict:
    with open(RUN_MATRIX_JSON, "r", encoding="utf-8") as f:
        return json.load(f)


def _select_run(cfg: dict, run_id: str, run_kind: str) -> dict:
    key = "scenarios" if run_kind == "scenario" else "ablations"
    for item in cfg.get(key, []):
        if item.get("id") == run_id:
            return item
    raise ValueError(f"{run_kind} '{run_id}' not found in {RUN_MATRIX_JSON}")


def _run_cmd(cmd: list[str], env: dict, cwd: Path) -> None:
    print(f"[paper1_run] Running: {' '.join(cmd)}")
    subprocess.run(cmd, check=True, env=env, cwd=str(cwd))


def _ensure_required_outputs(paths: list[str]) -> None:
    missing = []
    for p in paths:
        abspath = PROJECT_ROOT / p
        if not abspath.exists():
            missing.append(str(abspath))
    if missing:
        raise FileNotFoundError("Missing required outputs:\n" + "\n".join(missing))


def _archive_outputs(run_id: str) -> Path:
    ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = RAW_OUTPUT_DIR / "paper1_runs" / f"{run_id}_{ts}"
    run_dir.mkdir(parents=True, exist_ok=True)

    files = [
        RAW_OUTPUT_DIR / "baseline_wt_trips.csv",
        RAW_OUTPUT_DIR / "mp_ltfs1.0.1_wt_trips.csv",
        RAW_OUTPUT_DIR / "ltfs_elevated_usage_debug.csv",
    ]
    for f in files:
        if f.exists():
            shutil.copy2(f, run_dir / f.name)
    return run_dir


def main() -> None:
    parser = argparse.ArgumentParser(description="Run Paper 1 scenario or ablation by ID.")
    g = parser.add_mutually_exclusive_group(required=True)
    g.add_argument("--scenario", help="Scenario ID (S0-S3)")
    g.add_argument("--ablation", help="Ablation ID (A0-A8)")
    parser.add_argument(
        "--with-freeflow",
        action="store_true",
        help="Regenerate policy-conditioned free-flow before baseline/LTFS/metrics.",
    )
    parser.add_argument(
        "--skip-metrics",
        action="store_true",
        help="Run controllers only and skip metrics reporting.",
    )
    args = parser.parse_args()

    matrix = _load_matrix()
    run_kind = "scenario" if args.scenario else "ablation"
    run_id = args.scenario if args.scenario else args.ablation
    run_cfg = _select_run(matrix, run_id, run_kind)

    env = os.environ.copy()
    env.update({str(k): str(v) for k, v in run_cfg.get("env", {}).items()})

    print(f"[paper1_run] Selected {run_kind}: {run_id}")
    print(f"[paper1_run] Description: {run_cfg.get('description', '')}")
    print(f"[paper1_run] Env overrides: {run_cfg.get('env', {})}")

    if args.with_freeflow:
        _run_cmd([sys.executable, str(FREEFLOW_SCRIPT)], env, CORE_DIR)

    _run_cmd([sys.executable, str(BASELINE_SCRIPT)], env, CORE_DIR)
    _run_cmd([sys.executable, str(LTFS_SCRIPT)], env, CORE_DIR)

    if not args.skip_metrics:
        _run_cmd([sys.executable, str(METRICS_SCRIPT)], env, CORE_DIR)

    required = run_cfg.get(
        "expected_outputs",
        [
            "outputs/raw/baseline_wt_trips.csv",
            "outputs/raw/mp_ltfs1.0.1_wt_trips.csv",
            "outputs/raw/ltfs_elevated_usage_debug.csv",
        ],
    )
    _ensure_required_outputs(required)
    out_dir = _archive_outputs(run_id)
    print(f"[paper1_run] Archived run outputs: {out_dir}")


if __name__ == "__main__":
    main()
