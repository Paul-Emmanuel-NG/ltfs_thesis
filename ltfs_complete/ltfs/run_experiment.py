"""
run_experiment.py — Batch driver for the Paper 1 result set.

Runs free-flow calibrations + the variant sweep + metrics computation in one go.
Designed so you can scope the work up or down from the command line based on
compute budget.

Usage:
    # Tightest defensible story (A0, A2, A8 × S1 × 3 seeds + freeflows)
    python -m ltfs.run_experiment --preset tight

    # Full ablation (A0..A8 × S1 × 3 seeds + freeflows)
    python -m ltfs.run_experiment --preset full

    # Full ablation + scenarios (A0..A8 × S1,S2,S3 × 3 seeds)
    python -m ltfs.run_experiment --preset all

    # Custom:
    python -m ltfs.run_experiment --variants A0 A2 A8 --scenarios S1 --seeds 1 2 --sim-time 3600

Call this AFTER you've done a pipeline sanity test with:
    python -m ltfs.runner --variant A0 --scenario S1 --seed 1 --sim-time 1800
"""

from __future__ import annotations
import argparse
import subprocess
import sys
import time
from pathlib import Path
from typing import List

from . import config as C
from .variants.ablation import ABLATION_SUITE


PRESETS = {
    "tight": {
        "variants": ["A0", "A2", "A8"],
        "scenarios": ["S1"],
        "seeds": [1, 2, 3],
    },
    "full": {
        "variants": ["A0", "A1", "A2", "A3", "A4", "A5", "A6", "A7", "A8"],
        "scenarios": ["S1"],
        "seeds": [1, 2, 3],
    },
    "all": {
        "variants": ["A0", "A1", "A2", "A3", "A4", "A5", "A6", "A7", "A8"],
        "scenarios": ["S1", "S2", "S3"],
        "seeds": [1, 2, 3],
    },
}


def _variant_needs_freeflow(variant_id: str) -> tuple[str, str]:
    """Map variant -> (signal_controller, gate_rule) for freeflow grouping."""
    v = ABLATION_SUITE[variant_id]
    return (v.signal_controller, v.gate_rule if v.uses_gating() else ("none" if v.gate_rule == "none" else "open"))


def required_freeflows(variants: List[str]) -> List[tuple[str, str]]:
    seen = set()
    pairs = []
    for v in variants:
        pair = _variant_needs_freeflow(v)
        if pair not in seen:
            seen.add(pair)
            pairs.append(pair)
    return pairs


def _run_cmd(cmd: List[str]) -> int:
    print(f"\n[batch] $ {' '.join(cmd)}")
    t0 = time.time()
    res = subprocess.run(cmd)
    dt = time.time() - t0
    print(f"[batch] done in {dt/60:.1f} min (exit={res.returncode})")
    return res.returncode

def _extra_flags(args) -> list:
    flags = []
    if args.sumo_cfg:
        flags += ["--sumo-cfg", args.sumo_cfg]
    if args.signal_override:
        flags += ["--signal-override", args.signal_override]
    return flags

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--preset", choices=list(PRESETS.keys()), default=None)
    ap.add_argument("--variants", nargs="+", default=None)
    ap.add_argument("--scenarios", nargs="+", default=None)
    ap.add_argument("--seeds", nargs="+", type=int, default=None)
    ap.add_argument("--sim-time", type=float, default=C.MAX_SIM_TIME_S)
    ap.add_argument("--skip-freeflow", action="store_true", help="Skip free-flow calibration (reuse existing)")
    ap.add_argument("--dry-run", action="store_true", help="Print commands without executing")
    ap.add_argument("--sumo-cfg", default=None, help="Override SUMO config path")
    ap.add_argument("--signal-override", default=None, help="Override signal controller (fixed|mp|pwmp|actuated)")
    args = ap.parse_args()

    if args.preset:
        cfg = PRESETS[args.preset]
        variants = cfg["variants"]
        scenarios = cfg["scenarios"]
        seeds = cfg["seeds"]
    else:
        variants = args.variants or ["A0"]
        scenarios = args.scenarios or ["S1"]
        seeds = args.seeds or [1]

    print(f"[batch] variants={variants}  scenarios={scenarios}  seeds={seeds}  sim_time={args.sim_time:.0f}")
    print(f"[batch] expected runs: {len(variants)}×{len(scenarios)}×{len(seeds)} = "
          f"{len(variants)*len(scenarios)*len(seeds)}")

    PY = sys.executable

    # 1) Free-flow calibrations — one per (signal, gate) pair needed by selected variants
    if not args.skip_freeflow:
        for sig, gate in required_freeflows(variants):
            # Use the first variant in the suite matching this pair as the template
            template = next(v for v in variants if _variant_needs_freeflow(v) == (sig, gate))
            cmd = [
                PY, "-m", "ltfs.runner",
                "--variant", template,
                "--scenario", "S1",
                "--seed", "1",
                "--freeflow",
                "--sim-time", str(args.sim_time),
                "--quiet",
            ] + _extra_flags(args)
            if args.dry_run:
                print(f"[dry-run] {' '.join(cmd)}")
            else:
                _run_cmd(cmd)

    # 2) Variant runs
    for variant in variants:
        for scenario in scenarios:
            for seed in seeds:
                cmd = [
                    PY, "-m", "ltfs.runner",
                    "--variant", variant,
                    "--scenario", scenario,
                    "--seed", str(seed),
                    "--sim-time", str(args.sim_time),
                    "--quiet",
                ] + _extra_flags(args)
                if args.dry_run:
                    print(f"[dry-run] {' '.join(cmd)}")
                else:
                    _run_cmd(cmd)

    print("\n[batch] all runs complete.")
    print("[batch] run `python -m ltfs.report` to aggregate metrics.")


if __name__ == "__main__":
    main()
