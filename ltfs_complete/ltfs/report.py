"""
report.py — Aggregate metrics across all completed runs and produce Table II
and Figures 2-5.

Call after run_experiment.py completes. Scans `results/` for trip CSVs,
matches them to the correct free-flow reference, computes metrics, and:

  - Writes results_table.csv (Table II structure from Sec X-E.1)
  - Writes figures/fig2_tt_distribution.png  (Fig 2: CDF across variants)
  - Writes figures/fig3_ablation_bars.png    (Fig 3: mean TT / WT per variant)
  - Writes figures/fig4_gate_occupancy.png   (Fig 4: admissions + occupancy over time)
  - Writes figures/fig5_class_fairness.png   (Fig 5: class-wise bars)

This is a best-effort aggregator. If some runs are missing, it reports what's
present and skips the rest.
"""

from __future__ import annotations
import argparse
import csv
import re
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from . import config as C
from .variants.ablation import ABLATION_SUITE, get_variant
from .metrics import compute_metrics, freeflow_name_for_variant


TRIP_FILENAME_RE = re.compile(r"^(A\d)_([sS]\d)_seed(\d+)_trips\.csv$")


def discover_runs(results_dir: Path) -> List[dict]:
    """Scan results_dir for trip CSVs, return structured list."""
    runs = []
    if not results_dir.exists():
        return runs
    for p in sorted(results_dir.glob("*_trips.csv")):
        m = TRIP_FILENAME_RE.match(p.name)
        if not m:
            continue
        variant_id, scenario_id, seed = m.group(1), m.group(2).upper(), int(m.group(3))
        runs.append({
            "path": p,
            "variant": variant_id,
            "scenario": scenario_id,
            "seed": seed,
        })
    return runs


def write_table(metrics_rows: List[dict], out_csv: Path) -> None:
    """Write Table II: one row per (variant, scenario), averaged over seeds."""
    if not metrics_rows:
        print(f"[report] no metrics to write.")
        return

    # Group by (variant, scenario) and average scalar metrics across seeds.
    groups: Dict[Tuple[str, str], List[dict]] = defaultdict(list)
    for m in metrics_rows:
        groups[(m["variant"], m["scenario"])].append(m)

    scalar_fields = [
        "n_trips", "mean_tt_s", "median_tt_s", "t95_tt_s",
        "buffer_index", "total_vehicle_hours", "total_vehicle_km",
        "mean_wt_s", "p95_wt_s", "frac_neg_wt",
        "gini_class_means", "admit_rate", "mean_express_time_s",
    ]

    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["variant", "scenario", "n_seeds"] + scalar_fields)
        for (vid, sid), rows in sorted(groups.items()):
            row = [vid, sid, len(rows)]
            for field in scalar_fields:
                vals = [r.get(field) for r in rows if isinstance(r.get(field), (int, float))
                        and r.get(field) == r.get(field)]  # filter NaN
                row.append(f"{statistics.mean(vals):.3f}" if vals else "")
            w.writerow(row)
    print(f"[report] wrote {out_csv}  ({len(groups)} (variant, scenario) cells)")


def try_plot(metrics_rows: List[dict], out_dir: Path) -> None:
    """Produce Figures 2-5 if matplotlib is available; skip gracefully otherwise."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[report] matplotlib not installed; skipping figures. "
              "pip install matplotlib to enable.")
        return

    out_dir.mkdir(parents=True, exist_ok=True)

    # Average per variant (collapse scenarios for Figure 3)
    by_variant: Dict[str, List[dict]] = defaultdict(list)
    for m in metrics_rows:
        by_variant[m["variant"]].append(m)

    variants = sorted(by_variant.keys())
    mean_tts = []
    mean_wts = []
    for v in variants:
        tts = [m["mean_tt_s"] for m in by_variant[v] if isinstance(m.get("mean_tt_s"), (int, float))]
        wts = [m["mean_wt_s"] for m in by_variant[v] if isinstance(m.get("mean_wt_s"), (int, float))
               and m["mean_wt_s"] == m["mean_wt_s"]]
        mean_tts.append(statistics.mean(tts) if tts else 0)
        mean_wts.append(statistics.mean(wts) if wts else 0)

    # Fig 3: ablation bars
    fig, ax = plt.subplots(1, 2, figsize=(10, 4))
    ax[0].bar(variants, mean_tts)
    ax[0].set_ylabel("Mean TT (s)")
    ax[0].set_title("Fig 3a: Ablation — Mean Travel Time")
    ax[0].tick_params(axis="x", rotation=45)
    ax[1].bar(variants, mean_wts, color="tab:orange")
    ax[1].set_ylabel("Mean WT (s)")
    ax[1].set_title("Fig 3b: Ablation — Mean Wasted Time")
    ax[1].tick_params(axis="x", rotation=45)
    fig.tight_layout()
    fig.savefig(out_dir / "fig3_ablation_bars.png", dpi=150)
    plt.close(fig)
    print(f"[report] wrote {out_dir / 'fig3_ablation_bars.png'}")

    # Fig 5: class-wise mean TT (requires _class_means in metrics)
    all_classes = set()
    for m in metrics_rows:
        if m.get("_class_means"):
            all_classes.update(m["_class_means"].keys())
    if all_classes:
        classes = sorted(all_classes)
        fig, ax = plt.subplots(figsize=(10, 5))
        width = 0.8 / max(1, len(variants))
        for i, v in enumerate(variants):
            vals = []
            for c in classes:
                xs = [m["_class_means"].get(c) for m in by_variant[v] if m.get("_class_means")]
                xs = [x for x in xs if isinstance(x, (int, float))]
                vals.append(statistics.mean(xs) if xs else 0)
            xpos = [j + i * width for j in range(len(classes))]
            ax.bar(xpos, vals, width=width, label=v)
        ax.set_xticks([j + (len(variants) - 1) * width / 2 for j in range(len(classes))])
        ax.set_xticklabels(classes, rotation=30, ha="right")
        ax.set_ylabel("Mean TT (s)")
        ax.set_title("Fig 5: Class-wise Mean Travel Time")
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(out_dir / "fig5_class_fairness.png", dpi=150)
        plt.close(fig)
        print(f"[report] wrote {out_dir / 'fig5_class_fairness.png'}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--results-dir", default=str(C.RESULTS_DIR))
    args = ap.parse_args()

    results_dir = Path(args.results_dir).resolve()
    print(f"[report] scanning {results_dir}")

    runs = discover_runs(results_dir)
    if not runs:
        print(f"[report] no *_trips.csv found in {results_dir}; nothing to report.")
        return

    print(f"[report] found {len(runs)} trip logs:")
    for r in runs:
        print(f"  {r['variant']}  {r['scenario']}  seed={r['seed']}  {r['path'].name}")

    metrics_rows: List[dict] = []
    for r in runs:
        v = get_variant(r["variant"])
        ff_name = freeflow_name_for_variant(v)
        ff_path = results_dir / ff_name
        if not ff_path.exists():
            print(f"[report] warning: no free-flow reference at {ff_path} "
                  f"for variant {v.id}; WT will be nan.")
        m = compute_metrics(
            str(r["path"]),
            str(ff_path) if ff_path.exists() else None,
            variant_id=v.id,
            scenario_id=r["scenario"],
        )
        m["seed"] = r["seed"]
        metrics_rows.append(m)

    # Print individual reports
    from .metrics import print_report
    for m in metrics_rows:
        print_report(m)

    # Aggregate to Table II
    table_path = results_dir / "results_table.csv"
    write_table(metrics_rows, table_path)

    # Plots
    try_plot(metrics_rows, results_dir / "figures")

    # Paper guard: traffic-reduction check A0 vs A8 (if both exist)
    from .metrics import compare_traffic_reduction
    a0 = next((m for m in metrics_rows if m["variant"] == "A0"), None)
    a8 = next((m for m in metrics_rows if m["variant"] == "A8"), None)
    if a0 and a8:
        compare_traffic_reduction(a0, a8)


if __name__ == "__main__":
    main()
