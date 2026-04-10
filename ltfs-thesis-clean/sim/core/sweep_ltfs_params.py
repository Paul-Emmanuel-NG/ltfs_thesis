# sweep_ltfs_params.py
#
# Sweep LTFS parameters (THETA, KAPPA), run mp_ltfs1.0.1_wt.py for each pair,
# compute TT and WT metrics, and append results into ltfs_sweep_results.csv.
#
# Requirements:
#   - free_flow_times.csv (from mp_freeflow.py)
#   - baseline_wt_trips.csv (from mp_single_baseline_wt.py)
#   - metrics_wt_all.py in the same folder
#   - mp_ltfs1.0.1_wt.py modified to read THETA, KAPPA from env:
#       THETA = float(os.getenv("LTFS_THETA", "60.0"))
#       KAPPA = float(os.getenv("LTFS_KAPPA", "0.85"))

import csv
import os
import sys
import subprocess
import statistics
from pathlib import Path

from metrics_wt_all import (
    load_free_flow,
    load_trips,
    percentile,
    gini,
    FREEFLOW_MP_CSV,
    BASELINE_TRIPS_CSV,
    LTFS_TRIPS_CSV,
    RAW_OUTPUT_DIR,
)

CORE_DIR = Path(__file__).resolve().parent
FREE_FLOW_CSV = FREEFLOW_MP_CSV
BASELINE_CSV = BASELINE_TRIPS_CSV
LTFS_CSV = LTFS_TRIPS_CSV


# Grid of parameters to try
THETA_VALUES = [30.0, 60.0, 90.0]      # seconds
KAPPA_VALUES = [0.7, 0.85, 0.95]       # occupancy caps


def compute_metrics(tt_list, wt_list):
    """Return a dict of metrics for TT and WT or None if no trips."""
    if not tt_list:
        return None

    # TT metrics
    mean_tt = statistics.mean(tt_list)
    std_tt = statistics.pstdev(tt_list) if len(tt_list) > 1 else 0.0
    med_tt = statistics.median(tt_list)
    p95_tt = percentile(tt_list, 0.95)
    bi_tt = (p95_tt - med_tt) / med_tt if med_tt > 0 else float("nan")
    g_tt = gini(tt_list)

    metrics = {
        "mean_tt": mean_tt,
        "std_tt": std_tt,
        "med_tt": med_tt,
        "p95_tt": p95_tt,
        "bi_tt": bi_tt,
        "gini_tt": g_tt,
    }

    # WT metrics if available
    if wt_list:
        mean_wt = statistics.mean(wt_list)
        std_wt = statistics.pstdev(wt_list) if len(wt_list) > 1 else 0.0
        med_wt = statistics.median(wt_list)
        p95_wt = percentile(wt_list, 0.95)
        bi_wt = (p95_wt - med_wt) / med_wt if med_wt > 0 else float("nan")
        g_wt = gini(wt_list)
    else:
        mean_wt = std_wt = med_wt = p95_wt = bi_wt = g_wt = float("nan")

    metrics.update({
        "mean_wt": mean_wt,
        "std_wt": std_wt,
        "med_wt": med_wt,
        "p95_wt": p95_wt,
        "bi_wt": bi_wt,
        "gini_wt": g_wt,
    })

    return metrics


def main():
    # 1) Load free-flow map
    free_map = load_free_flow(FREE_FLOW_CSV)
    if not free_map:
        print(f"No free-flow data loaded from {FREE_FLOW_CSV}.")
        print("Run mp_freeflow.py first.")
        return

    # 2) Load baseline trips and metrics once
    tt_b, wt_b, used_b, skipped_b = load_trips(BASELINE_CSV, free_map)
    base_metrics = compute_metrics(tt_b, wt_b)
    if base_metrics is None:
        print("No usable baseline trips. Run mp_single_baseline_wt.py first.")
        return

    print(f"Baseline trips used: {used_b}, skipped: {skipped_b}")
    print("Baseline mean TT: {:.2f} s, mean WT: {:.2f} s".format(
        base_metrics["mean_tt"], base_metrics["mean_wt"]
    ))
    print()

    # 3) Open CSV for writing results
    RAW_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    out_path = str(RAW_OUTPUT_DIR / "ltfs_sweep_results.csv")
    new_file = not os.path.exists(out_path)

    with open(out_path, "a", newline="") as f:
        writer = csv.writer(f)

        # Header if new file
        if new_file:
            writer.writerow([
                "theta",
                "kappa",
                "trips_used",
                "trips_skipped",
                "mean_tt",
                "std_tt",
                "med_tt",
                "p95_tt",
                "bi_tt",
                "gini_tt",
                "mean_wt",
                "std_wt",
                "med_wt",
                "p95_wt",
                "bi_wt",
                "gini_wt",
                "rel_mean_tt_pct",
                "rel_mean_wt_pct",
            ])

        # 4) Loop over parameter grid
        for theta in THETA_VALUES:
            for kappa in KAPPA_VALUES:
                print(f"Running LTFS with THETA={theta}, KAPPA={kappa} ...")

                # Set env vars for this run
                env = os.environ.copy()
                env["LTFS_THETA"] = str(theta)
                env["LTFS_KAPPA"] = str(kappa)

                # Run LTFS controller (this overwrites mp_ltfs1.0.1_wt_trips.csv)
                subprocess.run(
                    [sys.executable, str(CORE_DIR / "mp_ltfs1.0.1_wt.py")],
                    check=True,
                    env=env,
                    cwd=str(CORE_DIR),
                )

                # Load LTFS trips for this parameter pair
                tt_l, wt_l, used_l, skipped_l = load_trips(LTFS_CSV, free_map)
                ltfs_metrics = compute_metrics(tt_l, wt_l)

                if ltfs_metrics is None:
                    print("  No usable LTFS trips, skipping.")
                    continue

                # Relative changes vs baseline (mean TT and mean WT)
                def rel_change(new_val, base_val):
                    if base_val == 0 or base_val != base_val:  # NaN check
                        return float("nan")
                    return 100.0 * (new_val - base_val) / base_val

                rel_mean_tt = rel_change(ltfs_metrics["mean_tt"], base_metrics["mean_tt"])
                rel_mean_wt = rel_change(ltfs_metrics["mean_wt"], base_metrics["mean_wt"])

                print("  Trips used: {}, skipped: {}".format(used_l, skipped_l))
                print("  mean TT: {:.2f} s (Δ {:.2f} percent)".format(
                    ltfs_metrics["mean_tt"], rel_mean_tt
                ))
                print("  mean WT: {:.2f} s (Δ {:.2f} percent)".format(
                    ltfs_metrics["mean_wt"], rel_mean_wt
                ))
                print()

                # Write one row to CSV
                writer.writerow([
                    theta,
                    kappa,
                    used_l,
                    skipped_l,
                    ltfs_metrics["mean_tt"],
                    ltfs_metrics["std_tt"],
                    ltfs_metrics["med_tt"],
                    ltfs_metrics["p95_tt"],
                    ltfs_metrics["bi_tt"],
                    ltfs_metrics["gini_tt"],
                    ltfs_metrics["mean_wt"],
                    ltfs_metrics["std_wt"],
                    ltfs_metrics["med_wt"],
                    ltfs_metrics["p95_wt"],
                    ltfs_metrics["bi_wt"],
                    ltfs_metrics["gini_wt"],
                    rel_mean_tt,
                    rel_mean_wt,
                ])

    print(f"Sweep complete. Results appended to {out_path}")


if __name__ == "__main__":
    main()
