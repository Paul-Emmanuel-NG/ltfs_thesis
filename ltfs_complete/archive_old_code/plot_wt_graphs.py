# plot_wt_graphs.py
#
# Make graphs for WT (wasted time) using the existing metrics_wt_all pipeline.
#
# Assumes that these files already exist in the same folder:
#   free_flow_times.csv
#   baseline_wt_trips.csv
#   mp_ltfs1.0.1_wt_trips.csv
#
# And that metrics_wt_all.py is in the same folder.

import statistics
import matplotlib.pyplot as plt

from metrics_wt_all import (
    load_free_flow,
    load_trips,
    percentile,
    FREE_FLOW_CSV,
    BASELINE_CSV,
    LTFS_CSV,
)


def main():
    # 1) Load free-flow map (routeID -> TT_freeflow)
    free_map = load_free_flow(FREE_FLOW_CSV)
    if not free_map:
        print(f"No free-flow data loaded from {FREE_FLOW_CSV}.")
        print("Run mp_freeflow.py first and check free_flow_times.csv.")
        return

    # 2) Load baseline and LTFS trips with WT
    tt_b, wt_b, used_b, skipped_b = load_trips(BASELINE_CSV, free_map)
    tt_l, wt_l, used_l, skipped_l = load_trips(LTFS_CSV, free_map)

    if not wt_b or not wt_l:
        print("No WT data for one or both controllers. Check CSVs and rerun sims.")
        return

    print(f"Baseline trips used: {used_b}, skipped: {skipped_b}")
    print(f"LTFS trips used:    {used_l}, skipped: {skipped_l}")

    # 3) Compute summary WT metrics
    mean_wt_b = statistics.mean(wt_b)
    mean_wt_l = statistics.mean(wt_l)
    p95_wt_b = percentile(wt_b, 0.95)
    p95_wt_l = percentile(wt_l, 0.95)

    print()
    print("Baseline WT: mean = {:.2f}, P95 = {:.2f}".format(mean_wt_b, p95_wt_b))
    print("LTFS WT:     mean = {:.2f}, P95 = {:.2f}".format(mean_wt_l, p95_wt_l))
    print()

    # 4) Graph 1: WT distribution histogram
    plt.figure()
    plt.hist(wt_b, bins=50, alpha=0.5, label="Baseline MP")
    plt.hist(wt_l, bins=50, alpha=0.5, label="LTFS TST v1.0.1")
    plt.xlabel("Wasted time WT = TT_actual - TT_freeflow [s]")
    plt.ylabel("Number of trips")
    plt.title("WT distribution: baseline vs LTFS")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig("wt_histogram.png")

    # 5) Graph 2: bar chart of mean WT and 95th percentile WT
    configs = ["Baseline", "LTFS"]
    mean_vals = [mean_wt_b, mean_wt_l]
    p95_vals = [p95_wt_b, p95_wt_l]

    x = range(len(configs))
    width = 0.35

    plt.figure()
    plt.bar([i - width / 2 for i in x], mean_vals, width=width, label="Mean WT")
    plt.bar([i + width / 2 for i in x], p95_vals, width=width, label="95th WT")
    plt.xticks(list(x), configs)
    plt.ylabel("Wasted time [s]")
    plt.title("Mean and 95th percentile WT")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig("wt_summary_bars.png")

    # 6) Show plots on screen
    plt.show()


if __name__ == "__main__":
    main()
