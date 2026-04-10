# metrics_wt_compare.py
#
# Compute travel-time and wasted-time metrics using a free-flow reference.
#
# Usage:
#   python metrics_wt_compare.py free_flow_times.csv baseline_wt_trips.csv
#   python metrics_wt_compare.py free_flow_times.csv mp_ltfs1.0.1_wt_trips.csv
#
# Assumes:
#   free_flow_times.csv has columns:
#       vehID,routeID,class,TT_freeflow
#
#   trips CSV has columns:
#       vehID,routeID,class,depart,arrival,travel_time

import csv
import sys
import math
import statistics


def percentile(values, p):
    """Linear interpolation percentile (p in [0,1])."""
    if not values:
        return float("nan")
    vals = sorted(values)
    n = len(vals)
    if n == 1:
        return vals[0]
    k = (n - 1) * p
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return vals[int(k)]
    d0 = vals[f] * (c - k)
    d1 = vals[c] * (k - f)
    return d0 + d1


def gini(values):
    """Gini coefficient for a list of non-negative values."""
    if not values:
        return float("nan")
    vals = sorted(v for v in values if v >= 0)
    n = len(vals)
    if n == 0:
        return float("nan")
    total = sum(vals)
    if total == 0:
        return 0.0
    cum = 0.0
    for i, v in enumerate(vals, start=1):
        cum += i * v
    # standard discrete Gini formula
    return (2.0 * cum) / (n * total) - (n + 1.0) / n


def load_free_flow(path):
    """
    Load free-flow times and return a mapping:
        routeID -> average TT_freeflow over all vehicles on that route.
    """
    per_route = {}

    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            route_id = row.get("routeID", "")
            if not route_id:
                continue
            try:
                tt_ff = float(row["TT_freeflow"])
            except (KeyError, ValueError):
                continue
            per_route.setdefault(route_id, []).append(tt_ff)

    free = {}
    for route_id, times in per_route.items():
        free[route_id] = sum(times) / len(times)

    return free


def load_trips(path, free_flow_map):
    """
    Load trips and attach TT_freeflow and WT when possible.

    Returns:
        tt_list  - list of travel times
        wt_list  - list of wasted times
    """
    tt_list = []
    wt_list = []
    used = 0
    skipped = 0

    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            route_id = row.get("routeID", "")
            if not route_id:
                skipped += 1
                continue
            tt_ff = free_flow_map.get(route_id)
            if tt_ff is None:
                skipped += 1
                continue
            try:
                tt = float(row["travel_time"])
            except (KeyError, ValueError):
                skipped += 1
                continue

            tt_list.append(tt)
            wt_list.append(tt - tt_ff)
            used += 1

    return tt_list, wt_list, used, skipped


def print_metrics(label, tt_list, wt_list, used, skipped):
    print(f"=== Metrics for {label} ===")
    print(f"Trips with free-flow match: {used}")
    print(f"Trips skipped (no routeID or TT_ff): {skipped}")
    print()

    if not tt_list:
        print("No usable trips, cannot compute metrics.")
        return

    # Travel time metrics
    mean_tt = statistics.mean(tt_list)
    std_tt = statistics.pstdev(tt_list) if len(tt_list) > 1 else 0.0
    med_tt = statistics.median(tt_list)
    p95_tt = percentile(tt_list, 0.95)
    bi_tt = (p95_tt - med_tt) / med_tt if med_tt > 0 else float("nan")
    gini_tt = gini(tt_list)

    print("Travel time (TT) metrics [seconds]:")
    print(f"  Mean TT:      {mean_tt:.2f}")
    print(f"  Std dev TT:   {std_tt:.2f}")
    print(f"  Median TT:    {med_tt:.2f}")
    print(f"  95th perc TT: {p95_tt:.2f}")
    print(f"  Buffer index: {bi_tt:.3f}")
    print(f"  Gini (TT):    {gini_tt:.3f}")
    print()

    # Wasted time metrics
    if wt_list:
        mean_wt = statistics.mean(wt_list)
        std_wt = statistics.pstdev(wt_list) if len(wt_list) > 1 else 0.0
        med_wt = statistics.median(wt_list)
        p95_wt = percentile(wt_list, 0.95)
        bi_wt = (p95_wt - med_wt) / med_wt if med_wt > 0 else float("nan")
        gini_wt = gini(wt_list)

        print("Wasted time (WT = TT_actual - TT_freeflow) metrics [seconds]:")
        print(f"  Mean WT:      {mean_wt:.2f}")
        print(f"  Std dev WT:   {std_wt:.2f}")
        print(f"  Median WT:    {med_wt:.2f}")
        print(f"  95th perc WT: {p95_wt:.2f}")
        print(f"  Buffer index: {bi_wt:.3f}")
        print(f"  Gini (WT):    {gini_wt:.3f}")
    else:
        print("No wasted time data available.")


def main():
    if len(sys.argv) != 3:
        print("Usage:")
        print("  python metrics_wt_compare.py free_flow_times.csv trips.csv")
        sys.exit(1)

    free_path = sys.argv[1]
    trips_path = sys.argv[2]

    free_map = load_free_flow(free_path)
    if not free_map:
        print(f"No free-flow data loaded from {free_path}")
        sys.exit(1)

    tt_list, wt_list, used, skipped = load_trips(trips_path, free_map)
    label = trips_path
    print_metrics(label, tt_list, wt_list, used, skipped)


if __name__ == "__main__":
    main()
