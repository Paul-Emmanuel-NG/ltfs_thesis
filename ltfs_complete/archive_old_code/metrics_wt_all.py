# metrics_wt_all.py
# Option 1A (recommended): Policy-conditioned free-flow references
#
# Run order:
#   1) python mp_freeflow.py          -> produces freeflow_mp.csv + freeflow_ltfs.csv
#   2) python mp_single_baseline_wt.py -> produces baseline_wt_trips.csv
#   3) python mp_ltfs1.0.1_wt.py      -> produces mp_ltfs1.0.1_wt_trips.csv
#   4) python metrics_wt_all.py       -> prints TT/WT metrics using the correct free-flow per policy
#
# Free-flow CSV schema (from mp_freeflow.py):
#   routeKey,class,TT_freeflow
#
# Trip CSV schema (from controllers):
#   routeKey,class,travel_time,(...)   [baseline_wt_trips.csv / mp_ltfs1.0.1_wt_trips.csv]

from __future__ import annotations

import csv
import math
import statistics
from typing import Dict, Tuple, List


# ----------------
# HARD-CODED INPUTS
# ----------------
FREEFLOW_MP_CSV = "freeflow_mp.csv"
FREEFLOW_LTFS_CSV = "freeflow_ltfs.csv"

BASELINE_TRIPS_CSV = "baseline_wt_trips.csv"
LTFS_TRIPS_CSV = "mp_ltfs1.0.1_wt_trips.csv"


# ----------------
# Helpers
# ----------------
def percentile(values: List[float], p: float) -> float:
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


def gini(values: List[float]) -> float:
    """
    Standard Gini for non-negative values.
    """
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
    return (2.0 * cum) / (n * total) - (n + 1.0) / n


def _trip_key(row: dict) -> Tuple[str, str]:
    """
    Join key: (routeKey_or_routeID, class)
    - routeKey is preferred (stable across runs).
    - class is vehicle type id (passenger/bus/truck/...)
    """
    rk = (row.get("routeKey") or row.get("routeID") or "").strip()
    cls = (row.get("class") or "").strip()
    return rk, cls


def load_free_flow(path: str) -> Dict[Tuple[str, str], float]:
    """
    (routeKey,class) -> TT_freeflow
    If duplicates exist, keep the minimum TT_freeflow (best observed free-flow).
    """
    out: Dict[Tuple[str, str], float] = {}
    rows_seen = 0
    rows_used = 0
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            rows_seen += 1
            rk, cls = _trip_key(row)
            if not rk:
                continue
            try:
                ttff = float(row.get("TT_freeflow", "nan"))
            except ValueError:
                continue
            if math.isnan(ttff) or ttff <= 0:
                continue
            rows_used += 1
            key = (rk, cls)
            if key not in out or ttff < out[key]:
                out[key] = ttff
    print(f"=== Free-flow reference coverage ({path}) ===")
    print({"rows_seen": rows_seen, "rows_used": rows_used, "unique_pairs": len(out)})
    print()
    return out


def load_trips(path: str, free_map: Dict[Tuple[str, str], float]):
    tt_list: List[float] = []
    wt_list: List[float] = []
    used = 0
    skipped = 0

    with open(path, newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            rk, cls = _trip_key(row)
            if not rk:
                skipped += 1
                continue

            key = (rk, cls)
            if key not in free_map:
                skipped += 1
                continue

            try:
                tt = float(row.get("travel_time", "nan"))
            except ValueError:
                skipped += 1
                continue
            if math.isnan(tt) or tt < 0:
                skipped += 1
                continue

            ttff = free_map[key]
            wt = tt - ttff
            tt_list.append(tt)
            wt_list.append(wt)
            used += 1

    return tt_list, wt_list, used, skipped


def print_metrics(label: str, tt_list: List[float], wt_list: List[float], used: int, skipped: int):
    print(f"=== Metrics for {label} ===")
    print(f"Trips with free-flow match: {used}")
    print(f"Trips skipped (no (routeKey,class) TT_ff): {skipped}")
    print()

    if not tt_list:
        print("No usable trips.")
        print()
        return

    mean_tt = statistics.mean(tt_list)
    std_tt = statistics.pstdev(tt_list) if len(tt_list) > 1 else 0.0
    med_tt = statistics.median(tt_list)
    p95_tt = percentile(tt_list, 0.95)
    bi_tt = (p95_tt - med_tt) / med_tt if med_tt > 0 else float("nan")
    g_tt = gini(tt_list)

    print("Travel time (TT) metrics [seconds]:")
    print(f"  Mean TT:      {mean_tt:.2f}")
    print(f"  Std dev TT:   {std_tt:.2f}")
    print(f"  Median TT:    {med_tt:.2f}")
    print(f"  95th perc TT: {p95_tt:.2f}")
    print(f"  Buffer index: {bi_tt}")
    print(f"  Gini (TT):    {g_tt:.3f}")
    print()

    mean_wt = statistics.mean(wt_list)
    std_wt = statistics.pstdev(wt_list) if len(wt_list) > 1 else 0.0
    med_wt = statistics.median(wt_list)
    p95_wt = percentile(wt_list, 0.95)
    bi_wt = (p95_wt - med_wt) / med_wt if med_wt > 0 else float("nan")

    # WT can be negative if TT_freeflow > TT_actual for some trips (artifact of reference choice).
    # We report inequality on magnitude as well as fraction WT<0 for transparency.
    g_wt_mag = gini([abs(x) for x in wt_list])
    frac_neg = sum(1 for x in wt_list if x < 0) / len(wt_list) if wt_list else float("nan")

    print("Wasted time (WT = TT_actual - TT_freeflow) metrics [seconds]:")
    print(f"  Mean WT:      {mean_wt:.2f}")
    print(f"  Std dev WT:   {std_wt:.2f}")
    print(f"  Median WT:    {med_wt:.2f}")
    print(f"  95th perc WT: {p95_wt:.2f}")
    print(f"  Buffer index: {bi_wt}")
    print(f"  Gini (|WT|):  {g_wt_mag:.3f}")
    print(f"  Fraction WT<0: {frac_neg:.4f}")
    print()


def main():
    # Load policy-conditioned free-flow references
    ff_mp = load_free_flow(FREEFLOW_MP_CSV)
    ff_ltfs = load_free_flow(FREEFLOW_LTFS_CSV)

    tt_b, wt_b, used_b, skipped_b = load_trips(BASELINE_TRIPS_CSV, ff_mp)
    print_metrics("Single-layer Max Pressure baseline", tt_b, wt_b, used_b, skipped_b)

    tt_l, wt_l, used_l, skipped_l = load_trips(LTFS_TRIPS_CSV, ff_ltfs)
    print_metrics("LTFS TST (v1.0.1)", tt_l, wt_l, used_l, skipped_l)

    if tt_b and tt_l:
        mean_b = statistics.mean(tt_b)
        mean_l = statistics.mean(tt_l)
        mean_wb = statistics.mean(wt_b) if wt_b else float("nan")
        mean_wl = statistics.mean(wt_l) if wt_l else float("nan")

        print("=== Relative changes (LTFS vs baseline) ===")
        print(f"Δ Mean TT: {(mean_l - mean_b) / mean_b * 100:.2f} %")
        if math.isfinite(mean_wb) and mean_wb != 0 and math.isfinite(mean_wl):
            print(f"Δ Mean WT: {(mean_wl - mean_wb) / mean_wb * 100:.2f} %")
        else:
            print("Δ Mean WT: nan (baseline mean WT is 0 or undefined)")


if __name__ == "__main__":
    main()
