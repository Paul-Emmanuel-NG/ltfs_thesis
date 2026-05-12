"""
metrics/compute.py — Sec IX evaluation metrics.

Consumes trip CSVs produced by the runner plus policy-conditioned free-flow
references (Eq 28) produced by metrics/freeflow.py. Computes:

  Efficiency (IX-B): mean TT, median TT, total vehicle-hours, vehicle-km
  Reliability (IX-C): T95, buffer index (Eq 29)
  Fairness (IX-D): class-wise mean TT/WT, Gini over class means
  WT (Eq 27): excess over policy-conditioned free-flow
  LTFS-specific: gate admission rate, express usage, occupancy exceedance

Paper check (W16): traffic-reduction claim requires mean TT ↓ AND total
vehicle-hours ↓ relative to the relevant control variant. We compute both
and print a clear warning when one improves and the other doesn't.
"""

from __future__ import annotations
import csv
import math
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from ..standard_blocks import buffer_index, gini


# ============================================================================
# Trip CSV schema
# ============================================================================
# Written by runner:
#   vehID, routeKey, class, depart, arrival, travel_time, distance_m,
#   on_express_s, admitted, denied

TRIP_COLS = [
    "vehID", "routeKey", "class", "depart", "arrival",
    "travel_time", "distance_m", "on_express_s", "admitted", "denied",
]


def _percentile(values: List[float], p: float) -> float:
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
    return vals[f] * (c - k) + vals[c] * (k - f)


# ============================================================================
# Free-flow loader (Eq 28)
# ============================================================================

def load_freeflow(path: str) -> Dict[Tuple[str, str], float]:
    """
    Load policy-conditioned free-flow references, keyed on (routeKey, class).
    Takes the minimum observed value per key (Eq 28).
    """
    out: Dict[Tuple[str, str], float] = {}
    p = Path(path)
    if not p.exists():
        return out
    with open(p, newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            rk = (row.get("routeKey") or "").strip()
            cls = (row.get("class") or "").strip()
            if not rk:
                continue
            try:
                tt_ff = float(row.get("TT_freeflow", "nan"))
            except ValueError:
                continue
            if math.isnan(tt_ff) or tt_ff <= 0:
                continue
            key = (rk, cls)
            if key not in out or tt_ff < out[key]:
                out[key] = tt_ff
    return out


# ============================================================================
# Trip loader
# ============================================================================

def load_trips(path: str) -> List[dict]:
    trips: List[dict] = []
    p = Path(path)
    if not p.exists():
        return trips
    with open(p, newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            trips.append(row)
    return trips


def _safe_float(x: str, default: float = float("nan")) -> float:
    try:
        return float(x)
    except (TypeError, ValueError):
        return default


# ============================================================================
# Metric bundle
# ============================================================================

def compute_metrics(
    trips_path: str,
    freeflow_path: Optional[str] = None,
    variant_id: str = "",
    scenario_id: str = "",
) -> dict:
    """
    Compute the full metric suite for a single run.

    Returns a dict of named scalars + per-class breakdowns, ready for CSV
    aggregation across variants/scenarios/seeds.
    """
    trips = load_trips(trips_path)
    ff = load_freeflow(freeflow_path) if freeflow_path else {}

    # Filter: only trips with both depart and travel_time. We don't filter by
    # warmup here — the runner is responsible for including only trips whose
    # depart > WARMUP_S in the CSV in the first place.
    clean = []
    for t in trips:
        tt = _safe_float(t.get("travel_time", ""))
        if math.isnan(tt) or tt <= 0:
            continue
        clean.append(t)

    if not clean:
        return {
            "variant": variant_id,
            "scenario": scenario_id,
            "n_trips": 0,
            "error": "no completed trips",
        }

    tt_list = [_safe_float(t["travel_time"]) for t in clean]
    dist_list = [_safe_float(t.get("distance_m", "0"), 0.0) for t in clean]
    class_list = [t.get("class", "") for t in clean]
    on_exp = [_safe_float(t.get("on_express_s", "0"), 0.0) for t in clean]
    admitted = sum(1 for t in clean if (t.get("admitted") or "") == "True")
    denied = sum(1 for t in clean if (t.get("denied") or "") == "True")
    gate_eligible = admitted + denied

    # Efficiency
    mean_tt = statistics.mean(tt_list)
    median_tt = statistics.median(tt_list)
    total_veh_hours = sum(tt_list) / 3600.0
    total_veh_km = sum(dist_list) / 1000.0

    # Reliability
    t50, t95, bi = buffer_index(tt_list)

    # Fairness — class-wise mean TT, then Gini over the class means
    class_tts: Dict[str, List[float]] = defaultdict(list)
    for c, tt in zip(class_list, tt_list):
        class_tts[c].append(tt)
    class_means = {c: statistics.mean(ts) for c, ts in class_tts.items() if ts}
    gini_class = gini(list(class_means.values()))

    # WT (Eq 27)
    wt_list: List[float] = []
    class_wts: Dict[str, List[float]] = defaultdict(list)
    matched = 0
    unmatched = 0
    for t in clean:
        rk = t.get("routeKey", "")
        cls = t.get("class", "")
        tt = _safe_float(t["travel_time"])
        tt_ff = ff.get((rk, cls))
        if tt_ff is None:
            unmatched += 1
            continue
        wt = tt - tt_ff
        wt_list.append(wt)
        class_wts[cls].append(wt)
        matched += 1

    mean_wt = statistics.mean(wt_list) if wt_list else float("nan")
    p95_wt = _percentile(wt_list, 0.95) if wt_list else float("nan")
    frac_neg_wt = (sum(1 for w in wt_list if w < 0) / len(wt_list)) if wt_list else float("nan")

    # LTFS-specific
    admit_rate = (admitted / gate_eligible) if gate_eligible > 0 else float("nan")
    mean_express_s = statistics.mean(on_exp) if on_exp else 0.0

    return {
        "variant": variant_id,
        "scenario": scenario_id,
        "n_trips": len(clean),
        # Efficiency
        "mean_tt_s": mean_tt,
        "median_tt_s": median_tt,
        "t95_tt_s": t95,
        "total_vehicle_hours": total_veh_hours,
        "total_vehicle_km": total_veh_km,
        # Reliability
        "buffer_index": bi,
        # Fairness
        "gini_class_means": gini_class,
        "n_classes": len(class_means),
        # WT
        "matched_trips": matched,
        "unmatched_trips": unmatched,
        "mean_wt_s": mean_wt,
        "p95_wt_s": p95_wt,
        "frac_neg_wt": frac_neg_wt,
        # LTFS
        "admit_rate": admit_rate,
        "mean_express_time_s": mean_express_s,
        # Raw class-wise for downstream use
        "_class_means": class_means,
        "_class_wt_means": {c: statistics.mean(ws) for c, ws in class_wts.items() if ws},
    }


# ============================================================================
# CLI: compute for a single run or multiple
# ============================================================================

def print_report(m: dict) -> None:
    print(f"\n=== Variant {m.get('variant', '?')} on Scenario {m.get('scenario', '?')} ===")
    if m.get("n_trips", 0) == 0:
        print(f"  {m.get('error', 'no trips')}")
        return
    print(f"  Trips completed: {m['n_trips']}  (WT-matched: {m['matched_trips']}, unmatched: {m['unmatched_trips']})")
    print(f"  Mean TT: {m['mean_tt_s']:.2f} s   Median: {m['median_tt_s']:.2f} s   P95: {m['t95_tt_s']:.2f} s")
    print(f"  Buffer index: {m['buffer_index']:.3f}")
    print(f"  Total vehicle-hours: {m['total_vehicle_hours']:.1f}   vehicle-km: {m['total_vehicle_km']:.1f}")
    print(f"  Mean WT: {m['mean_wt_s']:.2f} s   P95 WT: {m['p95_wt_s']:.2f} s   Fraction WT<0: {m['frac_neg_wt']:.3f}")
    print(f"  Gini(class means): {m['gini_class_means']:.3f}  (over {m['n_classes']} classes)")
    print(f"  Gate admit rate: {m['admit_rate']}   Mean express time: {m['mean_express_time_s']:.1f} s")
    if m.get("_class_means"):
        print(f"  Class means (TT, s):")
        for cls, v in sorted(m["_class_means"].items()):
            print(f"    {cls:30s}  {v:7.2f}")


def compare_traffic_reduction(baseline: dict, treatment: dict) -> None:
    """
    Paper guard (Sec IX-B): traffic reduction = mean TT ↓ AND total
    vehicle-hours ↓. Print a clear warning when these disagree.
    """
    if baseline.get("n_trips", 0) == 0 or treatment.get("n_trips", 0) == 0:
        return
    dTT = (treatment["mean_tt_s"] - baseline["mean_tt_s"]) / baseline["mean_tt_s"] * 100
    dVH = (treatment["total_vehicle_hours"] - baseline["total_vehicle_hours"]) / baseline["total_vehicle_hours"] * 100
    print(f"\n=== {treatment.get('variant')} vs {baseline.get('variant')} ===")
    print(f"  Δ Mean TT: {dTT:+.2f} %     Δ Total vehicle-hours: {dVH:+.2f} %")
    if dTT < 0 and dVH < 0:
        print(f"  [OK] traffic reduction is defensible.")
    elif dTT < 0 and dVH >= 0:
        print(f"  [WARN] mean TT dropped but vehicle-hours didn't. Could be redistribution, not reduction.")
    elif dTT >= 0 and dVH < 0:
        print(f"  [WARN] vehicle-hours dropped but mean TT didn't. Check admission vs completion population.")
