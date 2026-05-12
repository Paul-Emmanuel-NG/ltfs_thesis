"""
verify_paper_data.py — Recompute every numerical claim in the LTFS paper from raw simulation CSVs.

This script reads the raw trip CSVs, recomputes every per-cell mean,
confidence interval, and percentage delta, and writes a self-contained
verification report to verification_report.txt.

Inputs (relative to --results):
  results/{A0..A8}_S1_seed{1..10}_trips.csv             light demand
  results/corridor_pen/{A0..A8}_S1_seed{1..9}_trips.csv corridor-pen
  results/{A0..A8}_S2_seed{1..10}_trips.csv             S2 incident
  results_wmax_{1p2,1p5,2p0}/A8_S1_seed{1..10}_trips.csv  wmax sweep
  results_urgency_theta_*/A8_S1_seed{1..10}_trips.csv   urgency thresholds
  light.{passenger,bus,truck}.rou.xml                   demand counts
  yanan_elevated.net.xml                                network topology

Outputs:
  verification_report.txt — every numerical claim, recomputed

Usage:
  python verify_paper_data.py [--results results] [--out verification_report.txt]

Author: Akinwole Paul-Emmanuel Oreoluwa, Hohai University, 2026
"""
from __future__ import annotations
import argparse
import sys
from pathlib import Path
from datetime import datetime, timezone

import numpy as np
import pandas as pd

VARIANTS = [f"A{i}" for i in range(9)]


def load_trips(d, variant, scenario, seed):
    f = d / f"{variant}_{scenario}_seed{seed}_trips.csv"
    if not f.exists():
        return None
    return pd.read_csv(f)


def per_seed(d, v, scen, n_max=10):
    rows = []
    for seed in range(1, n_max + 1):
        df = load_trips(d, v, scen, seed)
        if df is None or len(df) == 0:
            continue
        rows.append({
            "seed": seed,
            "n_trips": len(df),
            "mean_tt": df["travel_time"].mean(),
            "median_tt": df["travel_time"].median(),
            "p95_tt": df["travel_time"].quantile(0.95),
            "vh_total": df["travel_time"].sum() / 3600.0,
            "bus_mean": df.loc[df["class"] == "bus_bus", "travel_time"].mean()
                        if (df["class"] == "bus_bus").any() else np.nan,
            "bus_n": int((df["class"] == "bus_bus").sum()),
            "truck_mean": df.loc[df["class"] == "truck_truck", "travel_time"].mean()
                          if (df["class"] == "truck_truck").any() else np.nan,
            "truck_n": int((df["class"] == "truck_truck").sum()),
            "passenger_mean": df.loc[df["class"] == "veh_passenger", "travel_time"].mean()
                              if (df["class"] == "veh_passenger").any() else np.nan,
            "passenger_n": int((df["class"] == "veh_passenger").sum()),
        })
    return rows


def mean_ci(values):
    arr = np.asarray([v for v in values if not (isinstance(v, float) and np.isnan(v))])
    n = len(arr)
    if n == 0:
        return (np.nan, np.nan, 0)
    if n == 1:
        return (float(arr[0]), 0.0, 1)
    m = arr.mean()
    sd = arr.std(ddof=1)
    return (float(m), float(1.96 * sd / np.sqrt(n)), n)


def fmt(m, ci, prec=1):
    if np.isnan(m):
        return "NaN"
    return f"{m:.{prec}f} ± {ci:.{prec}f}"


def section(title, fh):
    print("\n" + "=" * 78, file=fh)
    print(title, file=fh)
    print("=" * 78, file=fh)


def verify_demand_counts(rou_dir, fh):
    section("DEMAND COUNTS — verifying \"6,274 demanded vehicles\" claim", fh)
    import xml.etree.ElementTree as ET
    counts = {}
    for kind in ["passenger", "bus", "truck"]:
        f = rou_dir / f"light.{kind}.rou.xml"
        if not f.exists():
            print(f"  [missing] {f}", file=fh)
            continue
        tree = ET.parse(f)
        n = sum(1 for v in tree.getroot().findall("vehicle"))
        counts[kind] = n
        print(f"  light.{kind}.rou.xml: {n} vehicles", file=fh)
    total = sum(counts.values())
    print(f"  Total: {total}", file=fh)
    print(f"\n  Paper claim: 6,274 vehicles  →  {'MATCH' if total == 6274 else 'MISMATCH'}", file=fh)


def verify_completion_rates(results, fh):
    section("COMPLETION RATES — verifying \"~7% complete\" claim", fh)
    for label, base, scen in [
        ("Light demand S1", results, "S1"),
        ("Corridor-pen S1", results / "corridor_pen", "S1"),
        ("S2 incident", results, "S2"),
    ]:
        ns = []
        for v in VARIANTS:
            for seed in range(1, 11):
                df = load_trips(base, v, scen, seed)
                if df is not None:
                    ns.append(len(df))
        if ns:
            print(f"  {label}: avg {np.mean(ns):.0f} completed/seed  "
                  f"({np.mean(ns)/6274*100:.1f}% of 6,274 demanded)",
                  file=fh)
    print(f"\n  Paper claim: ~425 (~7%) complete; ~420 light, ~390 corridor, ~415 S2", file=fh)


def verify_class_populations(results, fh):
    section("CLASS POPULATIONS — verifying \"~17 trucks, ~50 buses, ~350 passenger\" claim", fh)
    for v in ["A0", "A1", "A4", "A8"]:
        rows = per_seed(results, v, "S1")
        if not rows:
            continue
        bus_n = np.mean([r["bus_n"] for r in rows])
        truck_n = np.mean([r["truck_n"] for r in rows])
        passenger_n = np.mean([r["passenger_n"] for r in rows])
        print(f"  {v} light: bus={bus_n:.0f}, truck={truck_n:.0f}, passenger={passenger_n:.0f}",
              file=fh)
    print(f"\n  Paper claims: ~17 trucks, ~50 buses, ~350 passenger per seed", file=fh)


def verify_network_topology(net_xml, fh):
    section("NETWORK TOPOLOGY — verifying \"177 signalized junctions, 3,545 edges\" claim", fh)
    try:
        import sumolib
    except ImportError:
        print("  [sumolib unavailable; skipping]", file=fh)
        return
    net = sumolib.net.readNet(str(net_xml))
    edges = net.getEdges()
    nodes = net.getNodes()
    tls = [n for n in nodes if n.getType() == "traffic_light"]
    xs = [n.getCoord()[0] for n in nodes]
    ys = [n.getCoord()[1] for n in nodes]
    print(f"  Total non-internal edges: {len(edges)}", file=fh)
    print(f"  Traffic-light junctions: {len(tls)}", file=fh)
    print(f"  Bounding box: {(max(xs)-min(xs))/1000:.2f} km × {(max(ys)-min(ys))/1000:.2f} km",
          file=fh)
    print(f"\n  Paper claims: 177 signalized, 3,545 edges, 3.27×1.69 km", file=fh)


def verify_table_class_wise(results, scen, base, n_max, label, fh):
    section(f"TABLE — {label}: class-wise mean TT recomputation", fh)
    print(f"  {'Var':<4} {'n':>3} {'Bus mean ± CI':>20} {'Truck mean ± CI':>20} "
          f"{'Pass mean ± CI':>20}", file=fh)
    a1_bus = None
    for v in VARIANTS:
        rows = per_seed(base, v, scen, n_max=n_max)
        if not rows:
            continue
        bm, bc, bn = mean_ci([r["bus_mean"] for r in rows])
        tm, tc, tn = mean_ci([r["truck_mean"] for r in rows])
        pm, pc, pn = mean_ci([r["passenger_mean"] for r in rows])
        if v == "A1":
            a1_bus = bm
        delta = ""
        if a1_bus and v != "A1" and not np.isnan(bm):
            delta = f"  Δ_bus_vs_A1={(bm-a1_bus)/a1_bus*100:+.1f}%"
        print(f"  {v:<4} {len(rows):>3} {fmt(bm, bc, 1):>20} {fmt(tm, tc, 1):>20} "
              f"{fmt(pm, pc, 1):>20}{delta}", file=fh)


def verify_table_fleet(results, scen, base, n_max, label, fh):
    section(f"TABLE — {label}: fleet-wide recomputation", fh)
    print(f"  {'Var':<4} {'n':>3} {'Mean TT':>15} {'P95 TT':>15} {'VH':>15} "
          f"{'Δ VH vs A1':>14}", file=fh)
    a1_vh = None
    for v in VARIANTS:
        rows = per_seed(base, v, scen, n_max=n_max)
        if not rows:
            continue
        mt, ct, nt = mean_ci([r["mean_tt"] for r in rows])
        p9, pc, p9n = mean_ci([r["p95_tt"] for r in rows])
        vh, vc, vn = mean_ci([r["vh_total"] for r in rows])
        if v == "A1":
            a1_vh = vh
        delta = ""
        if a1_vh and v != "A1" and not np.isnan(vh):
            delta = f"{(vh - a1_vh)/a1_vh*100:+.2f}%"
        print(f"  {v:<4} {len(rows):>3} {fmt(mt, ct, 1):>15} {fmt(p9, pc, 0):>15} "
              f"{fmt(vh, vc, 1):>15} {delta:>14}", file=fh)


def verify_speed_claim(results, fh):
    section("SPEEDS — verifying \"13–14 km/h per-trip, ~5 km/h system\" claim", fh)
    for v in ["A0", "A1", "A4", "A8"]:
        per_trip_speeds = []
        sys_speeds = []
        for seed in range(1, 11):
            df = load_trips(results, v, "S1", seed)
            if df is None:
                continue
            d_h = df["travel_time"] / 3600
            dist_km = df["distance_m"] / 1000
            df["speed_kph"] = (dist_km / d_h.replace(0, np.nan))
            per_trip_speeds.append(df["speed_kph"].mean())
            sys_speeds.append(dist_km.sum() / d_h.sum() if d_h.sum() > 0 else np.nan)
        if per_trip_speeds:
            print(f"  {v}: per-trip mean speed = {np.mean(per_trip_speeds):.2f} km/h, "
                  f"system-mean = {np.mean(sys_speeds):.2f} km/h", file=fh)
    print(f"\n  Paper claims: per-trip 13–14 km/h, system-mean ~5 km/h", file=fh)


def verify_wmax_sweep(results, fh):
    section("WMAX SWEEP — verifying n=10 sensitivity claim", fh)
    parent = results.parent
    for label, dirname in [
        ("wmax=1.2", "results_wmax_1p2"),
        ("wmax=1.5 (default)", "results_wmax_1p5"),
        ("wmax=2.0", "results_wmax_2p0"),
    ]:
        d = parent / dirname
        if not d.exists():
            print(f"  {label}: [missing dir {dirname}]", file=fh)
            continue
        seed_means = []
        seed_p95s = []
        for seed in range(1, 11):
            f = d / f"A8_S1_seed{seed}_trips.csv"
            if not f.exists():
                continue
            df = pd.read_csv(f)
            seed_means.append(df["travel_time"].mean())
            seed_p95s.append(df["travel_time"].quantile(0.95))
        if seed_means:
            m, ci, n = mean_ci(seed_means)
            mp95 = np.mean(seed_p95s)
            print(f"  {label}: n={n}, per-seed mean TT = {m:.1f} ± {ci:.1f}, "
                  f"mean P95 = {mp95:.0f}", file=fh)
    print(f"\n  Paper claims: 746.3 / 738.0 / 747.3 per-seed mean TT, "
          f"P95 ~2066/1999/2036", file=fh)


def verify_threshold_sweep(results, fh):
    section("THRESHOLD SENSITIVITY — verifying THETA_S and THETA_U_S sweep", fh)
    parent = results.parent
    cells = [
        ("THETA_S=3.0", "results_urgency_theta_s_3"),
        ("THETA_S=5.0 (default)", "results_urgency_theta_s_5"),
        ("THETA_S=7.0", "results_urgency_theta_s_7"),
        ("THETA_U_S=1.5", "results_urgency_theta_u_s_1p5"),
        ("THETA_U_S=3.0 (default)", "results_urgency_theta_u_s_3"),
        ("THETA_U_S=4.5", "results_urgency_theta_u_s_4p5"),
    ]
    for label, dirname in cells:
        d = parent / dirname
        if not d.exists():
            print(f"  {label}: [missing dir {dirname}]", file=fh)
            continue
        seed_means = []
        for seed in range(1, 11):
            f = d / f"A8_S1_seed{seed}_trips.csv"
            if not f.exists():
                continue
            df = pd.read_csv(f)
            seed_means.append(df["travel_time"].mean())
        if seed_means:
            m, ci, n = mean_ci(seed_means)
            print(f"  {label}: n={n}, per-seed mean TT = {m:.1f} ± {ci:.1f}", file=fh)
    print(f"\n  Range across all 6 cells: ~742 to ~745 (3-second spread)", file=fh)
    print(f"  Interpretation: A8 robust to threshold values within ±40-50% of defaults", file=fh)


def verify_admission_rate(results, fh):
    section("ADMISSION RATE — verifying \"~75% admission\" claim from Fig 5", fh)
    rates = []
    for seed in range(1, 11):
        df = load_trips(results, "A8", "S1", seed)
        if df is None:
            continue
        a = df["admitted"].astype(str).str.lower() == "true"
        d = df["denied"].astype(str).str.lower() == "true"
        decisions = a | d
        if decisions.sum() == 0:
            continue
        rate = a.sum() / decisions.sum()
        rates.append(rate)
        print(f"  A8 seed {seed}: admit_rate = {rate*100:.1f}%", file=fh)
    if rates:
        print(f"\n  A8 mean admission rate: {np.mean(rates)*100:.1f}%", file=fh)
        print(f"  Paper claim: ~75%", file=fh)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--results", default="results")
    ap.add_argument("--rou", default=".")
    ap.add_argument("--net", default="yanan_elevated.net.xml")
    ap.add_argument("--out", default="verification_report.txt")
    args = ap.parse_args()

    results = Path(args.results).resolve()
    rou_dir = Path(args.rou).resolve()
    net = Path(args.net).resolve()
    out = Path(args.out).resolve()

    with open(out, "w", encoding="utf-8") as fh:
        print("=" * 78, file=fh)
        print("LTFS Paper — Data Verification Report", file=fh)
        print(f"Generated: {datetime.now(timezone.utc).strftime('%Y-%m-%d %H:%M UTC')}", file=fh)
        print(f"Results dir: {results}", file=fh)
        print(f"Network:     {net}", file=fh)
        print("=" * 78, file=fh)
        print("", file=fh)
        print("This report recomputes every numerical claim in the paper from raw", file=fh)
        print("simulation CSVs. Discrepancies between the reported values and the", file=fh)
        print("recomputed values would indicate stale or incorrect numbers.", file=fh)
        print("", file=fh)

        verify_demand_counts(rou_dir, fh)
        verify_completion_rates(results, fh)
        verify_class_populations(results, fh)
        verify_network_topology(net, fh)
        verify_speed_claim(results, fh)
        verify_admission_rate(results, fh)
        verify_wmax_sweep(results, fh)
        verify_threshold_sweep(results, fh)

        verify_table_class_wise(results, "S1", results, 10,
                                 "Table II — Light demand S1, class-wise", fh)
        verify_table_class_wise(results, "S1", results / "corridor_pen", 10,
                                 "Table III — Corridor-pen S1, class-wise", fh)
        verify_table_class_wise(results, "S2", results, 10,
                                 "Table IV — S2 incident, class-wise", fh)

        verify_table_fleet(results, "S1", results, 10,
                            "Table V — Light demand S1, fleet-wide", fh)
        verify_table_fleet(results, "S1", results / "corridor_pen", 10,
                            "Table VI — Corridor-pen S1, fleet-wide", fh)
        verify_table_fleet(results, "S2", results, 10,
                            "Table VII — S2 incident, fleet-wide", fh)

        section("END OF REPORT", fh)

    print(f"Wrote {out}")
    # Also print a small summary
    print(open(out, encoding="utf-8").read()[:2000])


if __name__ == "__main__":
    main()