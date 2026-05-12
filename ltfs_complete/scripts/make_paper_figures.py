"""
make_paper_figures.py — Regenerate all 5 paper figures from raw simulation CSVs.

This is the canonical script for paper figure generation. Each figure is
produced from raw trip CSVs in results/ with deterministic input lists
documented in the docstring. Re-running this script on the same CSVs
yields byte-identical PNGs (modulo matplotlib backend differences).

Inputs:
  results/{A0..A8}_S1_seed{1..10}_trips.csv         — light demand
  results/corridor_pen/{A0..A8}_S1_seed{1..9}_trips.csv  — corridor-pen
  results/{A0..A8}_S2_seed{1..10}_trips.csv         — S2 incident (n=10 cells)
  results/{A1,A3,A4,A6}_S2_seed{1..3}_trips.csv     — S2 incident (n=3 cells)
  yanan_elevated.net.xml                            — network for Fig 1

Outputs (in current directory):
  fig1_network_layout.png        — Fig 1: surface + express layout, junctions
  fig2_tt_distribution.png       — Fig 2: TT CDF + box plot, A1 vs A4 vs A8 light
  fig3_ablation_byregime.png     — Fig 3: mean TT + VH bars across 3 scenarios
  fig4_gate_dynamics.png         — Fig 4: A8 admit/deny over time, light demand
  fig5_classwise_byregime.png    — Fig 5: class-wise bars across 3 scenarios

Usage:
  python make_paper_figures.py [--results results/] [--out figures/]

Author: Akinwole Paul-Emmanuel Oreoluwa, Hohai University, 2026
"""
from __future__ import annotations
import argparse
import sys
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

VARIANTS = [f"A{i}" for i in range(9)]


# ----------------------------------------------------------------------------
# Data loading helpers
# ----------------------------------------------------------------------------

def load_trips(d: Path, variant: str, scenario: str, seed: int) -> pd.DataFrame | None:
    f = d / f"{variant}_{scenario}_seed{seed}_trips.csv"
    if not f.exists():
        return None
    return pd.read_csv(f)


def per_seed_aggregate(d: Path, variant: str, scenario: str, n_max: int = 10):
    """Aggregate per-seed stats: returns list of dicts."""
    rows = []
    for seed in range(1, n_max + 1):
        df = load_trips(d, variant, scenario, seed)
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
            "truck_mean": df.loc[df["class"] == "truck_truck", "travel_time"].mean()
                          if (df["class"] == "truck_truck").any() else np.nan,
            "passenger_mean": df.loc[df["class"] == "veh_passenger", "travel_time"].mean()
                              if (df["class"] == "veh_passenger").any() else np.nan,
        })
    return rows


def mean_ci(values):
    """Return (mean, half-width of 95% CI)."""
    arr = np.asarray([v for v in values if not (isinstance(v, float) and np.isnan(v))])
    n = len(arr)
    if n == 0:
        return (np.nan, np.nan)
    if n == 1:
        return (float(arr[0]), 0.0)
    m = arr.mean()
    sd = arr.std(ddof=1)
    return (float(m), float(1.96 * sd / np.sqrt(n)))


# ----------------------------------------------------------------------------
# Fig 1 — Network layout
# ----------------------------------------------------------------------------

def fig1_network_layout(net_xml: Path, out: Path):
    try:
        import sumolib
    except ImportError:
        print("[fig1] sumolib not available; skipping Fig 1")
        return
    print(f"[fig1] reading {net_xml}")
    net = sumolib.net.readNet(str(net_xml), withInternal=False)

    fig, ax = plt.subplots(figsize=(10, 5))
    surface_color = "#888888"
    express_color = "#cc3333"
    tunnel_color = "#3366cc"

    n_surface = n_express = n_tunnel = 0
    for edge in net.getEdges():
        try:
            shape = edge.getShape()
            if not shape or len(shape) < 2:
                continue
        except Exception:
            continue
        zs = []
        for lane in edge.getLanes():
            try:
                zs.extend([p[2] for p in lane.getShape3D()])
            except Exception:
                pass
        z_mean = np.mean(zs) if zs else 0.0
        speed = edge.getSpeed()

        xs = [p[0] for p in shape]
        ys = [p[1] for p in shape]
        if z_mean >= 5.0 and speed >= 16.6:  # elevated
            ax.plot(xs, ys, color=express_color, linewidth=1.2, zorder=3)
            n_express += 1
        elif z_mean <= -5.0 and speed >= 16.6:  # tunnel
            ax.plot(xs, ys, color=tunnel_color, linewidth=1.0, linestyle="--", zorder=2)
            n_tunnel += 1
        else:
            ax.plot(xs, ys, color=surface_color, linewidth=0.4, alpha=0.7, zorder=1)
            n_surface += 1

    n_tls = 0
    for node in net.getNodes():
        if node.getType() == "traffic_light":
            x, y = node.getCoord()
            ax.plot(x, y, "o", color="#ff8800", markersize=2.5, zorder=4)
            n_tls += 1

    ax.set_aspect("equal")
    ax.set_xlabel("Easting (m)")
    ax.set_ylabel("Northing (m)")
    ax.set_title(f"Yan'an Corridor subnetwork: {n_surface} surface edges, "
                 f"{n_express} elevated, {n_tunnel} tunnel, {n_tls} signalized")
    legend_elems = [
        Patch(facecolor=surface_color, label="Surface streets"),
        Patch(facecolor=express_color, label="Elevated express"),
        Patch(facecolor=tunnel_color, label="Tunnel express"),
        Patch(facecolor="#ff8800", label="Signalized junctions"),
    ]
    ax.legend(handles=legend_elems, loc="upper right", fontsize=8)
    plt.tight_layout()
    plt.savefig(out, dpi=200, bbox_inches="tight")
    plt.close()
    print(f"[fig1] wrote {out}")


# ----------------------------------------------------------------------------
# Fig 2 — TT distribution: A1 vs A4 vs A8 (light demand, all seeds pooled)
# ----------------------------------------------------------------------------

def fig2_tt_distribution(results: Path, out: Path):
    print("[fig2] loading A1, A4, A8 light demand trips...")
    pooled = {}
    for v in ["A1", "A4", "A8"]:
        all_tt = []
        for seed in range(1, 11):
            df = load_trips(results, v, "S1", seed)
            if df is not None:
                all_tt.extend(df["travel_time"].tolist())
        pooled[v] = np.asarray(all_tt)
        print(f"  {v}: pooled n={len(all_tt)}")

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    colors = {"A1": "#888888", "A4": "#1f77b4", "A8": "#d62728"}

    # CDF
    for v, tt in pooled.items():
        if len(tt) == 0:
            continue
        sorted_tt = np.sort(tt)
        cdf = np.arange(1, len(sorted_tt) + 1) / len(sorted_tt)
        axes[0].plot(sorted_tt, cdf, label=f"{v} (n={len(tt):,})",
                     color=colors[v], linewidth=2)
    axes[0].set_xlabel("Travel time (s)")
    axes[0].set_ylabel("Empirical CDF")
    axes[0].set_title("(a) Travel-time CDF, light demand S1")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    # Box plot
    box_data = [pooled[v] for v in ["A1", "A4", "A8"] if len(pooled[v]) > 0]
    box_labels = [v for v in ["A1", "A4", "A8"] if len(pooled[v]) > 0]
    bp = axes[1].boxplot(box_data, labels=box_labels, patch_artist=True,
                          showfliers=True, widths=0.6)
    for patch, v in zip(bp["boxes"], box_labels):
        patch.set_facecolor(colors[v])
        patch.set_alpha(0.6)
    axes[1].set_ylabel("Travel time (s)")
    axes[1].set_title("(b) Box plot, light demand S1")
    axes[1].grid(True, alpha=0.3, axis="y")

    plt.tight_layout()
    plt.savefig(out, dpi=200, bbox_inches="tight")
    plt.close()
    print(f"[fig2] wrote {out}")


# ----------------------------------------------------------------------------
# Fig 3 — Ablation by regime: mean TT and VH across 3 scenarios
# ----------------------------------------------------------------------------

def fig3_ablation_byregime(results: Path, out: Path):
    print("[fig3] computing per-variant aggregates across regimes...")
    scenarios = [
        ("Light demand S1", results, "S1", 10),
        ("Corridor-pen S1", results / "corridor_pen", "S1", 10),
        ("S2 incident", results, "S2", 10),
    ]

    fig, axes = plt.subplots(2, 3, figsize=(14, 7))

    for col, (title, base, scen, n_max) in enumerate(scenarios):
        means_tt, cis_tt = [], []
        means_vh, cis_vh = [], []
        ns = []
        for v in VARIANTS:
            rows = per_seed_aggregate(base, v, scen, n_max=n_max)
            ns.append(len(rows))
            mt, ct = mean_ci([r["mean_tt"] for r in rows])
            mv, cv = mean_ci([r["vh_total"] for r in rows])
            means_tt.append(mt)
            cis_tt.append(ct)
            means_vh.append(mv)
            cis_vh.append(cv)

        x = np.arange(len(VARIANTS))
        # Mean TT bar
        axes[0, col].bar(x, means_tt, yerr=cis_tt, color="#4472c4",
                          error_kw={"capsize": 3, "elinewidth": 1})
        # A1 horizontal line if A1 has data
        if not np.isnan(means_tt[1]):
            axes[0, col].axhline(means_tt[1], color="black", linestyle="--",
                                 linewidth=1, alpha=0.6, label="A1 (within-LTFS)")
            axes[0, col].legend(fontsize=8)
        axes[0, col].set_xticks(x)
        axes[0, col].set_xticklabels(VARIANTS, fontsize=9)
        axes[0, col].set_ylabel("Mean TT (s)")
        axes[0, col].set_title(title)
        axes[0, col].grid(True, alpha=0.3, axis="y")

        # VH bar
        axes[1, col].bar(x, means_vh, yerr=cis_vh, color="#70ad47",
                          error_kw={"capsize": 3, "elinewidth": 1})
        if not np.isnan(means_vh[1]):
            axes[1, col].axhline(means_vh[1], color="black", linestyle="--",
                                 linewidth=1, alpha=0.6, label="A1 (within-LTFS)")
        axes[1, col].set_xticks(x)
        axes[1, col].set_xticklabels(VARIANTS, fontsize=9)
        axes[1, col].set_ylabel("Vehicle-hours")
        axes[1, col].grid(True, alpha=0.3, axis="y")

        # n labels
        for i, n in enumerate(ns):
            axes[0, col].text(i, axes[0, col].get_ylim()[0] * 1.005, f"n={n}",
                              ha="center", va="bottom", fontsize=7, color="gray")

    plt.tight_layout()
    plt.savefig(out, dpi=200, bbox_inches="tight")
    plt.close()
    print(f"[fig3] wrote {out}")


# ----------------------------------------------------------------------------
# Fig 4 — Gate dynamics: A8 light demand, admit/deny over time + admission rate
# ----------------------------------------------------------------------------

def fig4_gate_dynamics(results: Path, out: Path):
    print("[fig4] loading A8 light demand seeds...")
    all_dfs = []
    for seed in range(1, 11):
        df = load_trips(results, "A8", "S1", seed)
        if df is None:
            continue
        all_dfs.append(df)
    if not all_dfs:
        print("[fig4] no A8 light data found")
        return

    pooled = pd.concat(all_dfs, ignore_index=True)
    pooled["admitted_b"] = pooled["admitted"].astype(str).str.lower() == "true"
    pooled["denied_b"] = pooled["denied"].astype(str).str.lower() == "true"
    print(f"  pooled n={len(pooled)} trips, n_seeds={len(all_dfs)}")

    bin_edges = np.arange(900, 4501, 300)
    bin_centers = 0.5 * (bin_edges[:-1] + bin_edges[1:])

    admit_counts = np.zeros(len(bin_centers))
    deny_counts = np.zeros(len(bin_centers))
    for i, (lo, hi) in enumerate(zip(bin_edges[:-1], bin_edges[1:])):
        mask = (pooled["depart"] >= lo) & (pooled["depart"] < hi)
        admit_counts[i] = pooled.loc[mask, "admitted_b"].sum()
        deny_counts[i] = pooled.loc[mask, "denied_b"].sum()

    decisions = admit_counts + deny_counts
    admit_rate = np.where(decisions > 0, admit_counts / decisions, np.nan)

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    width = 250
    axes[0].bar(bin_centers - width / 2, admit_counts, width=width,
                 color="#2ca02c", label="Admitted")
    axes[0].bar(bin_centers + width / 2, deny_counts, width=width,
                 color="#d62728", label="Denied")
    axes[0].set_xlabel("Trip departure time (s)")
    axes[0].set_ylabel("Decisions per 300s bin (pooled across seeds)")
    axes[0].set_title("(a) Gate decisions over time, A8 light demand")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3, axis="y")

    axes[1].plot(bin_centers, admit_rate * 100, "o-", color="#2ca02c", linewidth=2)
    axes[1].axhline(75, color="black", linestyle="--", linewidth=1, alpha=0.5,
                     label="~75% reference")
    axes[1].set_xlabel("Trip departure time (s)")
    axes[1].set_ylabel("Admission rate (%)")
    axes[1].set_ylim(0, 100)
    axes[1].set_title("(b) Admission rate over time")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(out, dpi=200, bbox_inches="tight")
    plt.close()
    print(f"[fig4] wrote {out}")


# ----------------------------------------------------------------------------
# Fig 5 — Class-wise by regime: bus / truck / passenger across scenarios
# ----------------------------------------------------------------------------

def fig5_classwise_byregime(results: Path, out: Path):
    print("[fig5] computing class-wise aggregates...")
    scenarios = [
        ("Light demand S1", results, "S1", 10),
        ("Corridor-pen S1", results / "corridor_pen", "S1", 10),
        ("S2 incident", results, "S2", 10),
    ]

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5), sharey=True)

    for col, (title, base, scen, n_max) in enumerate(scenarios):
        bus, truck, passenger = [], [], []
        bus_ci, truck_ci, passenger_ci = [], [], []
        for v in VARIANTS:
            rows = per_seed_aggregate(base, v, scen, n_max=n_max)
            mb, cb = mean_ci([r["bus_mean"] for r in rows])
            mt, ct = mean_ci([r["truck_mean"] for r in rows])
            mp, cp = mean_ci([r["passenger_mean"] for r in rows])
            bus.append(mb); bus_ci.append(cb)
            truck.append(mt); truck_ci.append(ct)
            passenger.append(mp); passenger_ci.append(cp)

        x = np.arange(len(VARIANTS))
        w = 0.27
        axes[col].bar(x - w, bus, width=w, yerr=bus_ci, label="Bus",
                       color="#1f77b4", error_kw={"capsize": 2})
        axes[col].bar(x, truck, width=w, yerr=truck_ci, label="Truck",
                       color="#ff7f0e", error_kw={"capsize": 2})
        axes[col].bar(x + w, passenger, width=w, yerr=passenger_ci, label="Passenger",
                       color="#2ca02c", error_kw={"capsize": 2})
        axes[col].set_xticks(x)
        axes[col].set_xticklabels(VARIANTS, fontsize=9)
        axes[col].set_title(title)
        axes[col].grid(True, alpha=0.3, axis="y")
        if col == 0:
            axes[col].set_ylabel("Mean travel time (s)")
        if col == 2:
            axes[col].legend(loc="upper right", fontsize=9)

    plt.tight_layout()
    plt.savefig(out, dpi=200, bbox_inches="tight")
    plt.close()
    print(f"[fig5] wrote {out}")


# ----------------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--results", default="results", help="Results dir with trip CSVs")
    ap.add_argument("--out", default=".", help="Output dir for PNGs")
    ap.add_argument("--net", default="yanan_elevated.net.xml",
                    help="Network XML (for Fig 1)")
    ap.add_argument("--skip-fig1", action="store_true",
                    help="Skip Fig 1 (network layout, requires sumolib)")
    args = ap.parse_args()

    results = Path(args.results).resolve()
    out_dir = Path(args.out).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    net = Path(args.net).resolve()

    print(f"results dir: {results}")
    print(f"out dir:     {out_dir}")
    print()

    if not args.skip_fig1:
        fig1_network_layout(net, out_dir / "fig1_network_layout.png")
    fig2_tt_distribution(results, out_dir / "fig2_tt_distribution.png")
    fig3_ablation_byregime(results, out_dir / "fig3_ablation_byregime.png")
    fig4_gate_dynamics(results, out_dir / "fig4_gate_dynamics.png")
    fig5_classwise_byregime(results, out_dir / "fig5_classwise_byregime.png")
    print("\n[done]")


if __name__ == "__main__":
    main()
