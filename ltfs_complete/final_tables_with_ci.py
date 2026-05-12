import csv, statistics, glob, os, re, math
from collections import defaultdict

def load_freeflow(path):
    if not os.path.exists(path):
        return {}
    fmap = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            fmap[(r["routeKey"], r["class"])] = float(r["TT_freeflow"])
    return fmap

FF_MAP = {
    "A0": "freeflow_MP_nogate.csv",
    "A1": "freeflow_MP_nogate.csv",
    "A2": "freeflow_MP_TST.csv",
    "A3": "freeflow_MP_UWA.csv",
    "A4": "freeflow_MP_UWA.csv",
    "A5": "freeflow_MP_UWA.csv",
    "A6": "freeflow_PWMP_UWA.csv",
    "A7": "freeflow_PWMP_UWA.csv",
    "A8": "freeflow_PWMP_UWA.csv",
}

def aggregate(folder, scenario, ff_dir):
    """Return {variant: list of per-seed dicts}"""
    ff_cache = {}
    for v, p in FF_MAP.items():
        full = os.path.join(ff_dir, p)
        if full not in ff_cache:
            ff_cache[full] = load_freeflow(full)

    agg = defaultdict(list)
    pattern = os.path.join(folder, f"A?_{scenario}_seed*_trips.csv")
    for fp in sorted(glob.glob(pattern)):
        m = re.match(rf"A(\d)_{scenario}_seed(\d+)_trips\.csv", os.path.basename(fp))
        if not m:
            continue
        v = "A" + m.group(1)
        ff = ff_cache[os.path.join(ff_dir, FF_MAP[v])]
        with open(fp) as f:
            rows = list(csv.DictReader(f))
        if not rows:
            continue
        tts = [float(r["travel_time"]) for r in rows]
        wts = [float(r["travel_time"]) - ff[(r["routeKey"], r["class"])]
               for r in rows if (r["routeKey"], r["class"]) in ff]
        p95 = sorted(tts)[int(0.95 * len(tts))]
        median = statistics.median(tts)
        mean = statistics.mean(tts)
        veh_h = sum(tts) / 3600.0
        agg[v].append({
            "n": len(rows), "mean": mean, "median": median, "p95": p95,
            "veh_h": veh_h, "bi": (p95 - median) / median,
            "wt_mean": statistics.mean(wts) if wts else float("nan"),
            "wt_p95": (sorted(wts)[int(0.95*len(wts))] if wts else float("nan")),
        })
    return agg

def stats_with_ci(values):
    """Return (mean, sd, se, ci95) for a list of values."""
    n = len(values)
    if n < 1:
        return float("nan"), float("nan"), float("nan"), float("nan")
    m = statistics.mean(values)
    if n < 2:
        return m, 0.0, 0.0, 0.0
    sd = statistics.stdev(values)
    se = sd / math.sqrt(n)
    ci = 1.96 * se
    return m, sd, se, ci

def report(agg, label):
    print()
    print("=" * 100)
    print(f"  {label}  (n_seeds per cell shown)")
    print("=" * 100)
    print(f"{'Var':4} {'n_seeds':>7} {'meanTT±CI':>17} {'P95±CI':>15} {'VehH±CI':>14} {'meanWT±CI':>16}")
    print("-" * 100)

    a0_runs = agg.get("A0", [])
    a0_means = [r["mean"] for r in a0_runs]
    a0_vh = [r["veh_h"] for r in a0_runs]
    a0_m, _, _, a0_ci = stats_with_ci(a0_means)
    a0_v, _, _, a0_v_ci = stats_with_ci(a0_vh)

    rows_for_sig = []

    for vid in sorted(agg.keys()):
        runs = agg[vid]
        n = len(runs)
        m_, _, _, m_ci = stats_with_ci([r["mean"] for r in runs])
        p_, _, _, p_ci = stats_with_ci([r["p95"] for r in runs])
        v_, _, _, v_ci = stats_with_ci([r["veh_h"] for r in runs])
        wm, _, _, wm_ci = stats_with_ci([r["wt_mean"] for r in runs])
        print(f"{vid:4} {n:>7} {m_:>8.1f}±{m_ci:>5.1f}s {p_:>7.0f}±{p_ci:>4.0f}s {v_:>5.1f}±{v_ci:>4.1f} {wm:>8.1f}±{wm_ci:>4.1f}s")
        rows_for_sig.append((vid, m_, m_ci, v_, v_ci))

    print()
    print(f"  --- Significance check (vs A0): mean ± CI overlap with A0 mean ± CI ---")
    print(f"  A0: mean TT = {a0_m:.1f} ± {a0_ci:.1f}s   VH = {a0_v:.2f} ± {a0_v_ci:.2f}")
    for vid, m_, m_ci, v_, v_ci in rows_for_sig:
        if vid == "A0":
            continue
        # CI overlap test for mean TT
        tt_overlap = (m_ + m_ci >= a0_m - a0_ci) and (m_ - m_ci <= a0_m + a0_ci)
        vh_overlap = (v_ + v_ci >= a0_v - a0_v_ci) and (v_ - v_ci <= a0_v + a0_v_ci)
        tt_pct = (m_ - a0_m) / a0_m * 100
        vh_pct = (v_ - a0_v) / a0_v * 100
        tt_label = "TREND" if tt_overlap else "SIG  "
        vh_label = "TREND" if vh_overlap else "SIG  "
        print(f"  {vid}: TT {tt_pct:+5.1f}% [{tt_label}]   VH {vh_pct:+5.1f}% [{vh_label}]")

# === Light demand S1 ===
agg = aggregate("results", "S1", "results/light_freeflow_v2" if os.path.exists("results/light_freeflow_v2/freeflow_MP_nogate.csv") else "results")
report(agg, "Light demand S1 (10 seeds expected)")

# === Light demand S2 ===
agg = aggregate("results", "S2", "results/light_freeflow_v2" if os.path.exists("results/light_freeflow_v2/freeflow_MP_nogate.csv") else "results")
report(agg, "Light demand S2 incident (10 seeds expected on A0/A2/A5/A7/A8)")

# === Corridor-pen S1 ===
agg = aggregate("results/corridor_pen", "S1", "results/corridor_pen")
report(agg, "Corridor-pen S1 (9 seeds expected)")
