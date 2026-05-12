import csv, statistics, glob, os, re, math
from collections import defaultdict

def aggregate(folder, scenario):
    agg = defaultdict(list)
    pattern = os.path.join(folder, f"A?_{scenario}_seed*_trips.csv")
    for fp in sorted(glob.glob(pattern)):
        m = re.match(rf"A(\d)_{scenario}_seed(\d+)_trips\.csv", os.path.basename(fp))
        if not m: continue
        v = "A" + m.group(1)
        with open(fp) as f:
            rows = list(csv.DictReader(f))
        if not rows: continue
        tts = [float(r["travel_time"]) for r in rows]
        agg[v].append({"mean": statistics.mean(tts), "veh_h": sum(tts)/3600.0})
    return agg

def stats(values):
    n = len(values); m = statistics.mean(values)
    if n < 2: return m, 0
    sd = statistics.stdev(values); ci = 1.96 * sd / math.sqrt(n)
    return m, ci

for label, folder, scen in [("Light S1", "results", "S1"),
                            ("Corridor-pen S1", "results/corridor_pen", "S1"),
                            ("Light S2", "results", "S2")]:
    print(f"\n=== {label}: each variant vs A1 (within-LTFS control) ===")
    agg = aggregate(folder, scen)
    a1 = agg.get("A1", [])
    if not a1:
        print("  no A1 data"); continue
    a1m, a1m_ci = stats([r["mean"] for r in a1])
    a1v, a1v_ci = stats([r["veh_h"] for r in a1])
    print(f"  A1 baseline: meanTT = {a1m:.1f}±{a1m_ci:.1f}s   VH = {a1v:.2f}±{a1v_ci:.2f}")
    for v in sorted(agg.keys()):
        if v in ("A0", "A1"): continue
        runs = agg[v]
        if not runs: continue
        m, m_ci = stats([r["mean"] for r in runs])
        vh, vh_ci = stats([r["veh_h"] for r in runs])
        tt_overlap = (m + m_ci >= a1m - a1m_ci) and (m - m_ci <= a1m + a1m_ci)
        vh_overlap = (vh + vh_ci >= a1v - a1v_ci) and (vh - vh_ci <= a1v + a1v_ci)
        tt_pct = (m - a1m) / a1m * 100
        vh_pct = (vh - a1v) / a1v * 100
        tt_lab = "TREND" if tt_overlap else "SIG  "
        vh_lab = "TREND" if vh_overlap else "SIG  "
        print(f"  {v} vs A1: TT {tt_pct:+5.1f}% [{tt_lab}]   VH {vh_pct:+5.1f}% [{vh_lab}]")
