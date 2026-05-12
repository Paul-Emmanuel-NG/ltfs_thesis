import csv, statistics, glob, os, re, math
from collections import defaultdict

def aggregate_by_class(folder, scenario):
    """Returns {variant: list of per-seed dicts with class breakdowns}"""
    agg = defaultdict(list)
    pattern = os.path.join(folder, f"A?_{scenario}_seed*_trips.csv")
    for fp in sorted(glob.glob(pattern)):
        m = re.match(rf"A(\d)_{scenario}_seed(\d+)_trips\.csv", os.path.basename(fp))
        if not m: continue
        v = "A" + m.group(1)
        with open(fp) as f:
            rows = list(csv.DictReader(f))
        if not rows: continue
        by_class = defaultdict(list)
        for r in rows:
            by_class[r["class"]].append(float(r["travel_time"]))
        rec = {}
        for c, tts in by_class.items():
            rec[c] = {
                "n": len(tts),
                "mean": statistics.mean(tts),
                "veh_h": sum(tts)/3600.0,
            }
        agg[v].append(rec)
    return agg

def stats(values):
    n = len(values)
    if n < 1: return float("nan"), float("nan")
    m = statistics.mean(values)
    if n < 2: return m, 0.0
    sd = statistics.stdev(values)
    return m, 1.96 * sd / math.sqrt(n)

def report_class(agg, label, baseline="A1"):
    print()
    print("=" * 90)
    print(f"  {label}: per-class mean TT vs {baseline}")
    print("=" * 90)
    classes = ["bus_bus", "truck_truck", "veh_passenger"]
    base_runs = agg.get(baseline, [])
    if not base_runs:
        print(f"  no {baseline} data"); return
    base_means = {}
    for c in classes:
        vals = [r[c]["mean"] for r in base_runs if c in r]
        base_means[c] = stats(vals)
    print(f"  {baseline} baseline class means:")
    for c in classes:
        m, ci = base_means[c]
        print(f"    {c}: {m:.1f}±{ci:.1f}s")
    print()
    print(f"  {'Variant':10} {'bus':>20} {'truck':>20} {'passenger':>20}")
    for vid in sorted(agg.keys()):
        if vid == baseline: continue
        runs = agg[vid]
        if not runs: continue
        line = f"  {vid:10}"
        for c in classes:
            vals = [r[c]["mean"] for r in runs if c in r]
            m, ci = stats(vals)
            base_m, base_ci = base_means[c]
            pct = (m - base_m) / base_m * 100 if base_m else 0
            overlap = (m + ci >= base_m - base_ci) and (m - ci <= base_m + base_ci)
            tag = "TR" if overlap else "**"
            line += f"  {pct:+5.1f}%[{tag}] (n{len(vals)})"
        print(line)

for label, folder, scen in [("Light S1", "results", "S1"),
                            ("Corridor-pen S1", "results/corridor_pen", "S1"),
                            ("Light S2", "results", "S2")]:
    agg = aggregate_by_class(folder, scen)
    report_class(agg, label, baseline="A1")
