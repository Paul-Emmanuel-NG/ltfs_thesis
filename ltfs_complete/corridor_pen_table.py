import csv, statistics, glob, os, re
from collections import defaultdict

agg = defaultdict(list)
for fp in sorted(glob.glob("results/A?_S1_seed?_trips.csv")):
    name = os.path.basename(fp)
    m = re.match(r"(A\d)_S1_seed\d_trips\.csv", name)
    if not m: continue
    v = m.group(1)
    with open(fp) as f:
        rows = list(csv.DictReader(f))
    if not rows: continue
    tts = [float(r["travel_time"]) for r in rows]
    p95 = sorted(tts)[int(0.95 * len(tts))]
    median = statistics.median(tts)
    mean = statistics.mean(tts)
    veh_h = sum(tts) / 3600.0
    agg[v].append({
        "n": len(rows), "mean": mean, "median": median,
        "p95": p95, "veh_h": veh_h, "bi": (p95 - median) / median,
    })

print("Corridor-pen demand (3 seeds avg)")
print(f"{'Var':4} {'n':>5} {'mean_TT':>9} {'P95':>7} {'BI':>5} {'veh_h':>6}  {'vs_A0':>16}")
print("-" * 65)
a0 = agg.get("A0")
a0m = sum(r["mean"] for r in a0)/len(a0)
a0p = sum(r["p95"] for r in a0)/len(a0)
a0v = sum(r["veh_h"] for r in a0)/len(a0)
for vid in sorted(agg.keys()):
    runs = agg[vid]
    n = sum(r["n"] for r in runs)/len(runs)
    m_ = sum(r["mean"] for r in runs)/len(runs)
    p_ = sum(r["p95"] for r in runs)/len(runs)
    b_ = sum(r["bi"] for r in runs)/len(runs)
    v_ = sum(r["veh_h"] for r in runs)/len(runs)
    if vid != "A0":
        delta = f"TT{(m_-a0m)/a0m*100:+.1f}% VH{(v_-a0v)/a0v*100:+.1f}%"
    else:
        delta = "---"
    print(f"{vid:4} {n:>5.0f} {m_:>8.1f}s {p_:>6.0f}s {b_:>5.2f}  {v_:>5.1f}  {delta:>16}")
