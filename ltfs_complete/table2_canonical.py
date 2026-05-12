import csv, statistics, glob, os, re
from collections import defaultdict

# Canonical Table II: aggregate S1 across 3 seeds per variant
agg = defaultdict(list)
for fp in sorted(glob.glob("results/A?_S1_seed?_trips.csv")):
    name = os.path.basename(fp)
    m = re.match(r"(A\d)_S1_seed\d_trips\.csv", name)
    if not m:
        continue
    variant = m.group(1)
    with open(fp) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        continue
    tts = [float(r["travel_time"]) for r in rows]
    p95 = sorted(tts)[int(0.95 * len(tts))]
    median = statistics.median(tts)
    mean = statistics.mean(tts)
    veh_h = sum(tts) / 3600.0
    agg[variant].append({
        "n": len(rows),
        "mean": mean,
        "median": median,
        "p95": p95,
        "veh_h": veh_h,
        "bi": (p95 - median) / median,
    })

print("Table II (S1, mean across 3 seeds)")
print(f"{'Var':4} {'n_avg':>6} {'mean_TT':>9} {'median':>8} {'p95_TT':>8} {'BI':>6} {'veh_h':>7}")
print("-" * 60)
a0 = agg.get("A0")
a0_mean = sum(r["mean"] for r in a0)/len(a0) if a0 else 0
a0_p95 = sum(r["p95"] for r in a0)/len(a0) if a0 else 0
a0_vh = sum(r["veh_h"] for r in a0)/len(a0) if a0 else 0
for vid in sorted(agg.keys()):
    runs = agg[vid]
    n_avg = sum(r["n"] for r in runs)/len(runs)
    mean_avg = sum(r["mean"] for r in runs)/len(runs)
    median_avg = sum(r["median"] for r in runs)/len(runs)
    p95_avg = sum(r["p95"] for r in runs)/len(runs)
    bi_avg = sum(r["bi"] for r in runs)/len(runs)
    vh_avg = sum(r["veh_h"] for r in runs)/len(runs)
    delta_mean = (mean_avg - a0_mean) / a0_mean * 100 if a0_mean else 0
    delta_p95 = (p95_avg - a0_p95) / a0_p95 * 100 if a0_p95 else 0
    delta_vh = (vh_avg - a0_vh) / a0_vh * 100 if a0_vh else 0
    marker = ""
    if vid != "A0":
        marker = f" [TT{delta_mean:+.1f}% P95{delta_p95:+.1f}% VH{delta_vh:+.1f}%]"
    print(f"{vid:4} {n_avg:>6.0f} {mean_avg:>8.1f}s {median_avg:>7.0f}s {p95_avg:>7.0f}s {bi_avg:>5.2f}  {vh_avg:>5.1f}{marker}")
