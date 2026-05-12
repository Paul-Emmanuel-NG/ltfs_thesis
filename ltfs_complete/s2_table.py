import csv, statistics, glob, os, re
from collections import defaultdict

# Aggregate S2 across variants; only one seed per variant for S2
results = {}
for fp in sorted(glob.glob("results/A?_S2_seed*_trips.csv")):
    name = os.path.basename(fp)
    m = re.match(r"(A\d)_S2_seed\d_trips\.csv", name)
    if not m:
        continue
    variant = m.group(1)
    with open(fp) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        continue
    tts = [float(r["travel_time"]) for r in rows]
    veh_h = sum(tts) / 3600.0
    p95 = sorted(tts)[int(0.95 * len(tts))]
    median = statistics.median(tts)
    mean = statistics.mean(tts)
    results[variant] = {
        "n": len(rows),
        "mean_tt": mean,
        "median_tt": median,
        "p95_tt": p95,
        "veh_h": veh_h,
        "buffer_idx": (p95 - median) / median,
    }

# Print as a table
print("S2 (Incident scenario) — single seed, per variant")
print(f"{'Var':4} {'n':>5} {'mean_tt':>9} {'median':>8} {'p95':>8} {'veh_h':>7} {'BI':>5}")
print("-" * 50)
a0 = results.get("A0", {})
for vid in sorted(results.keys()):
    r = results[vid]
    delta_pct = ""
    if a0 and vid != "A0":
        d = (r["mean_tt"] - a0["mean_tt"]) / a0["mean_tt"] * 100
        delta_pct = f" ({d:+.1f}%)"
    print(f"{vid:4} {r['n']:>5} {r['mean_tt']:>8.1f}s{delta_pct:>10} {r['median_tt']:>7.0f}s {r['p95_tt']:>7.0f}s {r['veh_h']:>6.1f} {r['buffer_idx']:>5.2f}")
