import csv, statistics, os, glob

print(f"{'File':40} {'n':>5} {'mean':>8} {'median':>8} {'p95':>8} {'sd':>7} {'cv%':>6}")
print("-" * 90)

for fp in sorted(glob.glob("results/pilot_*.csv")):
    name = os.path.basename(fp).replace("pilot_", "").replace(".csv", "")
    with open(fp) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        print(f"{name:40} EMPTY")
        continue
    tts = [float(r["travel_time"]) for r in rows]
    n = len(tts)
    mean = statistics.mean(tts)
    median = statistics.median(tts)
    p95 = sorted(tts)[int(0.95 * n)]
    sd = statistics.stdev(tts) if n > 1 else 0
    cv = sd / mean * 100 if mean else 0
    print(f"{name:40} {n:>5} {mean:>7.1f}s {median:>7.0f}s {p95:>7.0f}s {sd:>6.0f}s {cv:>5.1f}%")
