import csv, statistics, glob, os
print(f"{'File':35} {'n':>5} {'mean_TT':>8} {'mean_dist':>10} {'speed':>9} {'p95':>8}")
print("-" * 80)
for fp in sorted(glob.glob("results/calib_*.csv")) + sorted(glob.glob("results/pilot_*corrpen_3600*.csv")):
    name = os.path.basename(fp).replace(".csv","")
    with open(fp) as f:
        rows = list(csv.DictReader(f))
    if not rows: continue
    tts = [float(r["travel_time"]) for r in rows]
    dists = [float(r["distance_m"]) for r in rows if r.get("distance_m")]
    n = len(tts)
    mean_tt = statistics.mean(tts)
    mean_d = statistics.mean(dists) if dists else float("nan")
    speed_kmh = (mean_d / 1000) / (mean_tt / 3600)
    p95 = sorted(tts)[int(0.95*n)]
    print(f"{name:35} {n:>5} {mean_tt:>7.0f}s {mean_d:>8.0f}m {speed_kmh:>7.1f}km/h {p95:>7.0f}s")
