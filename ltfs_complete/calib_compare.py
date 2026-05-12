import csv, statistics, glob, os

print(f"{'File':35} {'n':>5} {'mean_TT':>8} {'route_km':>9} {'speed':>8} {'p95':>8}")
print("-" * 80)
for fp in sorted(glob.glob("results/calib_*.csv")) + sorted(glob.glob("results/pilot_*3600*.csv")):
    name = os.path.basename(fp).replace(".csv","")
    with open(fp) as f:
        rows = list(csv.DictReader(f))
    if not rows: continue
    tts = [float(r["travel_time"]) for r in rows]
    # Try to get route length if column exists
    try:
        route_lens = [float(r["route_length"]) for r in rows]
        mean_len = statistics.mean(route_lens) / 1000
        speed_kmh = mean_len / (statistics.mean(tts)/3600)
    except (KeyError, ValueError):
        mean_len = float("nan"); speed_kmh = float("nan")
    n = len(tts)
    mean = statistics.mean(tts)
    p95 = sorted(tts)[int(0.95*n)]
    print(f"{name:35} {n:>5} {mean:>7.0f}s {mean_len:>8.2f}km {speed_kmh:>6.1f}km/h {p95:>7.0f}s")
