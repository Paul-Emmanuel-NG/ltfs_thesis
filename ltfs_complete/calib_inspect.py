import csv, statistics, glob, os
print(f"{'File':35} {'n':>5} {'mean_TT':>8} {'p95':>8}")
print("-" * 60)
for fp in sorted(glob.glob("results/calib_*.csv")) + sorted(glob.glob("results/pilot_*corrpen_3600*.csv")):
    name = os.path.basename(fp).replace(".csv","")
    with open(fp) as f:
        reader = csv.DictReader(f)
        cols = reader.fieldnames
        rows = list(reader)
    if not rows: continue
    tts = [float(r["travel_time"]) for r in rows]
    n = len(tts)
    mean = statistics.mean(tts)
    p95 = sorted(tts)[int(0.95*n)]
    print(f"{name:35} {n:>5} {mean:>7.0f}s {p95:>7.0f}s")
    if name == sorted([os.path.basename(f).replace(".csv","") for f in glob.glob("results/calib_*.csv")])[0]:
        print(f"  (cols: {cols})")
