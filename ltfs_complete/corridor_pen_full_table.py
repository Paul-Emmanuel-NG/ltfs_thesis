import csv, statistics, glob, os, re
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
    "A0": "results/freeflow_MP_nogate.csv",
    "A1": "results/freeflow_MP_nogate.csv",
    "A2": "results/freeflow_MP_TST.csv",
    "A3": "results/freeflow_MP_UWA.csv",
    "A4": "results/freeflow_MP_UWA.csv",
    "A5": "results/freeflow_MP_UWA.csv",
    "A6": "results/freeflow_PWMP_UWA.csv",
    "A7": "results/freeflow_PWMP_UWA.csv",
    "A8": "results/freeflow_PWMP_UWA.csv",
}
ff_cache = {}
for v, p in FF_MAP.items():
    if p not in ff_cache:
        ff_cache[p] = load_freeflow(p)

agg = defaultdict(list)
for fp in sorted(glob.glob("results/A?_S1_seed?_trips.csv")):
    name = os.path.basename(fp)
    m = re.match(r"(A\d)_S1_seed\d_trips\.csv", name)
    if not m: continue
    v = m.group(1)
    ff = ff_cache.get(FF_MAP[v], {})
    with open(fp) as f:
        rows = list(csv.DictReader(f))
    if not rows: continue
    tts = [float(r["travel_time"]) for r in rows]
    wts = []
    for r in rows:
        key = (r["routeKey"], r["class"])
        if key in ff:
            wts.append(float(r["travel_time"]) - ff[key])
    p95 = sorted(tts)[int(0.95 * len(tts))]
    median = statistics.median(tts)
    mean = statistics.mean(tts)
    veh_h = sum(tts) / 3600.0
    wt_mean = statistics.mean(wts) if wts else float("nan")
    wt_p95 = sorted(wts)[int(0.95*len(wts))] if wts else float("nan")
    agg[v].append({
        "n": len(rows), "mean": mean, "median": median, "p95": p95,
        "veh_h": veh_h, "bi": (p95-median)/median,
        "wt_mean": wt_mean, "wt_p95": wt_p95, "n_wt": len(wts),
    })

print("Corridor-pen demand (S1, mean over 3 seeds, with WT)")
print(f"{'Var':4} {'n':>4} {'mean_TT':>9} {'P95_TT':>8} {'BI':>5} {'veh_h':>6} {'mean_WT':>9} {'P95_WT':>8}  {'vs_A0':>20}")
print("-" * 95)
a0 = agg["A0"]
a0m = sum(r["mean"] for r in a0)/len(a0)
a0v = sum(r["veh_h"] for r in a0)/len(a0)
for vid in sorted(agg.keys()):
    runs = agg[vid]
    n = sum(r["n"] for r in runs)/len(runs)
    m_ = sum(r["mean"] for r in runs)/len(runs)
    p_ = sum(r["p95"] for r in runs)/len(runs)
    b_ = sum(r["bi"] for r in runs)/len(runs)
    v_ = sum(r["veh_h"] for r in runs)/len(runs)
    wm = sum(r["wt_mean"] for r in runs)/len(runs)
    wp = sum(r["wt_p95"] for r in runs)/len(runs)
    if vid != "A0":
        d = f"TT{(m_-a0m)/a0m*100:+.1f}% VH{(v_-a0v)/a0v*100:+.1f}%"
    else:
        d = "---"
    print(f"{vid:4} {n:>4.0f} {m_:>8.1f}s {p_:>7.0f}s {b_:>5.2f}  {v_:>5.1f}  {wm:>8.1f}s {wp:>7.0f}s  {d:>20}")
