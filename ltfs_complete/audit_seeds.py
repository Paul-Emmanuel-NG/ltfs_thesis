import os, glob, re
from collections import defaultdict

def scan(folder, scenario):
    seeds_by_variant = defaultdict(set)
    for fp in glob.glob(folder + f"/A?_{scenario}_seed*_trips.csv"):
        m = re.match(rf"A(\d)_{scenario}_seed(\d+)_trips\.csv", os.path.basename(fp))
        if m:
            seeds_by_variant["A"+m.group(1)].add(int(m.group(2)))
    return seeds_by_variant

print("=== corridor_pen S1 (results/corridor_pen/) ===")
d = scan("results/corridor_pen", "S1")
for v in sorted(d.keys()):
    seeds = sorted(d[v])
    missing = [s for s in range(1,11) if s not in seeds]
    print(f"  {v}: seeds {seeds}  missing: {missing}")

print()
print("=== light S1 (results/) ===")
d = scan("results", "S1")
for v in sorted(d.keys()):
    seeds = sorted(d[v])
    missing = [s for s in range(1,11) if s not in seeds]
    print(f"  {v}: seeds {seeds}  missing: {missing}")

print()
print("=== light S2 (results/) ===")
d = scan("results", "S2")
for v in sorted(d.keys()):
    seeds = sorted(d[v])
    missing = [s for s in range(1,11) if s not in seeds]
    print(f"  {v}: seeds {seeds}  missing: {missing}")
