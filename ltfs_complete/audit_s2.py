import os, glob, re
from collections import defaultdict
d = defaultdict(set)
for fp in glob.glob("results/A?_S2_seed*_trips.csv"):
    m = re.match(r"A(\d)_S2_seed(\d+)_trips\.csv", os.path.basename(fp))
    if m:
        d["A"+m.group(1)].add(int(m.group(2)))
for v in sorted(d.keys()):
    seeds = sorted(d[v])
    print(f"  {v}: {len(seeds)} seeds {seeds}")
