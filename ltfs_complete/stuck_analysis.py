import os
import xml.etree.ElementTree as ET
from collections import Counter

path = "tripinfos_light.xml"
if not os.path.exists(path):
    print("no tripinfos_light.xml found — rerun sim first")
    raise SystemExit(0)

tree = ET.parse(path)
stuck_edges = Counter()
arrival_edges = Counter()
n_total = 0
n_stuck = 0
for ti in tree.getroot().findall("tripinfo"):
    n_total += 1
    wt = float(ti.get("waitingTime", "0"))
    lane = ti.get("arrivalLane", "")
    edge = lane.rsplit("_", 1)[0] if lane else ""
    arrival_edges[edge] += 1
    if wt > 500:
        n_stuck += 1
        stuck_edges[edge] += 1

print("total tripinfos:", n_total)
print("stuck (waitingTime > 500s):", n_stuck)
print()
print("Top 15 edges where stuck vehicles ended up:")
for e, n in stuck_edges.most_common(15):
    print("  " + str(n) + "  " + str(e))
print()
print("Top 15 arrival edges overall (for reference):")
for e, n in arrival_edges.most_common(15):
    print("  " + str(n) + "  " + str(e))
