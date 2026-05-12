import sys
sys.path.insert(0, ".")
from ltfs.net_utils import build_express_edges
import sumolib

express = build_express_edges("yanan_elevated.net.xml", 16.7, 6.0, -5.0, 4)
n = sumolib.net.readNet("yanan_elevated.net.xml")
ranked = []
for eid in express:
    try:
        e = n.getEdge(eid)
        ranked.append((e.getLength(), eid))
    except:
        pass
ranked.sort(reverse=True)
print("Top 10 longest express edges (candidates for --via):")
for length, eid in ranked[:10]:
    print(f"  {length:.0f}m  {eid}")
