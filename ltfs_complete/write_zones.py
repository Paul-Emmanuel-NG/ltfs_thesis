import sys
sys.path.insert(0, ".")
import sumolib
from ltfs.net_utils import build_express_edges

n = sumolib.net.readNet("yanan_elevated.net.xml")
express = build_express_edges("yanan_elevated.net.xml", 16.7, 6.0, -5.0, 4)

xs = []
for eid in express:
    try:
        e = n.getEdge(eid)
        for x, y in e.getShape():
            xs.append(x)
    except:
        pass
center_x = (min(xs)+max(xs))/2

west_edges, east_edges = [], []
for e in n.getEdges():
    eid = e.getID()
    if eid in express or eid.startswith(":"):
        continue
    shape = e.getShape()
    if not shape:
        continue
    mid_x = sum(p[0] for p in shape) / len(shape)
    if not e.allows("passenger"):
        continue
    if mid_x < center_x - 500:
        west_edges.append(eid)
    elif mid_x > center_x + 500:
        east_edges.append(eid)

with open("west_edges.txt", "w") as f:
    f.write("\n".join(west_edges))
with open("east_edges.txt", "w") as f:
    f.write("\n".join(east_edges))

print(f"wrote west_edges.txt: {len(west_edges)} edges")
print(f"wrote east_edges.txt: {len(east_edges)} edges")
