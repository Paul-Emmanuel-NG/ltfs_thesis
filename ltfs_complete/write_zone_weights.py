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

# Write randomTrips weight files
# Format: <edges><edge id="..." value="..."/></edges>
with open("zones.src.xml", "w") as fsrc, open("zones.dst.xml", "w") as fdst:
    fsrc.write("<edgedata>\n  <interval begin='0' end='100000'>\n")
    fdst.write("<edgedata>\n  <interval begin='0' end='100000'>\n")
    n_west = n_east = 0
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
        # Origin weight (west = 1, else 0)
        if mid_x < center_x - 500:
            fsrc.write(f'    <edge id="{eid}" value="1.0"/>\n')
            n_west += 1
        # Destination weight (east = 1, else 0)
        if mid_x > center_x + 500:
            fdst.write(f'    <edge id="{eid}" value="1.0"/>\n')
            n_east += 1
    fsrc.write("  </interval>\n</edgedata>\n")
    fdst.write("  </interval>\n</edgedata>\n")

print(f"zones.src.xml: {n_west} west edges weighted 1.0")
print(f"zones.dst.xml: {n_east} east edges weighted 1.0")
