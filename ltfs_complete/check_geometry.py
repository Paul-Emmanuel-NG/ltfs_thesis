import sys
sys.path.insert(0, ".")
import sumolib
from ltfs.net_utils import build_express_edges

n = sumolib.net.readNet("yanan_elevated.net.xml")
express = build_express_edges("yanan_elevated.net.xml", 16.7, 6.0, -5.0, 4)

# Get express y range
ys = []
for eid in express:
    try:
        e = n.getEdge(eid)
        for x, y in e.getShape():
            ys.append(y)
    except:
        pass
y_min, y_max = min(ys), max(ys)
print(f"Express layer Y range: {y_min:.0f} to {y_max:.0f}")

# Count surface edges by Y position relative to express
above = 0  # north of express
within = 0  # at express level
below = 0  # south of express
for e in n.getEdges():
    eid = e.getID()
    if eid in express or eid.startswith(":"):
        continue
    if not e.allows("passenger"):
        continue
    shape = e.getShape()
    if not shape:
        continue
    mid_y = sum(p[1] for p in shape) / len(shape)
    if mid_y > y_max + 100:
        above += 1
    elif mid_y < y_min - 100:
        below += 1
    else:
        within += 1

print(f"Surface edges north of express: {above}")
print(f"Surface edges within express y-band: {within}")
print(f"Surface edges south of express: {below}")
