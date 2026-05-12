import sys
sys.path.insert(0, ".")
import sumolib

n = sumolib.net.readNet("yanan_elevated.net.xml")

# Get bounding box of the express layer
from ltfs.net_utils import build_express_edges
express = build_express_edges("yanan_elevated.net.xml", 16.7, 6.0, -5.0, 4)

xs, ys = [], []
for eid in express:
    try:
        e = n.getEdge(eid)
        for x, y in e.getShape():
            xs.append(x)
            ys.append(y)
    except:
        pass

print("Express layer bounding box:")
print(f"  X (east-west): {min(xs):.0f} to {max(xs):.0f}  (width {max(xs)-min(xs):.0f}m)")
print(f"  Y (north-south): {min(ys):.0f} to {max(ys):.0f}  (height {max(ys)-min(ys):.0f}m)")
print(f"  Center: ({(min(xs)+max(xs))/2:.0f}, {(min(ys)+max(ys))/2:.0f})")

# Now categorize all surface edges by position relative to corridor
# West of corridor center, East of corridor center
center_x = (min(xs)+max(xs))/2
west_edges = []
east_edges = []
for e in n.getEdges():
    eid = e.getID()
    if eid in express or eid.startswith(":"):
        continue
    # Use edge midpoint
    shape = e.getShape()
    if not shape:
        continue
    mid_x = sum(p[0] for p in shape) / len(shape)
    # Only consider passenger-passable edges
    if not e.allows("passenger"):
        continue
    if mid_x < center_x - 500:  # well west of center
        west_edges.append(eid)
    elif mid_x > center_x + 500:  # well east of center
        east_edges.append(eid)

print(f"\nWest zone (mid_x < center-500m): {len(west_edges)} edges")
print(f"East zone (mid_x > center+500m): {len(east_edges)} edges")
