import sumolib
import sys
sys.path.insert(0, ".")
from ltfs.net_utils import build_express_edges, build_gate_edges
import statistics

express = build_express_edges("yanan_elevated.net.xml", 16.7, 6.0, -5.0, 4)
gates = build_gate_edges("yanan_elevated.net.xml", express)

n = sumolib.net.readNet("yanan_elevated.net.xml")

express_z = []
surface_z = []
gate_z = []
for e in n.getEdges():
    eid = e.getID()
    if eid.startswith(":"): continue
    shape = e.getShape3D()
    if not shape: continue
    z_mean = statistics.mean(z for x,y,z in shape)
    if eid in express:
        express_z.append(z_mean)
    elif eid in gates:
        gate_z.append(z_mean)
    else:
        surface_z.append(z_mean)

def summarize(vals, label):
    if not vals:
        print(f"  {label}: NONE")
        return
    print(f"  {label}: n={len(vals)}, min={min(vals):.1f}, max={max(vals):.1f}, mean={statistics.mean(vals):.1f}, median={statistics.median(vals):.1f}")

print("Z-coordinate by edge category:")
summarize(express_z, "express")
summarize(gate_z, "gates")
summarize(surface_z, "surface")

print()
print("Z distribution of express edges:")
import collections
buckets = collections.Counter(round(z/2)*2 for z in express_z)
for z in sorted(buckets.keys()):
    print(f"  z={z:>4}m: {'#'*buckets[z]} ({buckets[z]})")
