import sys
sys.path.insert(0, ".")
import sumolib
from ltfs.net_utils import build_express_edges, build_gate_edges

n = sumolib.net.readNet("yanan_elevated.net.xml")
express = build_express_edges("yanan_elevated.net.xml", 16.7, 6.0, -5.0, 4)
gates = build_gate_edges("yanan_elevated.net.xml", express)

# Corridor-friendly edges: express + gates (entry and exit). All other surface edges get penalty.
corridor_edges = express | gates

PENALTY = 10.0
NORMAL = 1.0

with open("corridor_penalty.weights.xml", "w") as f:
    f.write('<meandata>\n')
    f.write('  <interval begin="0" end="100000">\n')
    n_corr = 0
    n_pen = 0
    for e in n.getEdges():
        eid = e.getID()
        if eid.startswith(":"):
            continue
        # duarouter reads "traveltime" attribute as cost
        # Multiplier is applied via "traveltime"
        free_tt = e.getLength() / max(e.getSpeed(), 0.1)
        if eid in corridor_edges:
            cost = free_tt * NORMAL
            n_corr += 1
        else:
            cost = free_tt * PENALTY
            n_pen += 1
        f.write(f'    <edge id="{eid}" traveltime="{cost:.3f}"/>\n')
    f.write('  </interval>\n')
    f.write('</meandata>\n')

print(f"corridor edges (no penalty): {n_corr}")
print(f"surface edges (10x penalty): {n_pen}")
print(f"wrote corridor_penalty.weights.xml")
