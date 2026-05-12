import xml.etree.ElementTree as ET
import sys
sys.path.insert(0, ".")
from ltfs.net_utils import build_express_edges, build_gate_edges

express = build_express_edges("yanan_elevated.net.xml", 16.7, 6.0, -5.0, 4)
gates = build_gate_edges("yanan_elevated.net.xml", express)
corridor = express | gates

import sumolib
n = sumolib.net.readNet("yanan_elevated.net.xml")

t = ET.parse("corridor_penalty.weights.xml")
root = t.getroot()
intv = root.find("interval")

corr_costs = []
pen_costs = []
for e in intv.findall("edge"):
    eid = e.get("id")
    cost = float(e.get("traveltime"))
    try:
        free_tt = n.getEdge(eid).getLength() / max(n.getEdge(eid).getSpeed(), 0.1)
    except:
        continue
    ratio = cost / free_tt if free_tt > 0 else 0
    if eid in corridor:
        corr_costs.append(ratio)
    else:
        pen_costs.append(ratio)

print(f"corridor edges: {len(corr_costs)}, mean cost ratio = {sum(corr_costs)/len(corr_costs):.2f}")
print(f"penalty edges: {len(pen_costs)}, mean cost ratio = {sum(pen_costs)/len(pen_costs):.2f}")
