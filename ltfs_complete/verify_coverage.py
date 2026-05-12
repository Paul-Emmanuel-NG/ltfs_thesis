import xml.etree.ElementTree as ET
import sys
sys.path.insert(0, ".")
from ltfs.net_utils import build_express_edges, build_gate_edges

express = build_express_edges("yanan_elevated.net.xml", 16.7, 6.0, -5.0, 4)
gates = build_gate_edges("yanan_elevated.net.xml", express)

for f in ["light.passenger.rou.xml", "corridor_pen.passenger.rou.xml"]:
    t = ET.parse(f)
    vehicles = t.getroot().findall("vehicle")
    n_total = len(vehicles)
    n_via_gate = sum(1 for v in vehicles
                     if (set((v.find("route").get("edges") or "").split()) & gates))
    pct = 100 * n_via_gate / n_total if n_total else 0
    print(f"{f}: {n_via_gate}/{n_total} = {pct:.1f}% via gates")
