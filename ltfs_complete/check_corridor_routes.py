import xml.etree.ElementTree as ET
import sys
sys.path.insert(0, ".")
from ltfs.net_utils import build_express_edges, build_gate_edges

express = build_express_edges("yanan_elevated.net.xml", 16.7, 6.0, -5.0, 4)
gates = build_gate_edges("yanan_elevated.net.xml", express)

for f in ["corridor_pen.passenger.rou.xml", "corridor_pen.bus.rou.xml", "corridor_pen.truck.rou.xml"]:
    t = ET.parse(f)
    vehicles = t.getroot().findall("vehicle")
    if not vehicles:
        print(f + ": NO VEHICLES")
        continue
    n_total = len(vehicles)
    n_via_express = 0
    n_via_gate = 0
    for v in vehicles:
        r = v.find("route")
        if r is None:
            continue
        edges = set((r.get("edges") or "").split())
        if edges & express:
            n_via_express += 1
        if edges & gates:
            n_via_gate += 1
    pct_e = 100 * n_via_express / n_total
    pct_g = 100 * n_via_gate / n_total
    print(f + ":")
    print(f"  total vehicles: {n_total}")
    print(f"  via express edges: {n_via_express} ({pct_e:.1f}%)")
    print(f"  via gate edges:    {n_via_gate} ({pct_g:.1f}%)")
