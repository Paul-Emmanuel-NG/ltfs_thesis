import xml.etree.ElementTree as ET
import sys
sys.path.insert(0, ".")
from ltfs.net_utils import build_express_edges, build_gate_edges

express = build_express_edges("yanan_elevated.net.xml", 16.7, 6.0, -5.0, 4)
gates = build_gate_edges("yanan_elevated.net.xml", express)
print("express edges:", len(express))
print("gate entry edges:", len(gates))

for f in ["light.passenger.rou.xml", "light.bus.rou.xml", "light.truck.rou.xml"]:
    t = ET.parse(f)
    total = 0
    through_gate = 0
    through_express = 0
    for v in t.getroot().findall("vehicle"):
        total += 1
        r = v.find("route")
        if r is None:
            continue
        edges = set((r.get("edges") or "").split())
        if edges & gates:
            through_gate += 1
        if edges & express:
            through_express += 1
    pct_gate = 100 * through_gate / total if total else 0
    pct_exp = 100 * through_express / total if total else 0
    print(f + ": total=" + str(total) + "  via gate=" + str(through_gate) + " (" + str(round(pct_gate,1)) + "%)  via express=" + str(through_express) + " (" + str(round(pct_exp,1)) + "%)")
