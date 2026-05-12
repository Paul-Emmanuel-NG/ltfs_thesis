import xml.etree.ElementTree as ET
import sys
sys.path.insert(0, ".")
from ltfs.net_utils import build_express_edges, build_gate_edges
import sumolib

express = build_express_edges("yanan_elevated.net.xml", 16.7, 6.0, -5.0, 4)
gates = build_gate_edges("yanan_elevated.net.xml", express)

n = sumolib.net.readNet("yanan_elevated.net.xml")

# Build z-mean per edge for express edges
import statistics
elev_express = set()  # z >= 5
tunnel_express = set()  # z <= -5
for eid in express:
    try:
        e = n.getEdge(eid)
        shape = e.getShape3D()
        if not shape: continue
        z_mean = statistics.mean(z for x,y,z in shape)
        if z_mean >= 5:
            elev_express.add(eid)
        elif z_mean <= -5:
            tunnel_express.add(eid)
    except:
        pass

print(f"Elevated express edges (z >= 5m):  {len(elev_express)}")
print(f"Tunnel express edges (z <= -5m):   {len(tunnel_express)}")
print(f"Other (between -5 and 5):           {len(express) - len(elev_express) - len(tunnel_express)}")
print()

# Now check trips
for f in ["light.passenger.rou.xml", "corridor_pen.passenger.rou.xml"]:
    t = ET.parse(f)
    vehicles = t.getroot().findall("vehicle")
    n_total = len(vehicles)
    n_via_elev = 0
    n_via_tun = 0
    n_via_both = 0
    for v in vehicles:
        r = v.find("route")
        if r is None: continue
        edges = set((r.get("edges") or "").split())
        in_elev = bool(edges & elev_express)
        in_tun = bool(edges & tunnel_express)
        if in_elev: n_via_elev += 1
        if in_tun: n_via_tun += 1
        if in_elev and in_tun: n_via_both += 1
    print(f"{f}:")
    print(f"  total: {n_total}")
    print(f"  via elevated: {n_via_elev} ({100*n_via_elev/n_total:.1f}%)")
    print(f"  via tunnel:   {n_via_tun} ({100*n_via_tun/n_total:.1f}%)")
    print(f"  via both:     {n_via_both} ({100*n_via_both/n_total:.1f}%)")
