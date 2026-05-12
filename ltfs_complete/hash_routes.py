import xml.etree.ElementTree as ET
import hashlib
for f in ["light.passenger.rou.xml", "corridor.passenger.rou.xml"]:
    t = ET.parse(f)
    routes = []
    for v in t.getroot().findall("vehicle"):
        r = v.find("route")
        if r is not None:
            routes.append(r.get("edges", ""))
    routes_str = "|".join(sorted(routes))
    h = hashlib.md5(routes_str.encode()).hexdigest()[:12]
    print(f + ": n_routes=" + str(len(routes)) + " hash=" + h)
    # Check if any route mentions the corridor edge
    via_edge = "178702239#1"
    n_via = sum(1 for r in routes if via_edge in r.split())
    print("  routes containing " + via_edge + ": " + str(n_via) + " (" + str(round(100*n_via/len(routes),1)) + "%)")
