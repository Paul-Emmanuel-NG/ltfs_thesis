import xml.etree.ElementTree as ET
for f in ["light.passenger.rou.xml", "light.bus.rou.xml", "light.truck.rou.xml"]:
    deps = []
    for v in ET.parse(f).getroot().findall("vehicle"):
        try: deps.append(float(v.get("depart","0")))
        except: pass
    if deps:
        print(f + ": n=" + str(len(deps)) + " min=" + str(round(min(deps),1)) + " max=" + str(round(max(deps),1)))
