import xml.etree.ElementTree as ET
for f in ["light.passenger.rou.xml", "light.bus.rou.xml", "light.truck.rou.xml"]:
    t = ET.parse(f)
    types = set()
    for v in t.getroot().findall("vehicle"):
        types.add(v.get("type", ""))
    print(f + ": types=" + str(types))
