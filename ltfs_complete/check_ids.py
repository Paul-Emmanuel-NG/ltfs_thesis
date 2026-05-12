import xml.etree.ElementTree as ET
for f in ["light.passenger.rou.xml", "light.bus.rou.xml", "light.truck.rou.xml"]:
    t = ET.parse(f)
    ids = [v.get("id") for v in t.getroot().findall("vehicle")]
    print(f + ": first 3 ids = " + str(ids[:3]) + "  last = " + str(ids[-1]))
