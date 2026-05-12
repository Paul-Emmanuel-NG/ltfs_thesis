import xml.etree.ElementTree as ET
for f in ["light.passenger.rou.xml", "corridor.passenger.rou.xml"]:
    t = ET.parse(f)
    vehs = t.getroot().findall("vehicle")
    if vehs:
        first = vehs[0]
        last = vehs[-1]
        print(f + ":")
        print("  count:", len(vehs))
        print("  first id:", first.get("id"), "type:", first.get("type"))
        print("  last id:", last.get("id"), "type:", last.get("type"))
