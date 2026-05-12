import xml.etree.ElementTree as ET

PREFIXES = {
    "light.passenger.rou.xml": "veh",
    "light.bus.rou.xml": "bus",
    "light.truck.rou.xml": "truck",
}

for fname, prefix in PREFIXES.items():
    tree = ET.parse(fname)
    root = tree.getroot()
    n = 0
    for v in root.findall("vehicle"):
        old = v.get("id", "")
        if not old.startswith(prefix):
            v.set("id", prefix + old)
            n += 1
    tree.write(fname, encoding="utf-8", xml_declaration=True)
    print(fname + ": prefixed " + str(n) + " ids with '" + prefix + "'")
