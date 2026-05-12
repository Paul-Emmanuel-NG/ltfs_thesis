import xml.etree.ElementTree as ET

RENAMES = {
    # randomTrips default type -> paper type
    "passenger": "veh_passenger",
    "bus": "bus_bus",
    "truck": "truck_truck",
}

for f in ["light.passenger.rou.xml", "light.bus.rou.xml", "light.truck.rou.xml"]:
    tree = ET.parse(f)
    root = tree.getroot()
    changed = 0
    # vType elements
    for vt in root.findall("vType"):
        old = vt.get("id", "")
        if old in RENAMES:
            vt.set("id", RENAMES[old])
            changed += 1
    # vehicle elements referencing the type
    for v in root.findall("vehicle"):
        old = v.get("type", "")
        if old in RENAMES:
            v.set("type", RENAMES[old])
            changed += 1
    tree.write(f, encoding="utf-8", xml_declaration=True)
    print(f + ": renamed " + str(changed) + " elements")
