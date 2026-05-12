import xml.etree.ElementTree as ET
SPECS = {
    "corridor.passenger.rou.xml": ("passenger", "veh_passenger", "veh"),
    "corridor.bus.rou.xml":       ("bus",       "bus_bus",       "bus"),
    "corridor.truck.rou.xml":     ("truck",     "truck_truck",   "truck"),
}
for fname, (old_type, new_type, prefix) in SPECS.items():
    tree = ET.parse(fname)
    root = tree.getroot()
    n_type = 0
    n_id = 0
    for vt in root.findall("vType"):
        if vt.get("id", "") == old_type:
            vt.set("id", new_type); n_type += 1
    for v in root.findall("vehicle"):
        if v.get("type", "") == old_type:
            v.set("type", new_type); n_type += 1
        old_id = v.get("id", "")
        if old_id and not old_id.startswith(prefix):
            v.set("id", prefix + old_id); n_id += 1
    tree.write(fname, encoding="utf-8", xml_declaration=True)
    print(fname + ": types=" + str(n_type) + " ids=" + str(n_id))
