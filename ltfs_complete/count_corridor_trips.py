import xml.etree.ElementTree as ET
for f in ["corridor.passenger.trips.xml", "corridor.bus.trips.xml", "corridor.truck.trips.xml"]:
    n = len(ET.parse(f).getroot().findall("trip"))
    print(f + ": " + str(n) + " trips")
