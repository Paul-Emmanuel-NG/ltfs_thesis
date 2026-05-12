import xml.etree.ElementTree as ET
for f in ["light.passenger.rou.xml", "light.bus.rou.xml", "light.truck.rou.xml"]:
    n = len(ET.parse(f).getroot().findall("vehicle"))
    print(f + ": " + str(n) + " vehicles")
