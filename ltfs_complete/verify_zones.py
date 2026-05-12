import xml.etree.ElementTree as ET
t = ET.parse("corridor.passenger.trips.xml")
trips = t.getroot().findall("trip")
print("trips:", len(trips))

with open("west_edges.txt") as f:
    west = set(line.strip() for line in f if line.strip())
with open("east_edges.txt") as f:
    east = set(line.strip() for line in f if line.strip())

n_from_west = sum(1 for tr in trips if tr.get("from") in west)
n_to_east = sum(1 for tr in trips if tr.get("to") in east)
n_both = sum(1 for tr in trips if tr.get("from") in west and tr.get("to") in east)
print(f"from-west: {n_from_west} ({100*n_from_west/len(trips):.1f}%)")
print(f"to-east: {n_to_east} ({100*n_to_east/len(trips):.1f}%)")
print(f"both: {n_both} ({100*n_both/len(trips):.1f}%)")
