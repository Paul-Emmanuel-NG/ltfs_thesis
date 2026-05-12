import csv

def first_keys(path, n=5):
    with open(path) as f:
        r = csv.DictReader(f)
        keys = []
        for row in r:
            keys.append((row.get("routeKey", ""), row.get("class", "")))
            if len(keys) >= n:
                break
    return keys

print("=== A0 seed 1 trip CSV (first 5 keys) ===")
for k in first_keys("results/A0_S1_seed1_trips.csv"):
    print(" ", k)

print()
print("=== freeflow_MP_nogate.csv (first 5 keys) ===")
for k in first_keys("results/freeflow_MP_nogate.csv"):
    print(" ", k)

with open("results/A0_S1_seed1_trips.csv") as f:
    trip_keys = set()
    for row in csv.DictReader(f):
        trip_keys.add((row.get("routeKey", ""), row.get("class", "")))

with open("results/freeflow_MP_nogate.csv") as f:
    ff_keys = set()
    for row in csv.DictReader(f):
        ff_keys.add((row.get("routeKey", ""), row.get("class", "")))

print()
print(f"Trip keys: {len(trip_keys)}")
print(f"Freeflow keys: {len(ff_keys)}")
print(f"Overlap: {len(trip_keys & ff_keys)}")
