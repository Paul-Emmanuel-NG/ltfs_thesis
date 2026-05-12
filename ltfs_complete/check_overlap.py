import csv

# Get vehicle IDs from both runs
with open("results/freeflow_MP_NONE_trips.csv") as f:
    ff_vids = set(row["vehID"] for row in csv.DictReader(f))

with open("results/A0_S1_seed1_trips.csv") as f:
    a0_vids = set(row["vehID"] for row in csv.DictReader(f))

print(f"freeflow vids: {len(ff_vids)}")
print(f"A0 vids: {len(a0_vids)}")
print(f"vids in BOTH: {len(ff_vids & a0_vids)}")
print(f"vids only in freeflow: {len(ff_vids - a0_vids)}")
print(f"vids only in A0: {len(a0_vids - ff_vids)}")

# Show some examples
print()
print("Sample vids in BOTH:", list(ff_vids & a0_vids)[:5])
print("Sample vids only in A0:", list(a0_vids - ff_vids)[:5])
print("Sample vids only in freeflow:", list(ff_vids - a0_vids)[:5])
