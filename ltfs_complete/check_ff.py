import csv
with open("results/freeflow_MP_NONE_trips.csv") as f:
    rows = list(csv.DictReader(f))
deps = [float(r["depart"]) for r in rows]
arrs = [float(r["arrival"]) for r in rows]
print(f"trips: {len(rows)}")
print(f"depart range: {min(deps):.0f} to {max(deps):.0f}")
print(f"arrival range: {min(arrs):.0f} to {max(arrs):.0f}")
