import csv, statistics
with open("results/freeflow_MP_NONE_trips.csv") as f:
    rows = list(csv.DictReader(f))
deps = [float(r["depart"]) for r in rows]
arrs = [float(r["arrival"]) for r in rows]
tts = [float(r["travel_time"]) for r in rows]
print(f"trips: {len(rows)}")
print(f"depart: min={min(deps):.1f} median={statistics.median(deps):.1f} max={max(deps):.1f}")
print(f"arrival: min={min(arrs):.1f} max={max(arrs):.1f}")
print(f"travel_time: min={min(tts):.1f} median={statistics.median(tts):.1f} max={max(tts):.1f}")
# Sample 5
for r in rows[:5]:
    print(f"  vid={r['vehID']} depart={r['depart']} arrival={r['arrival']} tt={r['travel_time']}")
