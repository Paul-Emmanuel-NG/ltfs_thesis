import csv, statistics
for f in ["A8_S1_seed1_wmax12_trips.csv", "A8_S1_seed1_trips.csv", "A8_S1_seed1_wmax20_trips.csv"]:
    with open("results/" + f) as fp:
        rows = list(csv.DictReader(fp))
    tts = [float(r["travel_time"]) for r in rows]
    label = f.replace("A8_S1_seed1_", "").replace("_trips.csv", "")
    if label == "":
        label = "wmax15_default"
    print(f"{label}: n={len(tts)}  mean_tt={statistics.mean(tts):.2f}s  p95={sorted(tts)[int(0.95*len(tts))]:.0f}s")
