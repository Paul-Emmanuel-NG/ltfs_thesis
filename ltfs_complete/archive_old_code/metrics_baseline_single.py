# metrics_baseline_single.py

import csv
import numpy as np

from standard_blocks import buffer_index, gini

CSV_FILE = "baseline_singlelayer_trips.csv"


def main():
    travel_times = []

    # read travel times from CSV
    with open(CSV_FILE, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                tt = float(row["travel_time"])
                travel_times.append(tt)
            except (KeyError, ValueError):
                continue

    if not travel_times:
        print("No trips found in CSV. Did the simulation run long enough?")
        return

    tts = np.array(travel_times, dtype=float)

    # basic efficiency stats
    mean_tt = float(np.mean(tts))
    std_tt = float(np.std(tts))

    # reliability stats using your verified function
    T50, T95, BI = buffer_index(tts)

    # fairness or inequality across vehicles
    g_tt = gini(tts)

    print("=== Single layer Max Pressure baseline metrics ===")
    print(f"Number of completed trips: {len(tts)}")
    print(f"Mean travel time: {mean_tt:.2f} s")
    print(f"Std dev travel time: {std_tt:.2f} s")
    print(f"Median travel time (T50): {T50:.2f} s")
    print(f"95th percentile travel time (T95): {T95:.2f} s")
    print(f"Buffer index (BI): {BI:.3f}")
    print(f"Gini over trip travel times: {g_tt:.3f}")


if __name__ == "__main__":
    main()
