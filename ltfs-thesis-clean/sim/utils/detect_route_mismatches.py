# detect_route_mismatches.py
import csv
from collections import Counter, defaultdict

FREEFLOW = "free_flow_times.csv"
TRIPS = "baseline_wt_trips.csv"  # or LTFS trips csv

def read_keys(path: str, key_field_candidates=("routeKey", "routeID")):
    keys = []
    endpoints = []
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            k = ""
            for c in key_field_candidates:
                v = (row.get(c) or "").strip()
                if v:
                    k = v
                    break
            if not k:
                continue
            keys.append(k)
            # quick endpoint parse if using the routeKey format I gave
            # "first->last|n=...|h=..."
            if "->" in k and "|n=" in k:
                endpoints.append(k.split("|n=")[0])
    return set(keys), keys, endpoints

def main():
    ff_set, ff_all, ff_end = read_keys(FREEFLOW)
    tr_set, tr_all, tr_end = read_keys(TRIPS)

    only_ff = ff_set - tr_set
    only_tr = tr_set - ff_set

    print("=== Key coverage ===")
    print(f"Freeflow unique keys: {len(ff_set)}")
    print(f"Trips unique keys:    {len(tr_set)}")
    print(f"Only in freeflow:     {len(only_ff)}")
    print(f"Only in trips:        {len(only_tr)}")
    print()

    # Show which missing endpoints happen most often (helps spot route generation issues)
    ff_end_ctr = Counter(ff_end)
    tr_end_ctr = Counter(tr_end)

    print("=== Top endpoints in trips with NO freeflow match ===")
    missing_trip_endpoints = Counter()
    for k in only_tr:
        if "->" in k and "|n=" in k:
            missing_trip_endpoints[k.split("|n=")[0]] += 1

    for ep, c in missing_trip_endpoints.most_common(30):
        print(f"{ep}: {c}")

    print()
    print("=== Top endpoints in freeflow with NO trips match ===")
    missing_ff_endpoints = Counter()
    for k in only_ff:
        if "->" in k and "|n=" in k:
            missing_ff_endpoints[k.split("|n=")[0]] += 1

    for ep, c in missing_ff_endpoints.most_common(30):
        print(f"{ep}: {c}")

if __name__ == "__main__":
    main()
