import csv

def load_route_ids(csv_path):
    route_ids = set()
    with open(csv_path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            route_id = row.get("routeID")
            if route_id:
                route_ids.add(route_id)
    return route_ids

# Replace these with your actual filenames
freeflow_csv = "free_flow_times.csv"
ltfs_csv = "mp_ltfs1.0.1_wt_trips.csv"

ff_routes = load_route_ids(freeflow_csv)
ltfs_routes = load_route_ids(ltfs_csv)

only_in_ff = ff_routes - ltfs_routes
only_in_ltfs = ltfs_routes - ff_routes

print(f"Total routeIDs in free-flow: {len(ff_routes)}")
print(f"Total routeIDs in LTFS: {len(ltfs_routes)}")
print(f"RouteIDs only in free-flow: {len(only_in_ff)}")
print(f"RouteIDs only in LTFS: {len(only_in_ltfs)}\n")

print("Examples only in free-flow:", list(only_in_ff)[:10])
print("Examples only in LTFS:", list(only_in_ltfs)[:10])
