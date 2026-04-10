# mp_freeflow.py (Option 1 free-flow reference: low-demand, signals ON)
# Generates TT_freeflow(routeKey, class) from an uncongested simulation while preserving normal TLS programs.
#
# Key idea (used in Paper 1):
#   TT_ff(routeKey, class) = min TT observed under low demand with signals enabled.
#
# This script supports two ways to reduce interactions:
#   1) depart-time stretching (keeps ALL vehicles but spreads them out in time)  [recommended for coverage]
#   2) demand scaling via --scale (drops vehicles)                              [fast but may reduce coverage]

import csv
import os
import tempfile
import xml.etree.ElementTree as ET
from typing import List, Tuple

from sumolib import checkBinary
import traci

from route_key import make_route_key


# -----------------------
# Configuration (hard-coded: run with `python mp_freeflow.py`)
# -----------------------
# Keep this file in the SAME folder as your `osm.sumocfg` and route files,
# then run:  python mp_freeflow.py

# Use relative path by default (works with your `cd ...; python mp_freeflow.py` workflow).
# If you prefer absolute paths, replace the string below.
SUMO_CFG = r"C:\Users\akinw\Desktop\thesis\2025-12-02-16-58-29\osm.sumocfg"
NET_FILE = r"C:\Users\akinw\Desktop\thesis\2025-12-02-16-58-29\yanan_elevated.net.xml"

# Use GUI? (set to True if you want to watch the run)
USE_GUI = True

# Simulation step length
STEP_LENGTH = 1.0

# Output CSV written in the current working directory
LOG_CSV = "free_flow_times.csv"

# Option 1 free-flow method: preserve signals and remove congestion by spreading departures.
# Example: 10 means a vehicle at depart=100 becomes depart=1000 (same vehicle set, much lower density).
DEPART_STRETCH = 10.0

# Demand scaling drops vehicles and reduces (routeKey, class) coverage, so keep it disabled.
FREEFLOW_SCALE = None

# Extra time after the last (stretched) departure to allow all vehicles to arrive.
# This will be added to the automatically computed simulation end time.
SIM_END_BUFFER = 20000.0

# Hard safety cap (prevents infinite runs if something is wrong).
ABS_MAX_SIM_TIME = 500000.0

# Option A: spread all departures in time (keeps full routeKey coverage)
# Example: 10 means a vehicle at depart=100 becomes depart=1000.
DEPART_STRETCH = float(os.environ.get("FREEFLOW_DEPART_STRETCH", "10.0"))


# -----------------------
# Helpers
# -----------------------
def _parse_route_files_from_sumocfg(sumocfg_path: str) -> List[str]:
    """Return list of route file paths from <input><route-files value="..."> in a SUMO .sumocfg."""
    tree = ET.parse(sumocfg_path)
    root = tree.getroot()
    inp = root.find("input")
    if inp is None:
        return []
    rf = inp.find("route-files")
    if rf is None:
        return []
    val = rf.get("value", "").strip()
    if not val:
        return []
    # SUMO allows comma-separated list
    parts = [p.strip() for p in val.split(",") if p.strip()]
    # Resolve relative paths relative to sumocfg directory
    base_dir = os.path.dirname(os.path.abspath(sumocfg_path))
    resolved = []
    for p in parts:
        if os.path.isabs(p):
            resolved.append(p)
        else:
            resolved.append(os.path.normpath(os.path.join(base_dir, p)))
    return resolved


def _write_stretched_route_file(src_path: str, dst_path: str, stretch: float) -> float:
    tree = ET.parse(src_path)
    root = tree.getroot()

    max_depart = 0.0

    # Vehicles (common in duarouter outputs)
    for veh in root.findall("vehicle"):
        dep = veh.get("depart")
        if dep is None:
            continue
        try:
            dep_f = float(dep)
        except ValueError:
            continue
        dep_s = dep_f * stretch
        veh.set("depart", f"{dep_s:.2f}")
        if dep_s > max_depart:
            max_depart = dep_s

    # Persons (if pedestrian demand is present)
    for person in root.findall("person"):
        dep = person.get("depart")
        if dep is None:
            continue
        try:
            dep_f = float(dep)
        except ValueError:
            continue
        dep_s = dep_f * stretch
        person.set("depart", f"{dep_s:.2f}")
        if dep_s > max_depart:
            max_depart = dep_s

    tree.write(dst_path, encoding="UTF-8", xml_declaration=True)
    return max_depart


def _prepare_freeflow_route_files(sumocfg_path: str, stretch: float) -> Tuple[List[str], str, float]:
    """
    Create temp stretched copies of the route files listed in sumocfg.
    Returns (temp_route_files, temp_dir, max_stretched_depart).
    """
    route_files = _parse_route_files_from_sumocfg(sumocfg_path)
    if not route_files or stretch == 1.0:
        return route_files, "", 0.0

    tmp_dir = tempfile.mkdtemp(prefix="freeflow_routes_")
    out_files = []
    max_depart = 0.0
    for rf in route_files:
        base = os.path.basename(rf)
        dst = os.path.join(tmp_dir, base.replace(".rou.xml", f".stretchedx{stretch:g}.rou.xml"))
        max_d = _write_stretched_route_file(rf, dst, stretch)
        out_files.append(dst)
        if max_d > max_depart:
            max_depart = max_d
    return out_files, tmp_dir, max_depart


def run() -> None:
    sumo_bin = checkBinary("sumo-gui" if USE_GUI else "sumo")

    # Prepare stretched route files for better routeKey coverage under low interactions
    route_files, tmp_dir, max_stretched_depart = _prepare_freeflow_route_files(SUMO_CFG, DEPART_STRETCH)

    # Auto-set simulation end time to cover all stretched departures + buffer
    max_sim_time = min(max_stretched_depart + SIM_END_BUFFER, ABS_MAX_SIM_TIME)

    cmd = [
        sumo_bin,
        "-c", SUMO_CFG,
        "--step-length", str(STEP_LENGTH),
        "--no-step-log", "true",
        "--end", str(max_sim_time),
        # IMPORTANT: keep TLS programs ON (do NOT use --tls.all-off)
    ]

    # Override route files if we created stretched copies
    if tmp_dir and route_files:
        cmd += ["--route-files", ",".join(route_files)]

    if FREEFLOW_SCALE is not None:
        cmd += ["--scale", str(FREEFLOW_SCALE)]

    traci.start(cmd)

    veh_depart = {}
    veh_edges = {}
    veh_class = {}

    with open(LOG_CSV, "w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["vehID", "routeKey", "class", "depart", "arrival", "TT_freeflow"])

        try:
            while (traci.simulation.getMinExpectedNumber() > 0 and
                   traci.simulation.getTime() < max_sim_time):

                traci.simulationStep()
                t = traci.simulation.getTime()

                for vid in traci.simulation.getDepartedIDList():
                    veh_depart[vid] = t
                    try:
                        veh_edges[vid] = list(traci.vehicle.getRoute(vid))
                    except traci.TraCIException:
                        veh_edges[vid] = []
                    try:
                        veh_class[vid] = traci.vehicle.getTypeID(vid)
                    except traci.TraCIException:
                        veh_class[vid] = ""

                for vid in traci.simulation.getArrivedIDList():
                    dep = veh_depart.pop(vid, None)
                    edges = veh_edges.pop(vid, [])
                    cls = veh_class.pop(vid, "")

                    if dep is None:
                        continue

                    tt = t - dep
                    rkey = make_route_key(edges)
                    writer.writerow([vid, rkey, cls, dep, t, tt])

        finally:
            traci.close()

    # Note: tmp_dir is intentionally not deleted automatically so you can inspect it if needed.
    # If you want auto-cleanup, delete tmp_dir here.


if __name__ == "__main__":
    run()
