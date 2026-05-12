# mp_single_baseline_wt.py
#
# Max Pressure controller (auto TLS footprint selection for Yan'an interchange)
#
# What changed vs your current mp_single_baseline_wt.py:
# - If $env:MP_TLS_IDS is NOT set, we automatically select *only* the TLS IDs
#   that fall within 400 m of the Yan'an anchor TLS (cluster_479314640_850287516)
#   using NET_FILE (no manual comma list needed).
# - If $env:MP_TLS_IDS IS set, it still overrides the auto-selection.

import csv
import math
import os
import xml.etree.ElementTree as ET

import traci
from sumolib import checkBinary

from standard_blocks import choose_phase
from ltfs_blocks import urgency_from_type_id, route_urgency_from_remaining_length, combined_urgency
from route_key import make_route_key

# ----------------
# Configuration
# ----------------
SUMO_CFG = r"C:\Users\akinw\Desktop\thesis\2025-12-02-16-58-29\osm.sumocfg"
NET_FILE = r"C:\Users\akinw\Desktop\thesis\2025-12-02-16-58-29\yanan_elevated.net.xml"

USE_GUI = True
STEP_LENGTH = 1.0
MIN_GREEN = 10.0
MAX_SIM_TIME = 7200.0

# Trip log for WT metrics (matches metrics_wt_all.py expectations)
LOG_CSV = "baseline_wt_trips.csv"

# Auto footprint selection (Yan'an interchange)
YANAN_ANCHOR_TLS = "cluster_479314640_850287516"
YANAN_RADIUS_M = 400.0

# If MP_TLS_IDS env var is set, it overrides the auto footprint.
TLS_IDS_ENV = os.getenv("MP_TLS_IDS", "").strip()


# ----------------
# Auto TLS footprint helper (net.xml parsing)
# ----------------
def _dist(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _load_net_index(net_file: str):
    """
    Build small indices needed to estimate TLS positions from a SUMO .net.xml.

    Returns:
      node_pos: junction/node id -> (x, y)
      junction_type: junction id -> type
      edge_ends: edge id -> (from_node, to_node)
      tls_ids: set of TLS ids (from connection tl=... plus traffic_light junctions)
      tls_edges: tls id -> set of edge IDs referenced by its controlled connections
    """
    tree = ET.parse(net_file)
    root = tree.getroot()

    node_pos: dict[str, tuple[float, float]] = {}
    junction_type: dict[str, str] = {}
    for j in root.findall("junction"):
        jid = j.get("id")
        if not jid:
            continue
        x = j.get("x")
        y = j.get("y")
        if x is None or y is None:
            continue
        try:
            node_pos[jid] = (float(x), float(y))
        except ValueError:
            continue
        junction_type[jid] = (j.get("type") or "").strip()

    edge_ends: dict[str, tuple[str, str]] = {}
    for e in root.findall("edge"):
        eid = e.get("id")
        if not eid or eid.startswith(":"):
            continue
        fr = e.get("from")
        to = e.get("to")
        if fr and to:
            edge_ends[eid] = (fr, to)

    tls_ids: set[str] = set()
    tls_edges: dict[str, set[str]] = {}

    for c in root.findall("connection"):
        tl = c.get("tl")
        if not tl:
            continue
        tls_ids.add(tl)
        fr_edge = c.get("from")
        to_edge = c.get("to")
        if fr_edge:
            tls_edges.setdefault(tl, set()).add(fr_edge)
        if to_edge:
            tls_edges.setdefault(tl, set()).add(to_edge)

    # Many OSM/netconvert networks also mark the controlling junction as type=traffic_light.
    for jid, t in junction_type.items():
        if t == "traffic_light":
            tls_ids.add(jid)

    return node_pos, edge_ends, tls_ids, tls_edges


def _tls_position(
    tls_id: str,
    node_pos: dict[str, tuple[float, float]],
    edge_ends: dict[str, tuple[str, str]],
    tls_edges: dict[str, set[str]],
) -> tuple[float, float] | None:
    # Prefer direct junction coordinates if tls_id exists as a junction id.
    if tls_id in node_pos:
        return node_pos[tls_id]

    # Otherwise, approximate by centroid of endpoints of edges mentioned in controlled connections.
    pts: list[tuple[float, float]] = []
    for eid in tls_edges.get(tls_id, set()):
        fr_to = edge_ends.get(eid)
        if not fr_to:
            continue
        fr, to = fr_to
        if fr in node_pos:
            pts.append(node_pos[fr])
        if to in node_pos:
            pts.append(node_pos[to])

    if not pts:
        return None

    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)
    return (cx, cy)


def tls_ids_within_radius(net_file: str, anchor_tls: str, radius_m: float) -> list[str]:
    node_pos, edge_ends, tls_ids, tls_edges = _load_net_index(net_file)
    anchor_xy = _tls_position(anchor_tls, node_pos, edge_ends, tls_edges)
    if anchor_xy is None:
        raise RuntimeError(f"Could not locate anchor TLS '{anchor_tls}' in net file: {net_file}")

    within: list[tuple[str, float]] = []
    for tid in tls_ids:
        xy = _tls_position(tid, node_pos, edge_ends, tls_edges)
        if xy is None:
            continue
        d = _dist(anchor_xy, xy)
        if d <= radius_m:
            within.append((tid, d))

    within.sort(key=lambda x: x[1])
    return [tid for tid, _d in within]


# ----------------
# Max Pressure helpers
# ----------------
_VEH_U_CACHE: dict[str, float] = {}
_VEH_LAST_T: dict[str, float] = {}
_EDGE_U_CACHE: dict[str, tuple[float,float]] = {}

def vehicle_u(veh_id: str, edges: list[str], now_t: float) -> float:
    last=_VEH_LAST_T.get(veh_id,-1e9)
    if (now_t-last)<PWMP_EDGE_U_PERIOD:
        return _VEH_U_CACHE.get(veh_id,0.4)
    try:
        type_id=traci.vehicle.getTypeID(veh_id)
    except Exception:
        type_id=""
    u_class=urgency_from_type_id(type_id)
    try:
        L=sum(get_edge_length(e) for e in edges if e and (not e.startswith(":")))
    except Exception:
        L=0.0
    u_route=route_urgency_from_remaining_length(L, ROUTE_L_REF)
    u=combined_urgency(u_class,u_route,URGENCY_LAM)
    _VEH_U_CACHE[veh_id]=u
    _VEH_LAST_T[veh_id]=now_t
    return u

def edge_mean_u(edge_id: str, now_t: float) -> float:
    last=_EDGE_U_CACHE.get(edge_id,(-1e9,0.4))
    if (now_t-last[0])<PWMP_EDGE_U_PERIOD:
        return float(last[1])
    try:
        vids=traci.edge.getLastStepVehicleIDs(edge_id)
    except Exception:
        vids=[]
    if not vids:
        mu=0.4
    else:
        us=[]
        for v in vids:
            # we don't know full route here; class-only fallback
            try:
                us.append(urgency_from_type_id(traci.vehicle.getTypeID(v)))
            except Exception:
                us.append(0.4)
        mu=float(sum(us)/max(1,len(us)))
    _EDGE_U_CACHE[edge_id]=(now_t,mu)
    return mu

def edge_queue(edge_id: str) -> float:
    """Queue proxy: number of vehicles on the edge in the last step."""
    return float(traci.edge.getLastStepVehicleNumber(edge_id))


def _is_green(ch: str) -> bool:
    """Treat both 'G' and 'g' as permissive greens."""
    return ch in ("G", "g")


def build_tls_config_from_network(tls_ids: list[str]) -> dict:
    """
    Build TLS_CONFIG automatically from the ACTIVE program:
    TLS_CONFIG[tls_id]["phases"][phase_index] = list of movements.

    Each movement:
      {
        "up_edge": <from_edge>,
        "down_edges": [<to_edge_1>, <to_edge_2>, ...],
        "turn_probs": [p1, p2, ...]  (equal split)
      }
    """
    tls_config: dict = {}

    for tls_id in tls_ids:
        controlled_links = traci.trafficlight.getControlledLinks(tls_id)
        if not controlled_links:
            continue

        active_prog_id = traci.trafficlight.getProgram(tls_id)
        programs = traci.trafficlight.getCompleteRedYellowGreenDefinition(tls_id)
        if not programs:
            continue

        active = None
        for p in programs:
            if getattr(p, "programID", None) == active_prog_id:
                active = p
                break
        if active is None:
            active = programs[0]
        if not getattr(active, "phases", None):
            continue

        phases_dict: dict[int, list[dict]] = {}
        for p_idx, ph in enumerate(active.phases):
            state = ph.state

            from_to: dict[str, set[str]] = {}
            for sg_idx, sg_links in enumerate(controlled_links):
                if sg_idx >= len(state):
                    break
                if not _is_green(state[sg_idx]):
                    continue
                for from_lane, to_lane, _via_lane in sg_links:
                    if not from_lane or not to_lane:
                        continue
                    try:
                        from_edge = traci.lane.getEdgeID(from_lane)
                        to_edge = traci.lane.getEdgeID(to_lane)
                    except traci.TraCIException:
                        continue
                    if not from_edge or from_edge.startswith(":"):
                        continue
                    if not to_edge or to_edge.startswith(":"):
                        continue
                    from_to.setdefault(from_edge, set()).add(to_edge)

            movements: list[dict] = []
            for from_edge, to_edges_set in from_to.items():
                to_edges = sorted(list(to_edges_set))
                if not to_edges:
                    continue
                k = len(to_edges)
                movements.append({"up_edge": from_edge, "down_edges": to_edges, "turn_probs": [1.0 / k] * k})
            phases_dict[p_idx] = movements

        tls_config[tls_id] = {"phases": phases_dict}

    return tls_config


def choose_phase_for_tls(tls_id: str, tls_config: dict, now_t: float) -> int:
    cfg = tls_config[tls_id]
    phase_to_movements = {}
    for phase_idx, movement_defs in cfg["phases"].items():
        movements = []
        for m in movement_defs:
            q_up = edge_queue(m["up_edge"])
            q_down_list = [edge_queue(e) for e in m["down_edges"]]
            rho_list = m["turn_probs"]
            movements.append({"q_up": q_up, "q_down_list": q_down_list, "rho_list": rho_list})
        phase_to_movements[phase_idx] = movements
    return choose_phase(phase_to_movements)


def _assert_tls_config_is_valid(tls_config: dict):
    sim_tls_ids = set(traci.trafficlight.getIDList())
    sim_edge_ids = set(traci.edge.getIDList())

    if not tls_config:
        raise RuntimeError("Auto-built TLS_CONFIG is empty. Check that your network has TLS and routes loaded.")

    for tls_id, cfg in tls_config.items():
        if tls_id not in sim_tls_ids:
            raise ValueError(f"TLS_ID '{tls_id}' not found in simulation. Check your cfg/network.")
        for phase_idx, movs in cfg["phases"].items():
            for m in movs:
                if m["up_edge"] not in sim_edge_ids:
                    raise ValueError(f"up_edge '{m['up_edge']}' not found (TLS {tls_id}, phase {phase_idx})")
                for de in m["down_edges"]:
                    if de not in sim_edge_ids:
                        raise ValueError(f"down_edge '{de}' not found (TLS {tls_id}, phase {phase_idx})")


# ----------------
# Main
# ----------------
def run():
    sumo_binary = checkBinary("sumo-gui" if USE_GUI else "sumo")
    traci.start([
        sumo_binary,
        "-c",
        SUMO_CFG,
        "--step-length",
        str(STEP_LENGTH),
        "--no-step-log",
        "true",
    ])

    try:
        all_tls = traci.trafficlight.getIDList()

        # 1) Override (manual) if MP_TLS_IDS is set
        if TLS_IDS_ENV:
            requested = [x.strip() for x in TLS_IDS_ENV.split(",") if x.strip()]
            tls_ids = [t for t in requested if t in all_tls]
            missing = [t for t in requested if t not in all_tls]
            if missing:
                raise ValueError(f"These TLS IDs are not in the loaded network: {missing}")
        else:
            # 2) Auto Yan'an footprint (400 m)
            footprint = tls_ids_within_radius(NET_FILE, YANAN_ANCHOR_TLS, YANAN_RADIUS_M)
            tls_ids = [t for t in footprint if t in all_tls]
            if not tls_ids:
                # Very defensive fallback
                tls_ids = list(all_tls)
                print("[baseline] WARNING: Auto footprint produced 0 TLS; falling back to ALL TLS.")

        # Auto-build config for the selected TLS IDs
        tls_config = build_tls_config_from_network(tls_ids)
        _assert_tls_config_is_valid(tls_config)

        controlled_tls_ids = list(tls_config.keys())
        current_phase = {tls_id: traci.trafficlight.getPhase(tls_id) for tls_id in controlled_tls_ids}
        time_in_phase = {tls_id: 0.0 for tls_id in controlled_tls_ids}

        print(f"[baseline] Controlling {len(controlled_tls_ids)} TLS (Yan'an 400m auto): {controlled_tls_ids}")

        # Trip logging (routeKey-compatible with freeflow + LTFS logs)
        veh_depart: dict[str, float] = {}
        veh_edges: dict[str, list[str]] = {}
        veh_class: dict[str, str] = {}

        with open(LOG_CSV, "w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(["vehID", "routeKey", "class", "depart", "arrival", "travel_time"])

            while traci.simulation.getTime() < MAX_SIM_TIME and traci.simulation.getMinExpectedNumber() > 0:
                traci.simulationStep()
                t = traci.simulation.getTime()

                # departures
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

                # arrivals (log)
                for vid in traci.simulation.getArrivedIDList():
                    dep = veh_depart.pop(vid, None)
                    edges = veh_edges.pop(vid, [])
                    cls = veh_class.pop(vid, "")

                    if dep is None:
                        continue

                    tt = t - dep
                    rkey = make_route_key(edges)
                    writer.writerow([vid, rkey, cls, dep, t, tt])

                # Max Pressure signal control (multi TLS)
                for tls_id in controlled_tls_ids:
                    time_in_phase[tls_id] += STEP_LENGTH
                    if time_in_phase[tls_id] < MIN_GREEN:
                        continue

                    best_phase = choose_phase_for_tls(tls_id, tls_config, t)
                    if best_phase != current_phase[tls_id]:
                        traci.trafficlight.setPhase(tls_id, best_phase)
                        current_phase[tls_id] = best_phase
                        time_in_phase[tls_id] = 0.0

    finally:
        traci.close()


if __name__ == "__main__":
    run()
