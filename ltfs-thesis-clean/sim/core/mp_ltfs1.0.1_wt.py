# mp_ltfs1.0.1_wt_400m.py
#
# LTFS-lite controller v1.0.1 (WT logging) + Yan'an 400 m multi-TLS footprint
#
# Key behavior:
# - Max-Pressure surface signal control (same choose_phase block)
# - Logical express layer X auto-detected from NET_FILE (speed + elevation)
# - Gates G auto-detected as incoming edges into the express layer, plus
#   any surface->express movements controlled by the selected TLS set
# - TST gating decides admission at gates; denied vehicles can be rerouted
# - Logs per-vehicle trips with routeKey + class for WT metrics
#
# Auto TLS selection:
# - If MP_TLS_IDS env var is set: uses that comma-separated list
# - Else: uses all TLS IDs within 400 m of the Yan'an anchor TLS
#   (cluster_479314640_850287516) computed from NET_FILE

import csv
import math
import os
import xml.etree.ElementTree as ET
from statistics import median
from pathlib import Path

import traci
import sumolib
from sumolib import checkBinary

from standard_blocks import choose_phase
from ltfs_blocks import tst_delta_t, tst_gate_decision, update_occupancy
from route_key import make_route_key

# ----------------
# Configuration
# ----------------
CORE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = CORE_DIR.parents[1]
RAW_OUTPUT_DIR = PROJECT_ROOT / "outputs" / "raw"

SUMO_CFG = str(PROJECT_ROOT / "sim" / "network" / "osm.sumocfg")
NET_FILE = str(PROJECT_ROOT / "sim" / "network" / "yanan_elevated.net.xml")

USE_GUI = True
STEP_LENGTH = 1.0
MIN_GREEN = 10.0
MAX_SIM_TIME = 7200.0

LOG_CSV = str(RAW_OUTPUT_DIR / "mp_ltfs1.0.1_wt_trips.csv")

# Auto footprint selection (Yan'an interchange)
YANAN_ANCHOR_TLS = "cluster_479314640_850287516"
YANAN_RADIUS_M = 400.0

# Optional override: comma-separated
TLS_IDS_ENV = os.getenv("MP_TLS_IDS", "").strip()

# Auto express settings
EXPRESS_MIN_SPEED = 16.7
EXPRESS_MIN_Z = 6.0
EXPRESS_KEEP_COMPONENTS = 2

EXPRESS_EDGES: set[str] = set()
GATE_EDGES: set[str] = set()

EXPRESS_CAPACITY = 400.0
THETA = float(os.getenv("LTFS_THETA", "60.0"))
KAPPA = float(os.getenv("LTFS_KAPPA", "0.85"))

EXPRESS_SPEED_FAST = 1.3  # multiplier applied ONLY to admitted vehicles on express
V_SURFACE_FF = 10.0
V_EXPRESS_FF = 20.0
GATE_UPDATE_PERIOD = 5

# If True, denied vehicles are rerouted to avoid express edges at the gate
REROUTE_DENIED = True

EDGE_LENGTH_CACHE: dict[str, float] = {}


# ----------------
# Auto TLS footprint helper (net.xml parsing)
# ----------------
def _dist(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _load_net_index(net_file: str):
    """
    Build indices to estimate TLS positions from a SUMO .net.xml.

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
    if tls_id in node_pos:
        return node_pos[tls_id]

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
# Max Pressure auto-config (multi-TLS)
# ----------------
def edge_queue(edge_id: str) -> float:
    return float(traci.edge.getLastStepVehicleNumber(edge_id))


def _is_green(ch: str) -> bool:
    return ch in ("G", "g")


def build_tls_config_from_network(tls_ids: list[str]) -> dict:
    """
    Build TLS_CONFIG automatically from the ACTIVE program.
    tls_config[tls_id]["phases"][phase_index] -> list of movements.
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
                movements.append(
                    {"up_edge": from_edge, "down_edges": to_edges, "turn_probs": [1.0 / k] * k}
                )
            phases_dict[p_idx] = movements

        tls_config[tls_id] = {"phases": phases_dict}

    if not tls_config:
        raise RuntimeError("Auto-built TLS_CONFIG is empty. Check network/TLS programs.")
    return tls_config


def choose_phase_for_tls(tls_id: str, tls_config: dict) -> int:
    cfg = tls_config[tls_id]
    phase_to_movements = {}
    for phase_idx, movement_defs in cfg["phases"].items():
        movements = []
        for m in movement_defs:
            movements.append(
                {
                    "q_up": edge_queue(m["up_edge"]),
                    "q_down_list": [edge_queue(e) for e in m["down_edges"]],
                    "rho_list": m["turn_probs"],
                }
            )
        phase_to_movements[phase_idx] = movements
    return choose_phase(phase_to_movements)


# ----------------
# Express layer / gates (auto)
# ----------------
def _parse_lane_shape_z(shape_str: str) -> list[float]:
    zs: list[float] = []
    if not shape_str:
        return zs
    for pt in shape_str.strip().split():
        parts = pt.split(",")
        if len(parts) >= 3:
            try:
                zs.append(float(parts[2]))
            except ValueError:
                pass
    return zs


def extract_edge_z_and_speed_from_netxml(net_file: str) -> tuple[dict[str, float], dict[str, float]]:
    edge_z_values: dict[str, list[float]] = {}
    edge_speed: dict[str, float] = {}

    tree = ET.parse(net_file)
    root = tree.getroot()

    for edge in root.findall("edge"):
        eid = edge.get("id", "")
        if not eid or eid.startswith(":"):
            continue

        max_spd = edge_speed.get(eid, 0.0)
        z_list = edge_z_values.get(eid, [])

        for lane in edge.findall("lane"):
            spd = lane.get("speed")
            if spd is not None:
                try:
                    max_spd = max(max_spd, float(spd))
                except ValueError:
                    pass

            shape = lane.get("shape")
            if shape:
                z_samples = _parse_lane_shape_z(shape)
                if z_samples:
                    z_list.extend(z_samples)

        edge_speed[eid] = max_spd
        edge_z_values[eid] = z_list

    edge_z: dict[str, float] = {}
    for eid, zs in edge_z_values.items():
        edge_z[eid] = float(median(zs)) if zs else 0.0

    return edge_z, edge_speed


def build_express_edges(net_file: str, min_speed: float, min_z: float, keep_components: int = 2) -> set[str]:
    edge_z, edge_speed = extract_edge_z_and_speed_from_netxml(net_file)
    net = sumolib.net.readNet(net_file)

    candidates = {eid for eid, z in edge_z.items() if (edge_speed.get(eid, 0.0) >= min_speed and z >= min_z)}
    if not candidates:
        candidates = {eid for eid, spd in edge_speed.items() if spd >= min_speed}

    visited: set[str] = set()
    components: list[set[str]] = []

    def neighbors(eid: str) -> set[str]:
        e = net.getEdge(eid)
        if e is None:
            return set()
        nbrs: set[str] = set()
        for out_e in e.getOutgoing().keys():
            oid = out_e.getID()
            if oid in candidates:
                nbrs.add(oid)
        for in_e in e.getIncoming().keys():
            iid = in_e.getID()
            if iid in candidates:
                nbrs.add(iid)
        return nbrs

    for eid in list(candidates):
        if eid in visited:
            continue
        stack = [eid]
        comp: set[str] = set()
        visited.add(eid)
        while stack:
            cur = stack.pop()
            comp.add(cur)
            for nb in neighbors(cur):
                if nb not in visited:
                    visited.add(nb)
                    stack.append(nb)
        components.append(comp)

    components.sort(key=len, reverse=True)
    kept = components[: max(1, int(keep_components))]
    return set().union(*kept) if kept else set()


def build_gate_edges(net_file: str, tls_ids: list[str], express_edges: set[str]) -> set[str]:
    """
    Gate edges = surface edges that feed into the express layer.
    - topo_gates: any incoming edge to an express edge (incoming graph)
    - tls_gates: edges that have a controlled movement surface->express for any TLS in tls_ids
    """
    net = sumolib.net.readNet(net_file)

    topo_gates: set[str] = set()
    for e_id in express_edges:
        e = net.getEdge(e_id)
        if e is None:
            continue
        for inc_edge in e.getIncoming().keys():
            inc_id = inc_edge.getID()
            if inc_id and (inc_id not in express_edges) and (not inc_id.startswith(":")):
                topo_gates.add(inc_id)

    tls_gates: set[str] = set()
    for tls_id in tls_ids:
        try:
            links = traci.trafficlight.getControlledLinks(tls_id)
            for signal_group in links:
                for (from_lane, to_lane, _via_lane) in signal_group:
                    if not from_lane or not to_lane:
                        continue
                    from_edge = traci.lane.getEdgeID(from_lane)
                    to_edge = traci.lane.getEdgeID(to_lane)
                    if (to_edge in express_edges) and (from_edge not in express_edges) and (not from_edge.startswith(":")):
                        tls_gates.add(from_edge)
        except Exception:
            continue

    return topo_gates.union(tls_gates)


# ----------------
# Routing / TST utilities
# ----------------
def get_edge_length(edge_id: str) -> float:
    if edge_id in EDGE_LENGTH_CACHE:
        return EDGE_LENGTH_CACHE[edge_id]
    length = 1.0
    try:
        if not edge_id.startswith(":"):
            lane_ids = traci.edge.getLaneIDs(edge_id)
            if lane_ids:
                length = traci.lane.getLength(lane_ids[0])
    except Exception:
        length = 1.0
    EDGE_LENGTH_CACHE[edge_id] = float(length)
    return EDGE_LENGTH_CACHE[edge_id]


def route_remaining_lengths(veh_id: str, express_edges: set[str]):
    route = list(traci.vehicle.getRoute(veh_id))
    idx = traci.vehicle.getRouteIndex(veh_id)
    if idx < 0:
        idx = 0
    rem_edges = route[idx:] if idx < len(route) else []

    express_len = 0.0
    surface_len = 0.0
    for e in rem_edges:
        L = get_edge_length(e)
        if e in express_edges:
            express_len += L
        else:
            surface_len += L

    return surface_len, express_len


def estimate_delta_t_for_vehicle(veh_id: str) -> float:
    surface_len, express_len = route_remaining_lengths(veh_id, EXPRESS_EDGES)
    if express_len <= 0.0:
        return 0.0
    tau_surface = (surface_len + express_len) / V_SURFACE_FF
    tau_express = surface_len / V_SURFACE_FF + express_len / V_EXPRESS_FF
    return tst_delta_t(tau_surface, tau_express)


def maybe_reroute_denied_vehicle(veh_id: str):
    """
    If denied at a gate and next edge is express, reroute to avoid express edges.
    This makes gating a real gate, not just a speed bonus.
    """
    if not REROUTE_DENIED:
        return

    try:
        route = list(traci.vehicle.getRoute(veh_id))
        idx = traci.vehicle.getRouteIndex(veh_id)
        if idx < 0:
            idx = 0
        if idx + 1 >= len(route):
            return

        next_edge = route[idx + 1]
        if next_edge not in EXPRESS_EDGES:
            return

        dest_edge = route[-1]
        cur_edge = traci.vehicle.getRoadID(veh_id)
        if cur_edge.startswith(":"):
            return

        # Temporary travel time inflation on express edges to get a non-express alternative
        for e in EXPRESS_EDGES:
            try:
                traci.edge.setEffort(e, 1e6)
            except Exception:
                pass

        rr = traci.simulation.findRoute(cur_edge, dest_edge)
        alt_edges = list(rr.edges)

        for e in EXPRESS_EDGES:
            try:
                traci.edge.setEffort(e, 0.0)
            except Exception:
                pass

        if alt_edges and alt_edges[0] == cur_edge:
            traci.vehicle.setRoute(veh_id, alt_edges)
    except Exception:
        return


# ----------------
# Main
# ----------------
def run():
    RAW_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    if not Path(SUMO_CFG).exists():
        raise FileNotFoundError(f"SUMO config not found: {SUMO_CFG}")
    if not Path(NET_FILE).exists():
        raise FileNotFoundError(f"SUMO net file not found: {NET_FILE}")

    # Decide footprint TLS IDs (from net.xml). TraCI filtering happens after start().
    if TLS_IDS_ENV:
        footprint_tls = [x.strip() for x in TLS_IDS_ENV.split(",") if x.strip()]
    else:
        footprint_tls = tls_ids_within_radius(NET_FILE, YANAN_ANCHOR_TLS, YANAN_RADIUS_M)

    sumo_bin = checkBinary("sumo-gui" if USE_GUI else "sumo")
    traci.start([sumo_bin, "-c", SUMO_CFG, "--step-length", str(STEP_LENGTH), "--no-step-log", "true"])

    global EXPRESS_EDGES, GATE_EDGES
    EXPRESS_EDGES = build_express_edges(NET_FILE, EXPRESS_MIN_SPEED, EXPRESS_MIN_Z, EXPRESS_KEEP_COMPONENTS)

    all_tls = traci.trafficlight.getIDList()
    requested = [t for t in footprint_tls if t in all_tls]
    missing = [t for t in footprint_tls if t not in all_tls]
    if missing:
        # Not fatal if NET_FILE includes TLS ids that are not loaded, but usually means mismatch.
        print(f"[ltfs] WARNING: {len(missing)} footprint TLS IDs not in loaded network (first 10): {missing[:10]}")

    if not requested:
        raise RuntimeError("No TLS selected for control. Check NET_FILE/SUMO_CFG or MP_TLS_IDS override.")

    tls_config = build_tls_config_from_network(requested)
    tls_ids = list(tls_config.keys())

    GATE_EDGES = build_gate_edges(NET_FILE, tls_ids, EXPRESS_EDGES)

    print(f"[ltfs] Controlling {len(tls_ids)} TLS (Yan'an 400m auto): {tls_ids}")
    print(f"[ltfs] Auto-built EXPRESS_EDGES: {len(EXPRESS_EDGES)}")
    print(f"[ltfs] Auto-built GATE_EDGES: {len(GATE_EDGES)}")

    current_phase = {tls_id: traci.trafficlight.getPhase(tls_id) for tls_id in tls_ids}
    time_in_phase = {tls_id: 0.0 for tls_id in tls_ids}

    veh_depart: dict[str, float] = {}
    veh_edges: dict[str, list[str]] = {}
    veh_class: dict[str, str] = {}

    # store original speedFactor so we do not change baseline vehicle types
    veh_speedfactor0: dict[str, float] = {}

    o_X = 0.0
    prev_express_veh: set[str] = set()
    admitted_to_express: dict[str, bool] = {}
    gate_decided: dict[str, bool] = {}

    with open(LOG_CSV, "w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["vehID", "routeKey", "class", "depart", "arrival", "travel_time"])

        try:
            while traci.simulation.getTime() < MAX_SIM_TIME:
                traci.simulationStep()
                t = traci.simulation.getTime()

                # departures
                for vid in traci.simulation.getDepartedIDList():
                    veh_depart[vid] = t
                    try:
                        edges = list(traci.vehicle.getRoute(vid))
                    except traci.TraCIException:
                        edges = []
                    veh_edges[vid] = edges
                    try:
                        veh_class[vid] = traci.vehicle.getTypeID(vid)
                    except traci.TraCIException:
                        veh_class[vid] = ""
                    try:
                        veh_speedfactor0[vid] = float(traci.vehicle.getSpeedFactor(vid))
                    except Exception:
                        veh_speedfactor0[vid] = 1.0

                    admitted_to_express[vid] = False
                    gate_decided[vid] = False

                # arrivals (log)
                for vid in traci.simulation.getArrivedIDList():
                    dep = veh_depart.pop(vid, None)
                    edges = veh_edges.pop(vid, [])
                    cls = veh_class.pop(vid, "")
                    veh_speedfactor0.pop(vid, None)

                    if dep is not None:
                        tt = t - dep
                        rkey = make_route_key(edges)
                        writer.writerow([vid, rkey, cls, dep, t, tt])

                    admitted_to_express.pop(vid, None)
                    gate_decided.pop(vid, None)

                # occupancy update
                current_express_veh: set[str] = set()
                for e in EXPRESS_EDGES:
                    for v in traci.edge.getLastStepVehicleIDs(e):
                        current_express_veh.add(v)

                inflow = len(current_express_veh - prev_express_veh)
                outflow = len(prev_express_veh - current_express_veh)
                if EXPRESS_CAPACITY > 0:
                    o_X = update_occupancy(o_X, inflow, outflow, EXPRESS_CAPACITY)
                prev_express_veh = current_express_veh

                # gating decisions
                if int(t) % GATE_UPDATE_PERIOD == 0:
                    # Only evaluate vehicles currently on gate edges (faster and avoids stale IDs)
                    for gate_edge in GATE_EDGES:
                        try:
                            gate_vids = traci.edge.getLastStepVehicleIDs(gate_edge)
                        except Exception:
                            gate_vids = []
                        for vid in gate_vids:
                            if gate_decided.get(vid, False):
                                continue
                            try:
                                delta_t = estimate_delta_t_for_vehicle(vid)
                                u_v = vehicle_combined_urgency(vid, EXPRESS_EDGES, t)
                            except Exception:
                                # Vehicle may have arrived/departed between queries
                                continue

                            if GATE_MODE == "UWA":
                                allow = (u_v * delta_t >= THETA_U) and (o_X <= KAPPA)
                            else:
                                # Default to TST
                                allow = tst_gate_decision(delta_t, o_X, THETA, KAPPA)

                            admitted_to_express[vid] = allow
                            gate_decided[vid] = True

                            if not allow:
                                maybe_reroute_denied_vehicle(vid)

                # apply speed bonus ONLY for admitted vehicles currently on express
                for vid, allowed in list(admitted_to_express.items()):
                    try:
                        edge_id = traci.vehicle.getRoadID(vid)
                    except traci.TraCIException:
                        continue

                    base_sf = veh_speedfactor0.get(vid, 1.0)
                    if allowed and edge_id in EXPRESS_EDGES:
                        traci.vehicle.setSpeedFactor(vid, base_sf * EXPRESS_SPEED_FAST)
                    else:
                        traci.vehicle.setSpeedFactor(vid, base_sf)

                # Max Pressure signal control (multi TLS)
                for tls_id in tls_ids:
                    time_in_phase[tls_id] += STEP_LENGTH
                    if time_in_phase[tls_id] < MIN_GREEN:
                        continue
                    best_phase = choose_phase_for_tls(tls_id, tls_config)
                    if best_phase != current_phase[tls_id]:
                        traci.trafficlight.setPhase(tls_id, best_phase)
                        current_phase[tls_id] = best_phase
                        time_in_phase[tls_id] = 0.0

        finally:
            traci.close()


if __name__ == "__main__":
    run()
