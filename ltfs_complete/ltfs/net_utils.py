"""
net_utils.py — Network-level utilities: express edge detection, gate detection,
TLS footprint selection, express capacity estimation.

Parses the SUMO .net.xml using sumolib where possible and raw XML for
lane z-coordinates (which sumolib does not expose).
"""

from __future__ import annotations
import math
import statistics
import xml.etree.ElementTree as ET
from typing import Dict, List, Set, Tuple

import sumolib


# ============================================================================
# Raw XML parsing for lane shape z-coordinates
# ============================================================================

def _parse_lane_shape_z(shape_str: str) -> List[float]:
    """Extract the z-coordinate from each point in a SUMO lane shape string."""
    zs: List[float] = []
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


def extract_edge_z_and_speed(net_file: str) -> Tuple[Dict[str, float], Dict[str, float]]:
    """
    Returns (edge_z, edge_speed):
      edge_z[eid]     = median z across all lane-shape samples (0 if no z data)
      edge_speed[eid] = max lane speed on that edge
    """
    tree = ET.parse(str(net_file))
    root = tree.getroot()

    edge_z_values: Dict[str, List[float]] = {}
    edge_speed: Dict[str, float] = {}

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
                z_list.extend(_parse_lane_shape_z(shape))
        edge_speed[eid] = max_spd
        edge_z_values[eid] = z_list

    edge_z: Dict[str, float] = {}
    for eid, zs in edge_z_values.items():
        edge_z[eid] = float(statistics.median(zs)) if zs else 0.0

    return edge_z, edge_speed


# ============================================================================
# Express edge detection (Ambiguity A3)
# ============================================================================

def build_express_edges(
    net_file: str,
    min_speed: float,
    min_z: float,
    max_z: float,
    keep_components: int = 4,
) -> Set[str]:
    """
    Identify express-layer edges by:
      - Speed ≥ min_speed, AND
      - (median_z ≥ min_z OR median_z ≤ max_z)  -- catches elevated AND tunnel

    Then keep the top-N largest connected components (elevated and tunnel
    segments form separate components in the graph).

    Ambiguity A3 in AUDIT.md: this is our interpretation of the paper's
    "functional express class" combining both grade-separated sub-facilities.
    """
    edge_z, edge_speed = extract_edge_z_and_speed(net_file)
    net = sumolib.net.readNet(str(net_file))

    # Candidates: fast AND (above OR below) — grade-separated in either direction
    candidates: Set[str] = set()
    for eid in edge_speed:
        spd = edge_speed.get(eid, 0.0)
        if spd < min_speed:
            continue
        z = edge_z.get(eid, 0.0)
        if z >= min_z or z <= max_z:
            candidates.add(eid)

    # Fallback: if no edge has z data (flat network), just speed
    if not candidates:
        candidates = {eid for eid, spd in edge_speed.items() if spd >= min_speed}

    # Connected components within candidates
    def neighbors(eid: str) -> Set[str]:
        try:
            e = net.getEdge(eid)
        except Exception:
            return set()
        nbrs: Set[str] = set()
        for out_e in e.getOutgoing().keys():
            oid = out_e.getID()
            if oid in candidates:
                nbrs.add(oid)
        for in_e in e.getIncoming().keys():
            iid = in_e.getID()
            if iid in candidates:
                nbrs.add(iid)
        return nbrs

    visited: Set[str] = set()
    components: List[Set[str]] = []
    for eid in list(candidates):
        if eid in visited:
            continue
        stack = [eid]
        comp: Set[str] = set()
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


# ============================================================================
# Express-layer capacity (Ambiguity A2)
# ============================================================================

def express_capacity_from_net(net_file: str, express_edges: Set[str], spacing_m: float) -> float:
    """
    Total vehicle capacity = sum over (edge, lane) of (lane_length / spacing_m).

    With spacing_m = 7.5 (avg vehicle + headway), this gives the number of
    vehicles that fit bumper-to-bumper on the express layer. Used to
    normalize occupancy o_X ∈ [0, 1] per N4 / Eq 15.
    """
    net = sumolib.net.readNet(str(net_file))
    total_capacity = 0.0
    for eid in express_edges:
        try:
            e = net.getEdge(eid)
        except Exception:
            continue
        for lane in e.getLanes():
            total_capacity += lane.getLength() / spacing_m
    return max(1.0, total_capacity)


# ============================================================================
# Gate edge detection
# ============================================================================

def build_gate_edges(
    net_file: str,
    express_edges: Set[str],
) -> Set[str]:
    """
    Gate edges = surface edges that have at least one outgoing connection
    into an express edge. Purely topological; no TraCI needed.
    """
    net = sumolib.net.readNet(str(net_file))
    gate_edges: Set[str] = set()
    for eid in express_edges:
        try:
            e = net.getEdge(eid)
        except Exception:
            continue
        for inc_edge in e.getIncoming().keys():
            iid = inc_edge.getID()
            if iid and (iid not in express_edges) and (not iid.startswith(":")):
                gate_edges.add(iid)
    return gate_edges


def build_exit_gate_edges(
    net_file: str,
    express_edges: Set[str],
) -> Set[str]:
    """
    Exit gate edges = surface edges that are targets of at least one outgoing
    connection from an express edge.

    Used by PWMP to detect discharge movements (edges leaving the express
    layer back to the surface) — needed for the I_dis indicator in Eq 10.
    """
    net = sumolib.net.readNet(str(net_file))
    exit_edges: Set[str] = set()
    for eid in express_edges:
        try:
            e = net.getEdge(eid)
        except Exception:
            continue
        for out_e in e.getOutgoing().keys():
            oid = out_e.getID()
            if oid and (oid not in express_edges) and (not oid.startswith(":")):
                exit_edges.add(oid)
    return exit_edges


# ============================================================================
# TLS footprint
# ============================================================================

def _tls_position(tls_id: str, net: "sumolib.net.Net") -> Tuple[float, float]:
    """Position of a TLS, preferring its node coords and falling back to
    centroid of controlled edges' endpoints."""
    try:
        node = net.getNode(tls_id)
        return node.getCoord()
    except Exception:
        pass
    # Fallback: centroid via TLS-controlled connections
    pts: List[Tuple[float, float]] = []
    for tls in net.getTrafficLights():
        if tls.getID() != tls_id:
            continue
        for conn in tls.getConnections():
            from_lane, to_lane, _link = conn[:3]
            for lane in (from_lane, to_lane):
                try:
                    e = lane.getEdge()
                    pts.append(e.getFromNode().getCoord())
                    pts.append(e.getToNode().getCoord())
                except Exception:
                    pass
    if not pts:
        return (0.0, 0.0)
    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)
    return (cx, cy)


def tls_ids_within_radius(net_file: str, anchor_tls: str, radius_m: float) -> List[str]:
    """
    Return TLS IDs whose position is within `radius_m` of the anchor TLS.
    Sorted by distance from anchor.

    Used to constrain the MP footprint to the Yan'an interchange instead of
    controlling all 177 TLSs in the greater network.
    """
    net = sumolib.net.readNet(str(net_file))
    # Collect all TLS IDs in the network
    tls_ids: Set[str] = set()
    for tls in net.getTrafficLights():
        tls_ids.add(tls.getID())
    # Junction nodes with type=traffic_light are TLS IDs too
    for node in net.getNodes():
        if node.getType() == "traffic_light":
            tls_ids.add(node.getID())

    anchor = _tls_position(anchor_tls, net)
    if anchor == (0.0, 0.0):
        raise RuntimeError(f"Could not locate anchor TLS '{anchor_tls}' in net {net_file}")

    within: List[Tuple[str, float]] = []
    for tid in tls_ids:
        pos = _tls_position(tid, net)
        if pos == (0.0, 0.0):
            continue
        d = math.hypot(pos[0] - anchor[0], pos[1] - anchor[1])
        if d <= radius_m:
            within.append((tid, d))
    within.sort(key=lambda x: x[1])
    return [tid for tid, _ in within]


# ============================================================================
# Summary printer — helpful startup diagnostic
# ============================================================================

def print_network_summary(net_file: str, express_edges: Set[str], gate_edges: Set[str],
                          exit_gate_edges: Set[str], capacity: float, tls_ids: List[str]) -> None:
    print(f"[net] express edges: {len(express_edges)}")
    print(f"[net] entry gate edges: {len(gate_edges)}")
    print(f"[net] exit gate edges: {len(exit_gate_edges)}")
    print(f"[net] express vehicle capacity: {capacity:.1f}")
    print(f"[net] controlled TLSs: {len(tls_ids)}")
    if len(express_edges) < 10:
        print(f"[net] WARNING: very few express edges — verify EXPRESS_MIN_SPEED "
              f"and EXPRESS_MIN_Z / EXPRESS_MAX_Z match your network.")
