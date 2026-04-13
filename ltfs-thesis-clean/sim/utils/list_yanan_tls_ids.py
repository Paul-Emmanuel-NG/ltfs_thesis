# list_yanan_tls_ids.py
# Outputs TLS IDs near an anchor TLS (Yan’an interchange center).
# Works directly on the .net.xml (no TraCI needed).

import csv
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, Tuple, Set, List, Optional

UTILS_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = UTILS_DIR.parents[1]
NET_FILE = str(PROJECT_ROOT / "sim" / "network" / "yanan_elevated.net.xml")
ANCHOR_TLS = "cluster_479314640_850287516"   # your known Yan’an center TLS
RADIUS_M = 400.0                             # tune (e.g., 200–800)
OUT_CSV = str(PROJECT_ROOT / "outputs" / "raw" / "yanan_tls_ids.csv")


def _dist(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def load_net_positions(net_file: str):
    """
    Returns:
      node_pos: node/junction id -> (x, y)
      junction_type: junction id -> type
      edge_ends: edge id -> (from_node, to_node)
      tls_ids: set of tls IDs seen in <connection tl="..."> and traffic_light junctions
      tls_edges: tls id -> set of edges referenced by its controlled connections
    """
    tree = ET.parse(net_file)
    root = tree.getroot()

    node_pos: Dict[str, Tuple[float, float]] = {}
    junction_type: Dict[str, str] = {}

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

    edge_ends: Dict[str, Tuple[str, str]] = {}
    for e in root.findall("edge"):
        eid = e.get("id")
        if not eid or eid.startswith(":"):
            continue
        fr = e.get("from")
        to = e.get("to")
        if fr and to:
            edge_ends[eid] = (fr, to)

    tls_ids: Set[str] = set()
    tls_edges: Dict[str, Set[str]] = {}

    # TLS IDs appear on connections as tl="..."
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

    # Also treat traffic_light junction IDs as TLS IDs (common in OSM/netconvert outputs)
    for jid, t in junction_type.items():
        if t == "traffic_light":
            tls_ids.add(jid)

    return node_pos, edge_ends, tls_ids, tls_edges


def tls_position(
    tls_id: str,
    node_pos: Dict[str, Tuple[float, float]],
    edge_ends: Dict[str, Tuple[str, str]],
    tls_edges: Dict[str, Set[str]],
) -> Optional[Tuple[float, float]]:
    """
    Prefer direct node/junction position if tls_id exists as a junction id.
    Fallback: centroid of endpoints of edges referenced by the TLS's controlled connections.
    """
    if tls_id in node_pos:
        return node_pos[tls_id]

    pts: List[Tuple[float, float]] = []
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


def main():
    node_pos, edge_ends, tls_ids, tls_edges = load_net_positions(NET_FILE)

    anchor_xy = tls_position(ANCHOR_TLS, node_pos, edge_ends, tls_edges)
    if anchor_xy is None:
        raise RuntimeError(f"Could not locate anchor TLS '{ANCHOR_TLS}' in net: {NET_FILE}")

    within: List[Tuple[str, float]] = []
    for tid in sorted(tls_ids):
        xy = tls_position(tid, node_pos, edge_ends, tls_edges)
        if xy is None:
            continue
        d = _dist(anchor_xy, xy)
        if d <= RADIUS_M:
            within.append((tid, d))

    within.sort(key=lambda x: x[1])

    print(f"Anchor TLS: {ANCHOR_TLS} at {anchor_xy}")
    print(f"Radius: {RADIUS_M} m")
    print(f"Found {len(within)} TLS IDs within radius:\n")
    for tid, d in within:
        print(f"  {tid}   (distance={d:.1f} m)")

    with open(OUT_CSV, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["tls_id", "distance_m", "anchor_tls", "radius_m", "net_file"])
        for tid, d in within:
            w.writerow([tid, f"{d:.3f}", ANCHOR_TLS, RADIUS_M, NET_FILE])

    print(f"\nSaved: {OUT_CSV}")


if __name__ == "__main__":
    main()
