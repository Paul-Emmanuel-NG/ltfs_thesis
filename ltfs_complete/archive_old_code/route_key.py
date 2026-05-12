# route_key.py
# Create a stable join key for trips even when SUMO routeID is empty.
# Uses the actual edge list from traci.vehicle.getRoute().

from __future__ import annotations
import hashlib
from typing import Iterable, List, Tuple


def normalize_edges(edges: Iterable[str]) -> List[str]:
    """Remove internal edges and keep only normal edge IDs."""
    out: List[str] = []
    for e in edges:
        if not e:
            continue
        if e.startswith(":"):
            continue
        out.append(e)
    return out


def make_route_key(edges: Iterable[str]) -> str:
    """
    A stable key for matching the same route across runs:
    - includes first edge, last edge, length, and a short md5 of the full sequence.
    """
    seq = normalize_edges(edges)
    if not seq:
        return "EMPTY"
    first = seq[0]
    last = seq[-1]
    n = len(seq)
    joined = " ".join(seq).encode("utf-8")
    h = hashlib.md5(joined).hexdigest()[:10]
    return f"{first}->{last}|n={n}|h={h}"


def route_endpoints(edges: Iterable[str]) -> Tuple[str, str]:
    seq = normalize_edges(edges)
    if not seq:
        return ("", "")
    return (seq[0], seq[-1])
