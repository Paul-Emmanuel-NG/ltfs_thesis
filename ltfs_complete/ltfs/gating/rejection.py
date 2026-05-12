"""
gating/rejection.py — Sec VII-D.

Deterministic surface fallback for denied vehicles. Paper requires:
once rejected at any gate, commit to a surface-only path and don't retry
admission at downstream gates.

Uses a custom dijkstra because sumolib v1.25's getShortestPath doesn't
support edge-weight functions.
"""

from __future__ import annotations
import heapq
from typing import List, Set

import sumolib
import traci


class RejectionHandler:
    def __init__(self, net_file: str, express_edges: Set[str]):
        self.net = sumolib.net.readNet(str(net_file))
        self.express_edges = express_edges
        self.denied: Set[str] = set()
        # Cache free-flow edge TTs for dijkstra
        self._edge_tt: dict[str, float] = {}
        for e in self.net.getEdges():
            spd = e.getSpeed()
            if spd <= 0:
                continue
            self._edge_tt[e.getID()] = e.getLength() / spd

    def is_denied(self, vid: str) -> bool:
        return vid in self.denied

    def _dijkstra_surface(self, origin_edge: str, dest_edge: str) -> List[str]:
        """Shortest surface-only path as a list of edge IDs. Empty if none."""
        if origin_edge == dest_edge:
            return [origin_edge]

        # Priority queue: (cost, edge_id)
        pq: list[tuple[float, str]] = [(0.0, origin_edge)]
        prev: dict[str, str] = {}
        visited: dict[str, float] = {}
        found = False
        while pq:
            cost, eid = heapq.heappop(pq)
            if eid in visited and visited[eid] <= cost:
                continue
            visited[eid] = cost
            if eid == dest_edge:
                found = True
                break
            try:
                e = self.net.getEdge(eid)
            except Exception:
                continue
            for out_edge in e.getOutgoing().keys():
                oid = out_edge.getID()
                if oid in self.express_edges:
                    continue
                if oid.startswith(":"):
                    continue
                edge_cost = self._edge_tt.get(oid, 1.0)
                new_cost = cost + edge_cost
                if oid not in visited or visited[oid] > new_cost:
                    prev[oid] = eid
                    heapq.heappush(pq, (new_cost, oid))

        if not found:
            return []

        # Reconstruct path
        path = [dest_edge]
        cur = dest_edge
        while cur in prev:
            cur = prev[cur]
            path.append(cur)
        path.reverse()
        return path

    def mark_and_reroute(self, vid: str, router_tau) -> bool:
        """Mark vid denied. Reroute to surface-only path. Returns True if rerouted."""
        if vid in self.denied:
            return False

        try:
            route = list(traci.vehicle.getRoute(vid))
            current_edge = traci.vehicle.getRoadID(vid)
        except traci.TraCIException:
            return False
        if not route or not current_edge or current_edge.startswith(":"):
            self.denied.add(vid)
            return False

        dest_edge = route[-1]
        new_route = self._dijkstra_surface(current_edge, dest_edge)
        self.denied.add(vid)
        if len(new_route) < 2:
            return False
        try:
            traci.vehicle.setRoute(vid, new_route)
            return True
        except traci.TraCIException:
            return False

    def cleanup(self, arrived_ids: List[str]) -> None:
        for vid in arrived_ids:
            self.denied.discard(vid)