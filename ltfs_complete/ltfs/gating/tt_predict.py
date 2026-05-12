"""
gating/tt_predict.py — Sec VII-A, Eq 23, 24.

Computes predicted remaining travel times T_hat_S and T_hat_X using a custom
Dijkstra over the network, because sumolib v1.25's getShortestPath doesn't
accept custom edge-weight functions. This means we use free-flow TT
(length/speed) rather than the router's smoothed tau_hat. For static routing
these are equivalent; for dynamic/Q routing this is an approximation.
"""

from __future__ import annotations
import heapq
from typing import Optional, Protocol, Set

import sumolib


class HasTau(Protocol):
    def get_tau(self, edge_id: str) -> float: ...


class GateTTPredictor:
    def __init__(
        self,
        net_file: str,
        express_edges: Set[str],
        entry_gate_edges: Set[str],
        exit_gate_edges: Set[str],
        router: HasTau,
    ):
        self.net = sumolib.net.readNet(str(net_file))
        self.express_edges = express_edges
        self.entry_gate_edges = entry_gate_edges
        self.exit_gate_edges = exit_gate_edges
        self.router = router
        # Cache free-flow edge TTs (length / speed) for speed
        self._edge_tt: dict[str, float] = {}
        for e in self.net.getEdges():
            spd = e.getSpeed()
            if spd <= 0:
                continue
            self._edge_tt[e.getID()] = e.getLength() / spd

    def _dijkstra(self, origin_edge: str, dest_edge: str, exclude: Set[str]) -> Optional[float]:
        """Shortest-path TT from origin_edge to dest_edge with excluded edges pruned."""
        if origin_edge == dest_edge:
            return 0.0
        try:
            origin = self.net.getEdge(origin_edge)
        except Exception:
            return None

        # Priority queue: (cost_so_far, edge_id)
        pq: list[tuple[float, str]] = [(0.0, origin_edge)]
        visited: dict[str, float] = {}
        while pq:
            cost, eid = heapq.heappop(pq)
            if eid in visited and visited[eid] <= cost:
                continue
            visited[eid] = cost
            if eid == dest_edge:
                return float(cost)
            try:
                e = self.net.getEdge(eid)
            except Exception:
                continue
            for out_edge in e.getOutgoing().keys():
                oid = out_edge.getID()
                if oid in exclude:
                    continue
                if oid.startswith(":"):
                    continue
                edge_cost = self._edge_tt.get(oid, 1.0)
                new_cost = cost + edge_cost
                if oid not in visited or visited[oid] > new_cost:
                    heapq.heappush(pq, (new_cost, oid))
        return None

    def t_hat_surface(self, current_edge: str, dest_edge: str) -> Optional[float]:
        """Shortest-path TT from current to dest, excluding express edges (Eq 23)."""
        return self._dijkstra(current_edge, dest_edge, exclude=self.express_edges)

    def t_hat_express(self, current_edge: str, dest_edge: str) -> Optional[float]:
        """Shortest-path TT from current to dest, express allowed (Eq 24)."""
        return self._dijkstra(current_edge, dest_edge, exclude=set())

    def delta_t(self, current_edge: str, dest_edge: str) -> Optional[float]:
        """
        Eq 14: ΔT = T_S − T_X. Positive means express is faster.

        If surface-only path doesn't exist but express does, returns a large
        positive value (treat as "surface infeasible, admit").
        If neither exists, returns None.
        """
        ts = self.t_hat_surface(current_edge, dest_edge)
        tx = self.t_hat_express(current_edge, dest_edge)
        if tx is None:
            return None
        if ts is None:
            return 1e6  # Surface infeasible, definitely admit
        return ts - tx