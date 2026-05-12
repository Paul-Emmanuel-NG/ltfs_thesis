"""
routing/static_dijkstra.py — Sec VI-A.

Static shortest-path routing. Vehicles use the route assigned at depart by
duarouter and never reroute. This is the baseline (A0-A5 default).

No per-step work; SUMO's default behaviour IS static routing. The class
exists so the runner can uniformly call router.step() regardless of which
routing strategy is active.
"""

from __future__ import annotations


class StaticRouter:
    def name(self) -> str:
        return "Static"

    def initialize(self) -> None:
        return None

    def step(self, t: float) -> bool:
        return False

    def get_tau(self, edge_id: str) -> float:
        """
        Fallback free-flow TT lookup for components that want a τ̂ from the
        router but routing is static. Returns 1.0 as a safe non-zero value.
        """
        return 1.0
