"""
routing/dynamic_tt.py — Sec VI-B, Eq 21.

Exponentially-smoothed link travel times, periodic shortest-path rerouting.

    τ̂_e(t) = ξ · τ_obs(t) + (1-ξ) · τ̂_e(t-1)

Vehicles are rerouted every DYN_TT_UPDATE_PERIOD_S via
traci.vehicle.rerouteTraveltime, which uses SUMO's internal shortest path on
the edge-travel-time map we update.
"""

from __future__ import annotations
from typing import Dict

import traci

from .. import config as C


class DynamicTTRouter:
    """
    Maintains exponentially-smoothed link TT estimates and triggers periodic
    rerouting. Also serves as the source of τ̂_e for gate TT prediction
    (gating/tt_predict.py).
    """

    def __init__(self, xi: float = C.DYN_TT_XI, period_s: float = C.DYN_TT_UPDATE_PERIOD_S):
        self.xi = xi
        self.period_s = period_s
        self.tau_hat: Dict[str, float] = {}
        self._last_update_t: float = -1e18
        self._edges: list[str] = []

    def name(self) -> str:
        return "DynamicTT"

    def initialize(self) -> None:
        """Call once after traci.start(). Seeds τ̂_e with free-flow times."""
        try:
            self._edges = [e for e in traci.edge.getIDList() if not e.startswith(":")]
        except traci.TraCIException:
            self._edges = []

        for eid in self._edges:
            try:
                # Free-flow time: length / speed. Use the first lane.
                lanes = traci.edge.getLaneNumber(eid)
                if lanes > 0:
                    lane_id = f"{eid}_0"
                    length = traci.lane.getLength(lane_id)
                    max_speed = traci.lane.getMaxSpeed(lane_id)
                    ff = length / max_speed if max_speed > 0 else 1.0
                else:
                    ff = 1.0
            except traci.TraCIException:
                ff = 1.0
            self.tau_hat[eid] = ff

    def step(self, t: float) -> bool:
        """
        Call each simulation step. Returns True if an update was performed
        (for logging / diagnostic use).
        """
        if t - self._last_update_t < self.period_s:
            return False
        self._last_update_t = t

        # 1) Update smoothed TTs from observations
        for eid in self._edges:
            try:
                tau_obs = traci.edge.getTraveltime(eid)
            except traci.TraCIException:
                continue
            if tau_obs <= 0:
                continue
            prev = self.tau_hat.get(eid, tau_obs)
            self.tau_hat[eid] = self.xi * tau_obs + (1.0 - self.xi) * prev

        # 2) Push τ̂ into SUMO's routing table and reroute active vehicles
        for eid, tau in self.tau_hat.items():
            try:
                traci.edge.adaptTraveltime(eid, float(tau))
            except traci.TraCIException:
                pass

        try:
            for vid in traci.vehicle.getIDList():
                # rerouteTraveltime uses traci.edge.adaptTraveltime values
                try:
                    traci.vehicle.rerouteTraveltime(vid, currentTravelTimes=False)
                except traci.TraCIException:
                    pass
        except traci.TraCIException:
            pass

        return True

    def get_tau(self, edge_id: str) -> float:
        """Return current smoothed TT for edge_id (for gate TT prediction)."""
        return self.tau_hat.get(edge_id, 1.0)
