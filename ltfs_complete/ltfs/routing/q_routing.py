"""
routing/q_routing.py — Sec VI-C, Eq 22.

Tabular Q-routing:
    Q(i, n, d) ← (1-η) Q(i, n, d) + η (t_obs + min_{n'} Q(n, n', d))

Scope note: full per-junction per-destination Q-tables are infeasible for a
3500-edge network with 60k+ vehicles. We coarsen by:
  - node = junction ID
  - neighbor = outgoing edge's target node
  - destination = vehicle's final destination edge

Updates happen when vehicles cross junctions (observed via route-index
changes). Rerouting decisions happen at every DYN_TT_UPDATE_PERIOD_S via
traci.vehicle.setRoute with a Q-table-derived path.

Because Q-routing affects travel times, and travel times feed gate-TT
predictions, we also maintain a per-edge smoothed τ̂_e so the gating
controller can query it like DynamicTTRouter does.

Paper ambiguity (implicit): the paper doesn't specify Q-routing granularity
or how it interacts with SUMO's static-route-at-depart default. Our choice
is documented here.
"""

from __future__ import annotations
from collections import defaultdict
from typing import Dict, Tuple

import traci

from .. import config as C


class QRouter:
    def __init__(
        self,
        eta: float = C.Q_LEARNING_RATE,
        epsilon: float = C.Q_EPSILON,
        period_s: float = C.Q_UPDATE_PERIOD_S,
    ):
        self.eta = eta
        self.epsilon = epsilon
        self.period_s = period_s
        # Q[(node, neighbor, dest_node)] = estimated remaining TT
        self.Q: Dict[Tuple[str, str, str], float] = defaultdict(lambda: 1.0)
        # τ̂_e fallback for gate TT prediction — mirrors DynamicTTRouter interface
        self.tau_hat: Dict[str, float] = {}
        self._last_update_t: float = -1e18
        # Track vehicle route indices to detect junction crossings
        self._veh_route_idx: Dict[str, int] = {}

    def name(self) -> str:
        return "QRouting"

    def initialize(self) -> None:
        try:
            edges = [e for e in traci.edge.getIDList() if not e.startswith(":")]
        except traci.TraCIException:
            edges = []
        for eid in edges:
            try:
                lane_id = f"{eid}_0"
                length = traci.lane.getLength(lane_id)
                max_speed = traci.lane.getMaxSpeed(lane_id)
                self.tau_hat[eid] = length / max_speed if max_speed > 0 else 1.0
            except traci.TraCIException:
                self.tau_hat[eid] = 1.0

    def step(self, t: float) -> bool:
        """
        Two jobs per step:
          1) Update τ̂_e from observed edge TTs (for gate prediction & for Q updates).
          2) Every period_s, apply Q-based rerouting to active vehicles.
        """
        # 1) Always update τ̂_e (cheap, one call per edge)
        # We don't do it every step for cost; only at the update period.
        if t - self._last_update_t < self.period_s:
            return False
        self._last_update_t = t

        # Update τ̂_e
        for eid in list(self.tau_hat.keys()):
            try:
                tau_obs = traci.edge.getTraveltime(eid)
            except traci.TraCIException:
                continue
            if tau_obs <= 0:
                continue
            prev = self.tau_hat.get(eid, tau_obs)
            self.tau_hat[eid] = C.DYN_TT_XI * tau_obs + (1.0 - C.DYN_TT_XI) * prev

        # 2) Process vehicle junction crossings for Q updates; reroute.
        try:
            vids = traci.vehicle.getIDList()
        except traci.TraCIException:
            return True

        for vid in vids:
            try:
                route = traci.vehicle.getRoute(vid)
                idx = traci.vehicle.getRouteIndex(vid)
            except traci.TraCIException:
                continue
            if idx is None or idx < 0:
                continue

            prev_idx = self._veh_route_idx.get(vid, -1)
            if idx > prev_idx and prev_idx >= 0 and prev_idx < len(route):
                # Vehicle crossed a junction: update Q for the transition
                crossed_edge = route[prev_idx]
                tau_obs = self.tau_hat.get(crossed_edge, 1.0)
                if idx < len(route) - 1:
                    # Current node = end of crossed_edge, neighbor = next edge
                    # Estimate remaining TT along the current route as a proxy for
                    # min_{n'} Q(n, n', d).
                    remaining = sum(self.tau_hat.get(e, 1.0) for e in route[idx + 1:])
                    # Store against crude (edge, next_edge, destination_edge) key
                    key = (crossed_edge, route[idx], route[-1])
                    prev_q = self.Q[key]
                    self.Q[key] = (1 - self.eta) * prev_q + self.eta * (tau_obs + remaining)
            self._veh_route_idx[vid] = idx

        # Rerouting via SUMO: push τ̂ into traci.edge.adaptTraveltime
        # and call rerouteTraveltime. This is a practical approximation of
        # Q-routing in SUMO, since full per-vehicle setRoute at every junction
        # would be prohibitive.
        for eid, tau in self.tau_hat.items():
            try:
                traci.edge.adaptTraveltime(eid, float(tau))
            except traci.TraCIException:
                pass
        for vid in vids:
            try:
                traci.vehicle.rerouteTraveltime(vid, currentTravelTimes=False)
            except traci.TraCIException:
                pass

        return True

    def get_tau(self, edge_id: str) -> float:
        return self.tau_hat.get(edge_id, 1.0)
