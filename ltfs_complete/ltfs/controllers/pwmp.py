"""
controllers/pwmp.py — Sec V-D, Eq 10, 11, 18.

Priority-Weighted Max Pressure. The core novel signal-control contribution
of the paper (N8).

Key details:
  - Uses split-positive pressure form (Eq 11): w·[π]⁺ + [π]⁻
  - Weight construction per Eq 10 with paper-specified clip to [1, w_max]
  - ū_up (Eq 8) aggregates signal-oriented urgency (Eq 6) over upstream vehicles
  - A6/A7/A8 differ in which terms of Eq 10 are active — controlled by Variant flags
"""

from __future__ import annotations
from typing import Dict, List, Optional, Set

import traci

from .. import config as C
from ..ltfs_blocks import (
    class_urgency,
    wait_urgency,
    signal_urgency,
    mean_upstream_urgency,
    pwmp_weight,
)
from ..standard_blocks import choose_phase
from ..variants.ablation import Variant
from .base import SignalController, movement_queue, edge_queue


class PWMPController(SignalController):
    """
    PWMP signal controller. Instantiated per-run with the active variant so
    the correct subset of Eq 10 terms is activated.
    """

    def __init__(
        self,
        variant: Variant,
        express_edges: Set[str],
        gate_entry_edges: Set[str],
        gate_exit_edges: Set[str],
    ):
        self.variant = variant
        self.express_edges = express_edges
        self.gate_entry_edges = gate_entry_edges
        self.gate_exit_edges = gate_exit_edges

        # Shared occupancy state, set by the runner each step
        self.o_X: float = 0.0

    def name(self) -> str:
        return f"PWMP({self.variant.id})"

    def set_occupancy(self, o_X: float) -> None:
        self.o_X = o_X

    # ------------------------------------------------------------------------
    # Movement classification (Eq 9 / Sec V-D.1)
    # ------------------------------------------------------------------------
    def _classify_movement(self, up_edge: str, down_edges: List[str]) -> dict:
        """
        Returns dict of boolean indicators for Eq 10:
          is_gate_feed : up_edge feeds an entry gate (i.e. down_edges contain express)
          is_discharge : up_edge is an exit gate (comes from express, goes to surface)
          is_express_up: up_edge is itself on the express layer
        """
        # gate_feed: surface up, express down (at least one downstream is express)
        is_gate_feed = (
            up_edge not in self.express_edges
            and any(d in self.express_edges for d in down_edges)
        )
        # discharge: up_edge is a surface edge that is fed by the express layer
        is_discharge = up_edge in self.gate_exit_edges
        # express_up: the movement is starting on an express edge
        is_express_up = up_edge in self.express_edges
        return {
            "is_gate_feed": is_gate_feed,
            "is_discharge": is_discharge,
            "is_express_up": is_express_up,
        }

    # ------------------------------------------------------------------------
    # Upstream urgency aggregation (Eq 8 via Eq 6)
    # ------------------------------------------------------------------------
    def _compute_u_up(self, up_lanes: List[str]) -> float:
        """
        Eq 8: ū_up(m, t) = mean of u^signal_v(t) over vehicles queued on m's upstream.

        u^signal_v from Eq 6 combines class urgency (Eq 2) and wait-time
        urgency (Eq 4).

        Returns 0 if no vehicles on upstream or if ū_up is disabled by variant.
        """
        if not self.variant.pwmp_u_up:
            return 0.0

        us: List[float] = []
        for lane in up_lanes:
            try:
                vids = traci.lane.getLastStepVehicleIDs(lane)
            except traci.TraCIException:
                continue
            for vid in vids:
                try:
                    type_id = traci.vehicle.getTypeID(vid)
                    u_class = class_urgency(
                        type_id, C.CLASS_URGENCY, C.CLASS_URGENCY_DEFAULT, C.U_MIN
                    )
                    w_v = traci.vehicle.getAccumulatedWaitingTime(vid)
                    u_wait = wait_urgency(w_v, C.W_REF_S)
                    us.append(signal_urgency(u_class, u_wait, C.LAMBDA_SIGNAL, C.U_MIN))
                except traci.TraCIException:
                    continue
        return mean_upstream_urgency(us)

    # ------------------------------------------------------------------------
    # Phase decision
    # ------------------------------------------------------------------------
    def decide(self, tls_id: str, t: float, tls_config: Dict) -> Optional[int]:
        cfg = tls_config.get(tls_id)
        if cfg is None:
            return None

        phase_to_movements: Dict[int, list] = {}
        for phase_idx, movement_defs in cfg["phases"].items():
            movs = []
            for m in movement_defs:
                q_up = movement_queue(m["up_lanes"])
                q_down_list = [edge_queue(e) for e in m["down_edges"]]

                cls = self._classify_movement(m["up_edge"], m["down_edges"])

                # Apply variant flags: turn off indicators that aren't active
                is_gate = cls["is_gate_feed"] and self.variant.pwmp_i_gate
                is_dis = cls["is_discharge"] and self.variant.pwmp_i_dis
                is_exp = cls["is_express_up"] and self.variant.pwmp_i_exp

                u_up = self._compute_u_up(m["up_lanes"])

                w = pwmp_weight(
                    is_gate_feed=is_gate,
                    is_discharge=is_dis,
                    is_express_up=is_exp,
                    u_up_mean=u_up,
                    o_X=self.o_X,
                    kappa=C.KAPPA,
                    alpha=C.PWMP_ALPHA,
                    beta=C.PWMP_BETA,
                    eta=C.PWMP_ETA,
                    gamma=C.PWMP_GAMMA,
                    w_max=C.PWMP_W_MAX,
                    occupancy_guard_gate=self.variant.pwmp_occupancy_guard_gate,
                )

                movs.append({
                    "q_up": q_up,
                    "q_down_list": q_down_list,
                    "rho_list": m["turn_probs"],
                    "w": w,
                })
            phase_to_movements[phase_idx] = movs

        try:
            return choose_phase(phase_to_movements, pwmp=True)
        except ValueError:
            return None
