"""
controllers/base.py — Controller interface.

All signal controllers implement the same interface so the runner can swap
between them based on the active variant (A0-A8).
"""

from __future__ import annotations
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Dict, List, Optional, Set

import traci


@dataclass
class TLSState:
    """Per-TLS state tracked by the runner."""
    tls_id: str
    current_phase: int
    time_in_phase: float = 0.0
    last_switch_time: float = 0.0


class SignalController(ABC):
    """
    Interface for surface-layer signal controllers (Sec V).

    Controllers are stateless with respect to sim time — the runner owns
    per-TLS state and calls step() each simulation step. Controllers only
    decide "given this TLS's current observations, what phase should it be in".
    """

    @abstractmethod
    def name(self) -> str:
        ...

    @abstractmethod
    def decide(self, tls_id: str, t: float, tls_config: Dict) -> Optional[int]:
        """
        Return the desired phase index for this TLS, or None to keep current.

        tls_config: the structure built by build_tls_config (below).
        """
        ...


def build_tls_config(tls_ids: List[str]) -> Dict[str, dict]:
    """
    Build per-TLS configuration from the ACTIVE traffic-light program in SUMO.

    Returns: {tls_id: {"phases": {phase_idx: [movement_dict, ...]}}}

    Each movement_dict is:
      {
        "up_edge": str,
        "up_lanes": [str, ...],        # Lanes on up_edge that serve this movement
        "down_edges": [str, ...],
        "turn_probs": [float, ...]     # Equal-split (Ambiguity A1)
      }

    up_lanes is the key correction vs the prior code, which used the whole
    edge's vehicle count as the queue proxy (AUDIT W6). With up_lanes we can
    compute the movement-level queue.
    """
    tls_config: Dict[str, dict] = {}

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

        phases_dict: Dict[int, list] = {}

        for p_idx, ph in enumerate(active.phases):
            state = ph.state

            # from_edge -> {"up_lanes": set, "down_edges": set}
            from_to: Dict[str, Dict[str, Set[str]]] = {}

            for sg_idx, sg_links in enumerate(controlled_links):
                if sg_idx >= len(state):
                    break
                # Green: 'G' permissive, 'g' protected with right-of-way
                if state[sg_idx] not in ("G", "g"):
                    continue

                for from_lane, to_lane, _via in sg_links:
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
                    entry = from_to.setdefault(from_edge, {"up_lanes": set(), "down_edges": set()})
                    entry["up_lanes"].add(from_lane)
                    entry["down_edges"].add(to_edge)

            movements: List[dict] = []
            for from_edge, payload in from_to.items():
                to_edges = sorted(payload["down_edges"])
                up_lanes = sorted(payload["up_lanes"])
                if not to_edges:
                    continue
                k = len(to_edges)
                movements.append({
                    "up_edge": from_edge,
                    "up_lanes": up_lanes,
                    "down_edges": to_edges,
                    "turn_probs": [1.0 / k] * k,  # Equal-split, Ambiguity A1
                })
            phases_dict[p_idx] = movements

        if phases_dict:
            tls_config[tls_id] = {"phases": phases_dict}

    if not tls_config:
        raise RuntimeError("No TLS config built — check network has TLSs with green phases.")
    return tls_config


def movement_queue(up_lanes: List[str]) -> float:
    """
    Movement-level queue = sum of vehicles on the lanes that serve this
    movement. Correct per Eq 7 (AUDIT W6 fix).
    """
    total = 0.0
    for lane in up_lanes:
        try:
            total += traci.lane.getLastStepVehicleNumber(lane)
        except traci.TraCIException:
            pass
    return total


def edge_queue(edge_id: str) -> float:
    """
    Whole-edge vehicle count, for downstream queue in Eq 7.
    """
    try:
        return float(traci.edge.getLastStepVehicleNumber(edge_id))
    except traci.TraCIException:
        return 0.0
