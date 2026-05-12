"""
controllers/max_pressure.py — Sec V-C, Eq S3-S4.

Classical Max Pressure: select the phase with maximum sum of movement
pressures, subject to min-green.

The key correction vs prior code (AUDIT W6): uses movement_queue (sum over
the lanes that serve the movement) instead of edge_queue for upstream.
Downstream still uses whole-edge counts.
"""

from __future__ import annotations
from typing import Dict, Optional

import traci

from ..standard_blocks import choose_phase
from .base import SignalController, movement_queue, edge_queue


class MaxPressureController(SignalController):
    def name(self) -> str:
        return "MaxPressure"

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
                rho_list = m["turn_probs"]
                movs.append({"q_up": q_up, "q_down_list": q_down_list, "rho_list": rho_list})
            phase_to_movements[phase_idx] = movs

        try:
            return choose_phase(phase_to_movements, pwmp=False)
        except ValueError:
            return None
