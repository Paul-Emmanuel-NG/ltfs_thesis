"""
controllers/actuated.py — Sec V-B.

Gap-based actuated control via TraCI: extend green while vehicles are present
on the approach; terminate to next phase when gap expires or max green hit.

Because the Yan'an network's TLSs are type="static", we can't rely on SUMO's
native actuated logic. We implement a minimal version directly.
"""

from __future__ import annotations
from typing import Dict, Optional

import traci

from .base import SignalController, movement_queue


class ActuatedController(SignalController):
    def __init__(self, gap_s: float, min_green: float, max_green: float):
        self.gap_s = gap_s
        self.min_green = min_green
        self.max_green = max_green
        # Track last time each (tls, phase) had vehicles on active approaches
        self._last_demand: Dict[str, float] = {}

    def name(self) -> str:
        return "Actuated"

    def decide(self, tls_id: str, t: float, tls_config: dict) -> Optional[int]:
        cfg = tls_config.get(tls_id)
        if cfg is None:
            return None

        try:
            current_phase = traci.trafficlight.getPhase(tls_id)
            phase_duration = traci.trafficlight.getPhaseDuration(tls_id)
        except traci.TraCIException:
            return None

        movements = cfg["phases"].get(current_phase, [])
        has_demand = False
        for mov in movements:
            if movement_queue(mov["up_lanes"]) > 0:
                has_demand = True
                self._last_demand[f"{tls_id}:{current_phase}"] = t
                break

        key = f"{tls_id}:{current_phase}"
        time_since_demand = t - self._last_demand.get(key, t)

        # If there's current demand or we're within the gap window, keep current.
        if has_demand or time_since_demand < self.gap_s:
            return None

        # Otherwise advance to the next non-empty phase.
        phases = sorted(cfg["phases"].keys())
        if current_phase not in phases:
            return phases[0] if phases else None
        idx = phases.index(current_phase)
        for i in range(1, len(phases) + 1):
            nxt = phases[(idx + i) % len(phases)]
            if cfg["phases"][nxt]:
                return nxt
        return None
