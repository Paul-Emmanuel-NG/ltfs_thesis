"""
controllers/fixed_time.py — Sec V-A.

SUMO-native fixed-time: we let SUMO run its loaded traffic-light program
without TraCI overrides. decide() always returns None (keep current),
which means the TLS program advances on its own schedule.

This matches "fixed time as defined in the network" in practice. Full
Webster optimization would require lane-volume and saturation estimates
that aren't in the network XML; that's a separate calibration task, flagged
as future work (Ambiguity A1-adjacent).
"""

from __future__ import annotations
from typing import Optional

from .base import SignalController


class FixedTimeController(SignalController):
    def name(self) -> str:
        return "FixedTime"

    def decide(self, tls_id: str, t: float, tls_config: dict) -> Optional[int]:
        # Let SUMO's own timer drive the phase. Returning None signals "no override".
        return None
