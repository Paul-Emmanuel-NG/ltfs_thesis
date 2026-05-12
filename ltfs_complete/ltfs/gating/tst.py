"""
gating/tst.py — Sec VII-B, Eq 25, N1.

Time Savings Threshold gate admission:
    admit = (ΔT_v(t) ≥ Θ) AND (o_X(t) ≤ κ)
"""

from __future__ import annotations
from typing import Optional

from .. import config as C
from ..ltfs_blocks import tst_admit


class TSTGate:
    """Stateless TST admission decision."""

    def __init__(self, theta_s: float = C.THETA_S, kappa: float = C.KAPPA):
        self.theta_s = theta_s
        self.kappa = kappa

    def name(self) -> str:
        return "TST"

    _log_count = 0

    def decide(self, delta_t_s: Optional[float], o_X: float) -> bool:
        """
        Return True to admit, False to deny.
        """
        if delta_t_s is None:
            return False
        return tst_admit(delta_t_s, o_X, self.theta_s, self.kappa)