"""
gating/uwa.py — Sec VII-C, Eq 15, 26, N2.

Urgency-weighted admission: u_v(t) · ΔT_v(t) ≥ Θ_u AND o_X(t) ≤ κ.

u_v(t) is computed from class urgency (Eq 2) and/or route urgency (Eq 3),
combined per Eq 5, per the Variant's urgency_mode.
"""

from __future__ import annotations
from typing import Optional

import traci

from .. import config as C
from ..ltfs_blocks import class_urgency, route_urgency, gating_urgency, uwa_admit
from ..variants.ablation import Variant


class UWAGate:
    """
    UWA admission with urgency computed per the active variant's mode:
      - "class"        → u_v = u^class
      - "route"        → u_v = u^route
      - "class_route"  → u_v = Eq 5 combination
      - "none"         → reduces to TST (u_v = 1)
    """

    def __init__(
        self,
        variant: Variant,
        theta_u_s: float = C.THETA_U_S,
        kappa: float = C.KAPPA,
        l_ref_m: float = C.L_REF_M,
    ):
        self.variant = variant
        self.theta_u_s = theta_u_s
        self.kappa = kappa
        self.l_ref_m = l_ref_m

    def name(self) -> str:
        return f"UWA({self.variant.urgency_mode})"

    def _compute_urgency(self, vid: str) -> float:
        """
        Compute u_v(t) for this vehicle per the variant's urgency_mode.
        """
        mode = self.variant.urgency_mode

        try:
            type_id = traci.vehicle.getTypeID(vid)
        except traci.TraCIException:
            type_id = ""

        u_class_val = class_urgency(
            type_id, C.CLASS_URGENCY, C.CLASS_URGENCY_DEFAULT, C.U_MIN
        )

        u_route_val = 0.0
        if mode in ("route", "class_route"):
            try:
                route = traci.vehicle.getRoute(vid)
                idx = traci.vehicle.getRouteIndex(vid)
            except traci.TraCIException:
                route, idx = [], -1
            if idx is not None and idx >= 0 and idx < len(route):
                L_rem = 0.0
                for e in route[idx:]:
                    if not e or e.startswith(":"):
                        continue
                    try:
                        # length via the first lane
                        L_rem += traci.lane.getLength(f"{e}_0")
                    except traci.TraCIException:
                        pass
                u_route_val = route_urgency(L_rem, self.l_ref_m)

        if mode == "class":
            return u_class_val
        if mode == "route":
            # Keep u_min floor even for pure-route mode
            from ..ltfs_blocks import _clamp
            return _clamp(u_route_val, C.U_MIN, 1.0)
        if mode == "class_route":
            return gating_urgency(u_class_val, u_route_val, C.LAMBDA_GATING, C.U_MIN)
        # "none": equivalent to TST semantics
        return 1.0

    def decide(self, vid: str, delta_t_s: Optional[float], o_X: float) -> bool:
        if delta_t_s is None:
            return False
        u_v = self._compute_urgency(vid)
        return uwa_admit(u_v, delta_t_s, o_X, self.theta_u_s, self.kappa)
