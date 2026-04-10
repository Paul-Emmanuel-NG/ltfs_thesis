# ltfs_blocks.py
#
# LTFS-specific building blocks for Paper 1:
# - TST ΔT computation
# - TST gate decision
# - Simple express-layer occupancy update
#
# Controllers (mp_ltfs*.py) call these functions; parameters like
# THETA, KAPPA, EXPRESS_CAPACITY live in the controller, not here.

import math


def _clamp01(x: float) -> float:
    """Clamp to [0, 1]. Non-finite becomes 0."""
    try:
        x = float(x)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(x):
        return 0.0
    if x < 0.0:
        return 0.0
    if x > 1.0:
        return 1.0
    return x


def tst_delta_t(tau_surface: float, tau_express: float) -> float:
    """
    Time Saving Threshold (TST) ΔT:

        ΔT = τ_S - τ_X

    Positive ΔT means "express is beneficial".
    """
    try:
        ts = float(tau_surface)
        tx = float(tau_express)
    except (TypeError, ValueError):
        return 0.0

    if not (math.isfinite(ts) and math.isfinite(tx)):
        return 0.0

    return ts - tx


def tst_gate_decision(delta_t: float, o_x: float, theta: float, kappa: float) -> bool:
    """
    TST gate decision:

        admit = (ΔT >= Θ) and (o_X <= κ)

    ΔT: estimated time saving in seconds
    Θ : minimum time saving threshold (seconds)
    o_X: express occupancy in [0, 1]
    κ : occupancy cap in [0, 1]
    """
    # sanitize parameters
    try:
        dt = float(delta_t)
    except (TypeError, ValueError):
        return False
    if not math.isfinite(dt):
        return False

    try:
        th = float(theta)
    except (TypeError, ValueError):
        th = 0.0
    if not math.isfinite(th):
        th = 0.0
    if th < 0.0:
        th = 0.0

    ox = _clamp01(o_x)
    kp = _clamp01(kappa)

    if ox > kp:
        return False
    return dt >= th


def update_occupancy(o_prev: float, inflow: int, outflow: int, capacity: float) -> float:
    """
    Update express corridor occupancy:

        o_next = o_prev + (inflow - outflow) / capacity

    then clamp to [0, 1].
    """
    op = _clamp01(o_prev)

    try:
        cap = float(capacity)
    except (TypeError, ValueError):
        cap = 0.0
    if not math.isfinite(cap) or cap <= 0.0:
        return op

    # counts should never be negative
    try:
        inflow_i = int(inflow)
    except (TypeError, ValueError):
        inflow_i = 0
    try:
        outflow_i = int(outflow)
    except (TypeError, ValueError):
        outflow_i = 0

    if inflow_i < 0:
        inflow_i = 0
    if outflow_i < 0:
        outflow_i = 0

    delta = (float(inflow_i) - float(outflow_i)) / cap
    return _clamp01(op + delta)


# -----------------------
# Simple unit tests for LTFS blocks
# -----------------------

# ----------------------------
# Priority helpers (Paper 1 PWMP + priority-aware LTFS)
# ----------------------------

# Default urgency mapping for the vehicle type IDs defined in the provided OSM route files.
# These are normalized to [0, 1] and are meant to be tuned experimentally.
DEFAULT_TYPE_URGENCY = {
    "bus_bus": 0.9,
    "truck_truck": 0.7,
    "veh_passenger": 0.4,
    "motorcycle_motorcycle": 0.35,
    "bike_bicycle": 0.2,
    "ped_pedestrian": 0.1,
}

def urgency_from_type_id(type_id: str,
                         default: float = 0.4,
                         mapping: dict = None) -> float:
    """Map SUMO vehicle type ID to an urgency/priority score in [0, 1]."""
    if mapping is None:
        mapping = DEFAULT_TYPE_URGENCY
    try:
        u = float(mapping.get(type_id, default))
    except Exception:
        u = float(default)
    return _clamp01(u)

def route_urgency_from_remaining_length(L_rem: float, L_ref: float) -> float:
    """Normalize remaining route length into [0, 1] as a route-based priority signal."""
    try:
        L_rem = float(L_rem)
        L_ref = float(L_ref)
        if not math.isfinite(L_rem) or not math.isfinite(L_ref) or L_ref <= 0.0:
            return 0.0
        return _clamp01(L_rem / L_ref)
    except Exception:
        return 0.0

def combined_urgency(u_class: float,
                     u_route: float,
                     lam: float = 0.6) -> float:
    """Combine class- and route-based urgency. lam in [0,1] weights class urgency."""
    lam = _clamp01(lam)
    u = lam * _clamp01(u_class) + (1.0 - lam) * _clamp01(u_route)
    return _clamp01(u)

def pwmp_weight_full(is_gate_feed: bool,
                     is_discharge: bool,
                     is_express_up: bool,
                     mean_edge_urgency: float,
                     o_x: float,
                     kappa: float,
                     alpha: float = 0.5,
                     beta: float = 0.7,
                     eta: float = 0.2,
                     gamma: float = 0.6,
                     w_max: float = 2.5) -> float:
    """
    Full PWMP movement weight for combining:
      (A) vehicle-class + route-based urgency (via mean_edge_urgency),
      (B) layer priority (express movements and discharge protection),
      (C) gate-feed coupling with occupancy safety guard.

    Weight form:
      w = clip( 1
                + alpha * I_gate_feed * I[o_x <= kappa]
                + beta  * I_discharge
                + eta   * I_express_up
                + gamma * mean_edge_urgency,
                [1, w_max] )

    Notes:
      - We guard only gate-feed amplification when o_x > kappa. Discharge remains prioritized to
        prevent express spillback.
      - All weights are positive and bounded to avoid starving low-priority traffic.
    """
    o_x = _clamp01(o_x)
    mean_u = _clamp01(mean_edge_urgency)
    gate_term = float(alpha) if (is_gate_feed and o_x <= kappa) else 0.0
    w = 1.0         + gate_term         + (float(beta) if is_discharge else 0.0)         + (float(eta) if is_express_up else 0.0)         + float(gamma) * mean_u
    if not math.isfinite(w):
        w = 1.0
    if w < 1.0:
        w = 1.0
    if w > w_max:
        w = w_max
    return float(w)

def _run_tests():
    # L1: ΔT basic
    dt1 = tst_delta_t(100.0, 80.0)
    assert abs(dt1 - 20.0) < 1e-9, f"L1 failed, got {dt1}"

    # L2: occupancy increase
    o1 = update_occupancy(0.0, inflow=50, outflow=0, capacity=100.0)
    assert abs(o1 - 0.5) < 1e-9, f"L2 failed, got {o1}"

    # L3: occupancy decrease and clamp at 0
    o2 = update_occupancy(0.2, inflow=0, outflow=50, capacity=100.0)
    assert abs(o2 - 0.0) < 1e-9, f"L3 failed, got {o2}"

    # L4: occupancy clamp at 1
    o3 = update_occupancy(0.9, inflow=30, outflow=0, capacity=100.0)
    assert abs(o3 - 1.0) < 1e-9, f"L4 failed, got {o3}"

    # L5: gating blocks when occupancy above κ
    allow1 = tst_gate_decision(delta_t=120.0, o_x=0.9, theta=60.0, kappa=0.8)
    assert allow1 is False, f"L5 failed, got {allow1}"

    # L6: gating admits when ΔT >= Θ and occupancy below κ
    allow2 = tst_gate_decision(delta_t=70.0, o_x=0.5, theta=60.0, kappa=0.8)
    assert allow2 is True, f"L6 failed, got {allow2}"

    # L7: gating rejects when ΔT < Θ even if occupancy is low
    allow3 = tst_gate_decision(delta_t=30.0, o_x=0.5, theta=60.0, kappa=0.8)
    assert allow3 is False, f"L7 failed, got {allow3}"

    # Extra: kappa out of range clamps, theta negative clamps
    allow4 = tst_gate_decision(delta_t=1.0, o_x=0.9, theta=-10.0, kappa=2.0)
    assert allow4 is True, f"Clamp test failed, got {allow4}"

    print("All ltfs_blocks tests passed.")


if __name__ == "__main__":
    _run_tests()
