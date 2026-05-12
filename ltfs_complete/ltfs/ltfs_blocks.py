"""
ltfs_blocks.py — LTFS-specific primitives (N1–N9).

Paper citations in each function docstring. This file deliberately contains
only pure functions and data — no TraCI imports — so it can be unit-tested
in isolation from SUMO.
"""

from __future__ import annotations
import math
from typing import Dict, Iterable


# ============================================================================
# Clamping helpers
# ============================================================================

def _clamp(x: float, lo: float, hi: float) -> float:
    try:
        v = float(x)
    except (TypeError, ValueError):
        return lo
    if not math.isfinite(v):
        return lo
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def _clamp01(x: float) -> float:
    return _clamp(x, 0.0, 1.0)


# ============================================================================
# Vehicle-class urgency (Eq 2)
# ============================================================================

def class_urgency(type_id: str, mapping: Dict[str, float], default: float, u_min: float) -> float:
    """
    Eq 2: u^class_v = f_class(c(v)), with u^class_v ∈ [u_min, 1] and u_min > 0.

    The nonzero floor (Eq 2 text) prevents any class from being permanently
    excluded.
    """
    try:
        u = float(mapping.get(type_id, default))
    except Exception:
        u = float(default)
    if not math.isfinite(u):
        u = float(default)
    return _clamp(u, u_min, 1.0)


# ============================================================================
# Route-based urgency (Eq 3)
# ============================================================================

def route_urgency(L_rem_m: float, L_ref_m: float) -> float:
    """
    Eq 3: u^route_v(t) = clip_{[0,1]}(L_rem_v(t) / L_ref).

    Vehicles with longer remaining trips receive higher urgency (they benefit
    most from express admission).
    """
    try:
        lr = float(L_rem_m)
        lref = float(L_ref_m)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(lr) or not math.isfinite(lref) or lref <= 0.0:
        return 0.0
    return _clamp01(lr / lref)


# ============================================================================
# Wait-based urgency (Eq 4)
# ============================================================================

def wait_urgency(W_v_s: float, W_ref_s: float) -> float:
    """
    Eq 4: u^wait_v(t) = clip_{[0,1]}(W_v(t) / W_ref).

    Paper default W_ref = 60 s. Used for signal control (Eq 6), NOT gating.
    """
    try:
        w = float(W_v_s)
        wref = float(W_ref_s)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(w) or not math.isfinite(wref) or wref <= 0.0:
        return 0.0
    return _clamp01(w / wref)


# ============================================================================
# Combined urgencies (Eq 5, Eq 6)
# ============================================================================

def gating_urgency(u_class: float, u_route: float, lam: float, u_min: float) -> float:
    """
    Eq 5: u_v(t) = clip_{[u_min, 1]}(λ · u^class + (1-λ) · u^route).

    A+C combination used by UWA gate admission (Eq 15).
    """
    lam = _clamp01(lam)
    u = lam * _clamp(u_class, u_min, 1.0) + (1.0 - lam) * _clamp01(u_route)
    return _clamp(u, u_min, 1.0)


def signal_urgency(u_class: float, u_wait: float, lam_s: float, u_min: float) -> float:
    """
    Eq 6: u^signal_v(t) = clip_{[u_min, 1]}(λ_s · u^class + (1-λ_s) · u^wait).

    A+D combination used by PWMP phase-weight construction (Eq 10, via Eq 8).
    Separate from gating_urgency because signal control responds to local
    wait-time pressure, not remaining-distance (Sec III-C.5 rationale).
    """
    lam = _clamp01(lam_s)
    u = lam * _clamp(u_class, u_min, 1.0) + (1.0 - lam) * _clamp01(u_wait)
    return _clamp(u, u_min, 1.0)


# ============================================================================
# Movement-level urgency aggregation (Eq 8)
# ============================================================================

def mean_upstream_urgency(u_signals: Iterable[float]) -> float:
    """
    Eq 8: ū_up(m, t) = mean over V_m(t) of u^signal_v(t), or 0 if |V_m| = 0.
    """
    us = [u for u in u_signals if u is not None]
    if not us:
        return 0.0
    return float(sum(us) / len(us))


# ============================================================================
# PWMP movement weight (Eq 10)
# ============================================================================

def pwmp_weight(
    is_gate_feed: bool,
    is_discharge: bool,
    is_express_up: bool,
    u_up_mean: float,
    o_X: float,
    kappa: float,
    alpha: float,
    beta: float,
    eta: float,
    gamma: float,
    w_max: float,
    occupancy_guard_gate: bool = True,
) -> float:
    """
    Eq 10:
        w_m(t) = clip_{[1, w_max]}(
            1
            + α · I_gate(m) · I[o_X(t) ≤ κ]     (A8 has the guard, A6/A7 use α always)
            + β · I_dis(m)
            + η · I_exp(m)
            + γ · ū_up(m, t)
        )

    Ablation flags control which terms are active:
      - Pass is_gate_feed=False, is_discharge=False, is_express_up=False for
        no indicators (A5 has none — but A5 uses MP not PWMP, so this fn is not called).
      - Pass u_up_mean=0.0 to disable the urgency-aggregation term (A6).
      - Set occupancy_guard_gate=False for A6/A7 (no guard); True for A8.

    By construction the returned weight is in [1, w_max].
    """
    o = _clamp01(o_X)
    k = _clamp01(kappa)
    u_mean = _clamp01(u_up_mean)

    if is_gate_feed:
        if occupancy_guard_gate:
            gate_term = float(alpha) if o <= k else 0.0
        else:
            gate_term = float(alpha)
    else:
        gate_term = 0.0

    w = (
        1.0
        + gate_term
        + (float(beta) if is_discharge else 0.0)
        + (float(eta) if is_express_up else 0.0)
        + float(gamma) * u_mean
    )
    if not math.isfinite(w):
        w = 1.0
    return _clamp(w, 1.0, float(w_max))


# ============================================================================
# Predicted time saving (Eq 14, 25)
# ============================================================================

def delta_t(t_hat_surface: float, t_hat_express: float) -> float:
    """
    Eq 14/25: ΔT_v(t) = T̂_S(v, t) - T̂_X(v, t).

    Positive means express is faster. See gating/tt_predict.py for how
    T̂_S and T̂_X are computed (Eq 23, 24).
    """
    try:
        ts = float(t_hat_surface)
        tx = float(t_hat_express)
    except (TypeError, ValueError):
        return 0.0
    if not (math.isfinite(ts) and math.isfinite(tx)):
        return 0.0
    return ts - tx


# ============================================================================
# Gate admission rules (Eq 15, 25, 26)
# ============================================================================

def tst_admit(delta_t_s: float, o_X: float, theta_s: float, kappa: float) -> bool:
    """
    Eq 25 / N1:
        admit = (ΔT ≥ Θ) AND (o_X ≤ κ)
    """
    try:
        dt = float(delta_t_s)
        th = max(0.0, float(theta_s))
    except (TypeError, ValueError):
        return False
    if not math.isfinite(dt):
        return False
    if _clamp01(o_X) > _clamp01(kappa):
        return False
    return dt >= th


def uwa_admit(u_v: float, delta_t_s: float, o_X: float, theta_u_s: float, kappa: float) -> bool:
    """
    Eq 15 / Eq 26 / N2:
        admit = (u_v · ΔT ≥ Θ_u) AND (o_X ≤ κ)

    Because u_v ∈ [u_min, 1] with u_min > 0, no class is permanently excluded
    — minimum-urgency vehicles just face effective threshold Θ_u / u_min.
    """
    try:
        u = float(u_v)
        dt = float(delta_t_s)
        th = max(0.0, float(theta_u_s))
    except (TypeError, ValueError):
        return False
    if not (math.isfinite(u) and math.isfinite(dt)):
        return False
    if _clamp01(o_X) > _clamp01(kappa):
        return False
    return u * dt >= th


# ============================================================================
# Express occupancy dynamics (N4)
# ============================================================================

def update_occupancy(o_prev: float, inflow: int, outflow: int, capacity_veh: float) -> float:
    """
    N4: o_X(t+1) = clip_{[0,1]}( o_X(t) + (inflow - outflow) / capacity )

    capacity_veh: express-layer vehicle-capacity (total vehicle slots across
    all express lanes, from express_capacity_from_net).

    See Ambiguity A2 in AUDIT.md for the capacity normalization choice.
    """
    op = _clamp01(o_prev)
    try:
        cap = float(capacity_veh)
    except (TypeError, ValueError):
        return op
    if not math.isfinite(cap) or cap <= 0.0:
        return op
    inflow_i = max(0, int(inflow))
    outflow_i = max(0, int(outflow))
    delta = (inflow_i - outflow_i) / cap
    return _clamp01(op + delta)


# ============================================================================
# Tests
# ============================================================================

def _tests():
    # u_min floor
    u = class_urgency("unknown_type", {"bus": 0.9}, default=0.0, u_min=0.1)
    assert u == 0.1, u

    # route urgency
    assert route_urgency(1500, 3000) == 0.5
    assert route_urgency(5000, 3000) == 1.0
    assert route_urgency(0, 3000) == 0.0

    # wait urgency
    assert wait_urgency(30, 60) == 0.5
    assert wait_urgency(120, 60) == 1.0

    # gating urgency keeps u_min floor
    u = gating_urgency(u_class=0.1, u_route=0.0, lam=0.6, u_min=0.1)
    assert u == 0.1

    # signal urgency ditto
    u = signal_urgency(u_class=0.1, u_wait=0.0, lam_s=0.5, u_min=0.1)
    assert u == 0.1

    # mean_upstream_urgency
    assert abs(mean_upstream_urgency([0.2, 0.4, 0.6]) - 0.4) < 1e-9
    assert mean_upstream_urgency([]) == 0.0

    # PWMP weight: w=1 when no indicators active
    w = pwmp_weight(False, False, False, 0.0, 0.0, 0.85, 0.5, 0.7, 0.2, 0.6, 1.5)
    assert w == 1.0

    # PWMP weight: gate+occupancy guard works
    w_guarded = pwmp_weight(True, False, False, 0.0, 0.9, 0.85,
                            0.5, 0.7, 0.2, 0.6, 1.5, occupancy_guard_gate=True)
    assert w_guarded == 1.0, f"guard should suppress gate amp: {w_guarded}"
    w_unguarded = pwmp_weight(True, False, False, 0.0, 0.9, 0.85,
                              0.5, 0.7, 0.2, 0.6, 1.5, occupancy_guard_gate=False)
    assert w_unguarded == 1.5, f"no guard should give 1+alpha clipped to 1.5: {w_unguarded}"

    # TST
    assert tst_admit(70, 0.5, 60, 0.85) is True
    assert tst_admit(30, 0.5, 60, 0.85) is False
    assert tst_admit(70, 0.9, 60, 0.85) is False

    # UWA floor behavior
    assert uwa_admit(0.1, 400, 0.5, 30, 0.85) is True  # 0.1 * 400 = 40 >= 30
    assert uwa_admit(0.1, 200, 0.5, 30, 0.85) is False  # 0.1 * 200 = 20 < 30

    # Occupancy
    assert abs(update_occupancy(0.0, 50, 0, 100) - 0.5) < 1e-9
    assert update_occupancy(0.9, 30, 0, 100) == 1.0  # clamped

    print("ltfs_blocks tests passed.")


if __name__ == "__main__":
    _tests()
