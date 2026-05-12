"""
standard_blocks.py — Standard (non-LTFS-specific) building blocks.

Maps to Appendix A S1-S9 of the paper. All formulas here are established in
the literature; nothing is claimed as new.

Key correction vs prior code: the split-positive form

    π^PWMP = w · [π]⁺ + [π]⁻

is the only way movement weights can interact with pressure (Eq 11 and Eq 18
in the paper). When w ≡ 1, this reduces exactly to classical π, which is
what the paper's "strict generalization" claim relies on (Sec V-D).

The prior code used w · π (full pressure, signed), which is mathematically
different and breaks the generalization claim. See AUDIT.md W1.
"""

from __future__ import annotations
from typing import Dict, List


def movement_pressure(q_up: float, q_down_list: List[float], rho_list: List[float]) -> float:
    """
    Classical movement pressure (Eq 7, S3):

        π_{i,m}(t) = q_{i,m}(t) − Σ_e ρ_{m,e} · q_e(t)

    q_up        Upstream movement queue (vehicles)
    q_down_list Downstream link queues (one per exit)
    rho_list    Turning ratios (one per exit); ∑ρ = 1 by convention
    """
    if len(q_down_list) != len(rho_list):
        raise ValueError(f"q_down_list/rho_list length mismatch: {len(q_down_list)} vs {len(rho_list)}")
    if not q_down_list:
        return float(q_up)
    weighted_down = sum(r * qd for r, qd in zip(rho_list, q_down_list))
    return float(q_up - weighted_down)


def phase_pressure_mp(movements: List[Dict]) -> float:
    """
    Classical Max Pressure phase pressure (Eq S4 / baseline of Eq 12):

        P^MP_i(φ, t) = Σ_{m∈mov(φ)} π_{i,m}(t)

    No weighting — this is the pure MP phase score for the A0–A5 variants.

    Each movement dict must include: "q_up", "q_down_list", "rho_list".
    """
    total = 0.0
    for mov in movements:
        total += movement_pressure(mov["q_up"], mov["q_down_list"], mov["rho_list"])
    return float(total)


def phase_pressure_pwmp(movements: List[Dict]) -> float:
    """
    PWMP phase pressure (Eq 11, 12, 18):

        π^PWMP_{i,m}(t) = w_m(t) · [π_{i,m}(t)]⁺ + [π_{i,m}(t)]⁻
        P^PWMP_i(φ, t)  = Σ_{m∈mov(φ)} π^PWMP_{i,m}(t)

    The split-positive form means:
      - Weights amplify movements with accumulating upstream demand (positive π).
      - Weights do NOT scale backpressure against spillback (negative π).

    Each movement dict must include: "q_up", "q_down_list", "rho_list", and "w"
    (movement weight ≥ 1; defaults to 1.0 if absent — matches classical MP exactly).
    """
    total = 0.0
    for mov in movements:
        w = float(mov.get("w", 1.0))
        if w < 1.0:
            raise ValueError(f"PWMP weight must be ≥ 1 (got {w}); see Eq 10 clip range.")
        pi = movement_pressure(mov["q_up"], mov["q_down_list"], mov["rho_list"])
        pi_pos = max(pi, 0.0)
        pi_neg = min(pi, 0.0)
        total += w * pi_pos + pi_neg
    return float(total)


def choose_phase(phase_to_movements: Dict[int, List[Dict]], pwmp: bool = False) -> int:
    """
    Select the phase index with maximum pressure.

    pwmp=False  → classical MP phase score (A0–A5 variants).
    pwmp=True   → PWMP phase score with split-positive weighting (A6–A8 variants).

    Ties: smallest phase index wins (stable).
    """
    if not phase_to_movements:
        raise ValueError("phase_to_movements is empty")

    score_fn = phase_pressure_pwmp if pwmp else phase_pressure_mp

    best_phase = None
    best_pressure = None
    for phase_idx, movements in sorted(phase_to_movements.items()):
        p = score_fn(movements)
        if best_pressure is None or p > best_pressure:
            best_phase = phase_idx
            best_pressure = p
    return int(best_phase)


# ============================================================================
# Reliability (S8, Eq 29)
# ============================================================================

def buffer_index(tt_list: List[float]) -> tuple[float, float, float]:
    """
    Eq 29: BI = (T_95 − T_50) / T_50

    Returns (T_50, T_95, BI). NaN if list is empty.
    """
    import math
    import statistics
    if not tt_list:
        return float("nan"), float("nan"), float("nan")
    vals = sorted(tt_list)
    n = len(vals)
    # Linear-interp percentile
    def perc(p):
        if n == 1:
            return vals[0]
        k = (n - 1) * p
        f = math.floor(k)
        c = math.ceil(k)
        if f == c:
            return vals[int(k)]
        return vals[f] * (c - k) + vals[c] * (k - f)
    t50 = perc(0.5)
    t95 = perc(0.95)
    bi = (t95 - t50) / t50 if t50 > 0 else float("nan")
    return t50, t95, bi


# ============================================================================
# Fairness (S9)
# ============================================================================

def gini(values: List[float]) -> float:
    """
    Standard Gini coefficient for non-negative values.

    Typically applied to class-wise mean travel times (Sec IX-D), not to
    per-vehicle travel times.
    """
    if not values:
        return float("nan")
    vals = sorted(v for v in values if v >= 0)
    n = len(vals)
    if n == 0:
        return float("nan")
    total = sum(vals)
    if total == 0:
        return 0.0
    cum = 0.0
    for i, v in enumerate(vals, start=1):
        cum += i * v
    return (2.0 * cum) / (n * total) - (n + 1.0) / n


# ============================================================================
# Self-test
# ============================================================================

def _tests():
    # Classical MP
    p1 = movement_pressure(10, [2, 4], [0.5, 0.5])
    assert abs(p1 - 7.0) < 1e-9, p1

    # Classical phase score
    movs = [
        {"q_up": 10, "q_down_list": [2], "rho_list": [1.0]},  # π = 8
        {"q_up": 4, "q_down_list": [1], "rho_list": [1.0]},   # π = 3
    ]
    assert abs(phase_pressure_mp(movs) - 11.0) < 1e-9

    # PWMP with w=1 collapses to MP
    movs_w = [
        {"q_up": 10, "q_down_list": [2], "rho_list": [1.0], "w": 1.0},
        {"q_up": 4, "q_down_list": [1], "rho_list": [1.0], "w": 1.0},
    ]
    assert abs(phase_pressure_pwmp(movs_w) - 11.0) < 1e-9

    # Split-positive: weight only amplifies positive pressure
    movs_split = [
        {"q_up": 10, "q_down_list": [2], "rho_list": [1.0], "w": 2.0},  # π=+8, w·8=16
        {"q_up": 1, "q_down_list": [5], "rho_list": [1.0], "w": 2.0},   # π=−4, unweighted=−4
    ]
    # Expected: 16 + (−4) = 12.  If old (wrong) code: w·(8) + w·(−4) = 16 − 8 = 8.
    result = phase_pressure_pwmp(movs_split)
    assert abs(result - 12.0) < 1e-9, f"split-positive broken: got {result}"

    # choose_phase
    assert choose_phase({0: movs, 1: [{"q_up": 3, "q_down_list": [0], "rho_list": [1.0]}]}) == 0

    # Buffer index
    _, _, bi = buffer_index([10, 20, 30, 40, 50])
    assert bi > 0

    # Gini
    assert abs(gini([1, 1, 1, 1]) - 0.0) < 1e-9
    g = gini([1, 2, 3, 4, 5])
    assert 0 < g < 1

    print("standard_blocks tests passed.")


if __name__ == "__main__":
    _tests()
