# standard_blocks.py
#
# Core "standard" building blocks for Paper 1:
# - Max Pressure movement and phase definitions
# - Phase selection rule
#
# This is the only place you should define the Max Pressure maths.
# Controllers (baseline, LTFS, RL) call these functions.

from typing import Dict, List


def movement_pressure(q_up: float,
                      q_down_list: List[float],
                      rho_list: List[float]) -> float:
    """
    Compute the pressure of a single movement (approach → set of exits).

        P = q_up - sum_i rho_i * q_down_i

    where:
        q_up        upstream queue (or occupancy proxy)
        q_down_i    downstream queue for exit i
        rho_i       turning probability to exit i

    All inputs are scalars. rho_list and q_down_list must have same length.
    """
    if len(q_down_list) != len(rho_list):
        raise ValueError("q_down_list and rho_list must have same length")

    weighted_down = sum(r * qd for r, qd in zip(rho_list, q_down_list))
    return float(q_up - weighted_down)


def phase_pressure(movements: List[Dict]) -> float:
    """
    Compute the total pressure of a phase as the sum of (optionally) weighted
    movement pressures.

    Each movement dict must include:
        - "q_up": float
        - "q_down_list": List[float]
        - "rho_list": List[float]

    Optional:
        - "w": float (priority weight). If omitted, defaults to 1.0.

    Notes:
        - To preserve Max-Pressure behavior and avoid starvation, weights should
          be positive and bounded, and should not be allowed to reach 0.
    """
    total = 0.0
    for mov in movements:
        w = float(mov.get("w", 1.0))
        if w <= 0.0:
            raise ValueError(f"Movement weight must be positive, got {w}")
        p = movement_pressure(
            mov["q_up"],
            mov["q_down_list"],
            mov["rho_list"],
        )
        total += w * p
    return float(total)


def choose_phase(phase_to_movements: Dict[int, List[Dict]]) -> int:
    """
    Given a mapping:
        phase_index -> list of movement dicts

    return the phase index with the highest phase pressure.
    Ties are broken by choosing the smallest phase index.
    """
    if not phase_to_movements:
        raise ValueError("phase_to_movements is empty")

    best_phase = None
    best_pressure = None

    for phase_idx, movements in phase_to_movements.items():
        p = phase_pressure(movements)
        if best_pressure is None or p > best_pressure or (
            p == best_pressure and (best_phase is None or phase_idx < best_phase)
        ):
            best_phase = phase_idx
            best_pressure = p

    return int(best_phase)


# -----------------------
# Simple unit tests (S1–S9 style)
# -----------------------

def _run_tests():
    # S1: movement_pressure basic
    p1 = movement_pressure(10, [2, 4], [0.5, 0.5])
    assert abs(p1 - 7.0) < 1e-9, f"S1 failed, got {p1}"

    # S2: movement_pressure no downstream queues
    p2 = movement_pressure(5, [], [])
    assert abs(p2 - 5.0) < 1e-9, f"S2 failed, got {p2}"

    # S3: movement_pressure with asymmetric turning
    p3 = movement_pressure(20, [3, 1], [0.7, 0.3])  # 20 - (0.7*3 + 0.3*1) = 20 - 2.4 = 17.6
    assert abs(p3 - 17.6) < 1e-9, f"S3 failed, got {p3}"

    # S4: phase_pressure sum of movements
    movs = [
        {"q_up": 10, "q_down_list": [2], "rho_list": [1.0]},   # 10 - 2 = 8
        {"q_up": 4, "q_down_list": [1], "rho_list": [1.0]},    # 4 - 1 = 3
    ]
    p_phase = phase_pressure(movs)
    assert abs(p_phase - 11.0) < 1e-9, f"S4 failed, got {p_phase}"

    # S5: choose_phase simple two phases
    phase_map = {
        0: [{"q_up": 10, "q_down_list": [2], "rho_list": [1.0]}],  # P = 8
        1: [{"q_up": 5, "q_down_list": [1], "rho_list": [1.0]}],   # P = 4
    }
    chosen = choose_phase(phase_map)
    assert chosen == 0, f"S5 failed, expected 0, got {chosen}"

    # S6: tie breaking, choose smaller index
    phase_map_tie = {
        0: [{"q_up": 10, "q_down_list": [2], "rho_list": [1.0]}],  # P = 8
        1: [{"q_up": 8, "q_down_list": [0], "rho_list": [1.0]}],   # P = 8
    }
    chosen2 = choose_phase(phase_map_tie)
    assert chosen2 == 0, f"S6 failed, expected 0, got {chosen2}"

    # S7: negative pressures allowed
    p_neg = movement_pressure(1, [3], [1.0])  # 1 - 3 = -2
    assert abs(p_neg + 2.0) < 1e-9, f"S7 failed, got {p_neg}"

    # S8: zero queues
    p_zero = movement_pressure(0, [0, 0], [0.5, 0.5])
    assert abs(p_zero - 0.0) < 1e-9, f"S8 failed, got {p_zero}"

    # S9: choose_phase with three phases
    phase_map_3 = {
        0: [{"q_up": 3, "q_down_list": [1], "rho_list": [1.0]}],   # P = 2
        1: [{"q_up": 4, "q_down_list": [1], "rho_list": [1.0]}],   # P = 3
        2: [{"q_up": 2, "q_down_list": [0], "rho_list": [1.0]}],   # P = 2
    }
    chosen3 = choose_phase(phase_map_3)
    assert chosen3 == 1, f"S9 failed, expected 1, got {chosen3}"

    print("All standard_blocks tests (S1–S9) passed.")


if __name__ == "__main__":
    _run_tests()
