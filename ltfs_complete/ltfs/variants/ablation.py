"""
variants/ablation.py — A0–A8 ablation specification (Table I, Sec VIII-D).

Each variant is a dataclass of boolean flags and string choices consumed by
the runner to wire up the right controllers/gates/urgency combiners.

Quick reference (paper Table I):

  A0  Surface-only, no gate, N/A urgency, MP
  A1  +Express, no gate (open), N/A urgency, MP
  A2  +Express, TST, None urgency, MP
  A3  +Express, UWA, class-only urgency, MP
  A4  +Express, UWA, route-only urgency, MP
  A5  +Express, UWA, class+route urgency, MP
  A6  +Express, UWA, class+route urgency, PWMP (I_gate, I_dis, I_exp, no u_up)
  A7  +Express, UWA, class+route urgency, PWMP (I_gate, I_dis, I_exp, u_up)
  A8  +Express, UWA, class+route urgency, PWMP (I_gate OCC-GUARDED, I_dis, I_exp, u_up)
"""

from __future__ import annotations
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class Variant:
    """Full ablation specification for a single variant."""

    id: str                         # "A0" .. "A8"

    # Layer config
    use_express_layer: bool         # A0 only has False

    # Gating
    gate_rule: str                  # "none" | "open" | "tst" | "uwa"
    urgency_mode: str               # "na" | "none" | "class" | "route" | "class_route"

    # Signal controller
    signal_controller: str          # "mp" | "pwmp"

    # PWMP indicators (Eq 10) — only relevant when signal_controller="pwmp"
    pwmp_i_gate: bool = False
    pwmp_i_dis: bool = False
    pwmp_i_exp: bool = False
    pwmp_u_up: bool = False         # γ · ū_up term (Eq 8)
    pwmp_occupancy_guard_gate: bool = False   # A8-only: guard I_gate by o_X ≤ κ

    # Free-text description
    description: str = ""

    def uses_pwmp(self) -> bool:
        return self.signal_controller == "pwmp"

    def uses_gating(self) -> bool:
        return self.gate_rule in ("tst", "uwa")


ABLATION_SUITE: dict[str, Variant] = {
    "A0": Variant(
        id="A0",
        use_express_layer=False,
        gate_rule="none",
        urgency_mode="na",
        signal_controller="mp",
        description="Surface-only Max Pressure. Baseline reference for single-layer regime.",
    ),
    "A1": Variant(
        id="A1",
        use_express_layer=True,
        gate_rule="open",       # All vehicles admitted if route touches express
        urgency_mode="na",
        signal_controller="mp",
        description="LTFS with gates held open. Tests raw-infrastructure effect.",
    ),
    "A2": Variant(
        id="A2",
        use_express_layer=True,
        gate_rule="tst",
        urgency_mode="none",
        signal_controller="mp",
        description="LTFS + TST gating + MP. First policy-based admission test.",
    ),
    "A3": Variant(
        id="A3",
        use_express_layer=True,
        gate_rule="uwa",
        urgency_mode="class",   # u_v = u^class only
        signal_controller="mp",
        description="LTFS + UWA (class-only urgency) + MP.",
    ),
    "A4": Variant(
        id="A4",
        use_express_layer=True,
        gate_rule="uwa",
        urgency_mode="route",   # u_v = u^route only
        signal_controller="mp",
        description="LTFS + UWA (route-only urgency) + MP.",
    ),
    "A5": Variant(
        id="A5",
        use_express_layer=True,
        gate_rule="uwa",
        urgency_mode="class_route",  # u_v = Eq 5
        signal_controller="mp",
        description="LTFS + UWA (class+route urgency) + MP. Full urgency combination.",
    ),
    "A6": Variant(
        id="A6",
        use_express_layer=True,
        gate_rule="uwa",
        urgency_mode="class_route",
        signal_controller="pwmp",
        pwmp_i_gate=True,
        pwmp_i_dis=True,
        pwmp_i_exp=True,
        pwmp_u_up=False,
        pwmp_occupancy_guard_gate=False,
        description="LTFS + UWA + PWMP indicators (no u_up).",
    ),
    "A7": Variant(
        id="A7",
        use_express_layer=True,
        gate_rule="uwa",
        urgency_mode="class_route",
        signal_controller="pwmp",
        pwmp_i_gate=True,
        pwmp_i_dis=True,
        pwmp_i_exp=True,
        pwmp_u_up=True,
        pwmp_occupancy_guard_gate=False,
        description="LTFS + UWA + PWMP with upstream urgency aggregation.",
    ),
    "A8": Variant(
        id="A8",
        use_express_layer=True,
        gate_rule="uwa",
        urgency_mode="class_route",
        signal_controller="pwmp",
        pwmp_i_gate=True,
        pwmp_i_dis=True,
        pwmp_i_exp=True,
        pwmp_u_up=True,
        pwmp_occupancy_guard_gate=True,   # THE distinguishing feature of A8
        description="Full LTFS with occupancy-guarded gate-feed amplification.",
    ),
}


def get_variant(variant_id: str) -> Variant:
    """Look up a variant by ID (case-insensitive)."""
    key = variant_id.strip().upper()
    if key not in ABLATION_SUITE:
        raise ValueError(f"Unknown variant {variant_id!r}. Valid: {sorted(ABLATION_SUITE)}")
    return ABLATION_SUITE[key]


def _tests():
    for vid in ("A0", "A1", "A2", "A3", "A4", "A5", "A6", "A7", "A8"):
        v = get_variant(vid)
        assert v.id == vid
    # A0 is surface-only
    assert get_variant("A0").use_express_layer is False
    # A8 is the only one with occupancy guard
    guards = {vid for vid, v in ABLATION_SUITE.items() if v.pwmp_occupancy_guard_gate}
    assert guards == {"A8"}, guards
    # Only A6-A8 use PWMP
    pwmp_variants = {vid for vid, v in ABLATION_SUITE.items() if v.uses_pwmp()}
    assert pwmp_variants == {"A6", "A7", "A8"}, pwmp_variants
    print("ablation tests passed.")


if __name__ == "__main__":
    _tests()
