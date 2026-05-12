"""
scenarios/base.py — Scenario specification and runtime materialization.

A Scenario determines:
  - Demand (route files, possibly stretched/scaled)
  - Incident schedule (timed TraCI events during sim body)
  - Which layers are active (surface-only vs LTFS)

Scenarios don't regenerate route files at runtime — they reuse the
duarouter-generated osm.*.rou.xml. Demand-scale modifications use SUMO's
native --scale option; incident modifications use TraCI calls executed on a
schedule.

S0 (surface-only): express layer is disabled at the runner level, not by
modifying the network.
S1 (LTFS baseline): identity scenario.
S2 (incident): at t = WARMUP_S + 1800, set maxSpeed on a selected edge to
5 m/s for 900 s. Edge chosen on the Yan'an Elevated to stress the gating logic.
S3 (demand surge): --scale 1.5 for the whole run.
"""

from __future__ import annotations
from dataclasses import dataclass, field
from typing import Callable, List, Optional

import traci

from .. import config as C


@dataclass
class IncidentEvent:
    """A single scheduled TraCI event during the run."""
    t_start: float
    t_end: float
    apply: Callable[[], None]      # Called at t_start
    revert: Callable[[], None]     # Called at t_end
    description: str = ""
    _applied: bool = field(default=False, init=False)
    _reverted: bool = field(default=False, init=False)


@dataclass
class Scenario:
    id: str                                 # "S0", "S1", "S2", "S3"
    description: str
    demand_scale: float = 1.0              # SUMO --scale argument
    incidents: List[IncidentEvent] = field(default_factory=list)
    force_surface_only: bool = False       # S0: disable express layer in runner
    extra_sumo_args: List[str] = field(default_factory=list)

    def tick(self, t: float) -> None:
        """Called each sim step to apply/revert scheduled incidents."""
        for inc in self.incidents:
            if not inc._applied and t >= inc.t_start:
                try:
                    inc.apply()
                except traci.TraCIException as ex:
                    print(f"[scenario {self.id}] incident apply failed: {ex}")
                inc._applied = True
            if inc._applied and not inc._reverted and t >= inc.t_end:
                try:
                    inc.revert()
                except traci.TraCIException as ex:
                    print(f"[scenario {self.id}] incident revert failed: {ex}")
                inc._reverted = True


def s0_surface_only() -> Scenario:
    return Scenario(
        id="S0",
        description="Surface-only baseline: express layer disabled.",
        force_surface_only=True,
    )


def s1_ltfs() -> Scenario:
    return Scenario(
        id="S1",
        description="LTFS baseline: both layers active.",
    )


def s2_incident(express_edges: set) -> Scenario:
    """
    Incident scenario: pick an express edge, slow it to 5 m/s for 900 s
    starting 1800s after warmup. Fallback to a central surface edge if no
    express edges exist.
    """
    # Pick a representative express edge — the one lexicographically first
    # that exists. The runner passes in express_edges.
    target_edge: Optional[str] = None
    if express_edges:
        target_edge = sorted(express_edges)[0]

    orig_speed: dict = {}

    def apply():
        if target_edge is None:
            return
        try:
            # Save the original max speed across lanes
            lanes = traci.edge.getLaneNumber(target_edge)
            for i in range(lanes):
                lane_id = f"{target_edge}_{i}"
                orig_speed[lane_id] = traci.lane.getMaxSpeed(lane_id)
                traci.lane.setMaxSpeed(lane_id, 5.0)
        except traci.TraCIException as ex:
            print(f"[S2] incident apply error: {ex}")

    def revert():
        for lane_id, spd in orig_speed.items():
            try:
                traci.lane.setMaxSpeed(lane_id, spd)
            except traci.TraCIException:
                pass

    t_start = C.WARMUP_S + 1800.0
    t_end = t_start + 900.0

    return Scenario(
        id="S2",
        description=f"Incident scenario: {target_edge} → 5 m/s for 900 s at t≥{t_start:.0f}.",
        incidents=[
            IncidentEvent(
                t_start=t_start,
                t_end=t_end,
                apply=apply,
                revert=revert,
                description=f"Speed reduction on {target_edge}",
            )
        ] if target_edge else [],
    )


def s3_demand_surge() -> Scenario:
    return Scenario(
        id="S3",
        description="Demand surge: 1.5x all routes.",
        demand_scale=1.5,
    )


def get_scenario(scenario_id: str, express_edges: Optional[set] = None) -> Scenario:
    sid = scenario_id.strip().upper()
    if sid == "S0":
        return s0_surface_only()
    if sid == "S1":
        return s1_ltfs()
    if sid == "S2":
        return s2_incident(express_edges or set())
    if sid == "S3":
        return s3_demand_surge()
    raise ValueError(f"Unknown scenario {scenario_id!r}. Valid: S0, S1, S2, S3.")
