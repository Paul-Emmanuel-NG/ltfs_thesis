"""
mp_controller.py

Max Pressure controller for one or more TLS using standard_blocks.py

Fixes vs your current file:
- Removes placeholder TLS_CONFIG (north_in, etc.)
- Auto-builds TLS_CONFIG from the TLS(s) in the loaded SUMO network
  using controlled links + the active traffic light program phases.
- Works for one or multiple TLS IDs.
- Includes basic validation and safe fallbacks.

How to use (PowerShell examples):
  # Control one TLS
  $env:MP_TLS_IDS="cluster_479314640_850287516"
  python mp_controller.py

  # Control multiple TLS (comma-separated)
  $env:MP_TLS_IDS="tlsA,tlsB,tlsC"
  python mp_controller.py

Notes:
- Turning probabilities are unknown from TLS alone; this script uses equal split
  across downstream options per incoming edge within each phase.
"""

import os
from pathlib import Path
import traci
from sumolib import checkBinary

from standard_blocks import choose_phase
from ltfs_blocks import urgency_from_type_id

# ----------------
# Configuration
# ----------------
CORE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = CORE_DIR.parents[1]
SUMO_CFG = str(PROJECT_ROOT / "sim" / "network" / "osm.sumocfg")

USE_GUI = True
STEP_LENGTH = 1.0
MIN_GREEN = 10.0
MAX_SIM_TIME = 7200.0

# If MP_TLS_IDS env var is not set, we will control ALL TLS in the network.
TLS_IDS_ENV = os.getenv("MP_TLS_IDS", "").strip()


# ----------------
# Helpers
# ----------------
_EDGE_U_CACHE: dict[str, tuple[float,float]] = {}

def edge_mean_u(edge_id: str, now_t: float) -> float:
    last=_EDGE_U_CACHE.get(edge_id,(-1e9,0.4))
    if (now_t-last[0])<PWMP_EDGE_U_PERIOD:
        return float(last[1])
    try:
        vids=traci.edge.getLastStepVehicleIDs(edge_id)
    except Exception:
        vids=[]
    if not vids:
        mu=0.4
    else:
        us=[]
        for v in vids:
            try:
                us.append(urgency_from_type_id(traci.vehicle.getTypeID(v)))
            except Exception:
                us.append(0.4)
        mu=float(sum(us)/max(1,len(us)))
    _EDGE_U_CACHE[edge_id]=(now_t,mu)
    return mu

def edge_queue(edge_id: str) -> float:
    """Queue proxy: number of vehicles on the edge in the last step."""
    return float(traci.edge.getLastStepVehicleNumber(edge_id))


def _is_green(ch: str) -> bool:
    # Treat both 'G' and 'g' as permissive greens.
    return ch in ("G", "g")


def build_tls_config_from_network(tls_ids: list[str]) -> dict:
    """
    Build TLS_CONFIG automatically:
    TLS_CONFIG[tls_id]["phases"][phase_index] = list of movements

    Each movement is:
      {
        "up_edge": <from_edge>,
        "down_edges": [<to_edge_1>, <to_edge_2>, ...],
        "turn_probs": [p1, p2, ...]  (equal split)
      }

    We group by from_edge within each phase because one incoming edge may feed
    multiple outgoing edges (different lanes/turns) that are green in that phase.
    """
    tls_config: dict = {}

    for tls_id in tls_ids:
        # Controlled links are grouped by signal index.
        # Each entry is a list of (fromLane, toLane, viaLane).
        controlled_links = traci.trafficlight.getControlledLinks(tls_id)
        if not controlled_links:
            # Some TLS may be internal/empty in certain nets
            continue

        # Get the current active program's phase definitions (state strings)
        active_prog_id = traci.trafficlight.getProgram(tls_id)
        programs = traci.trafficlight.getCompleteRedYellowGreenDefinition(tls_id)
        if not programs:
            continue
        program = None
        for p in programs:
            if getattr(p, 'programID', None) == active_prog_id:
                program = p
                break
        if program is None:
            program = programs[0]
        if not getattr(program, 'phases', None):
            continue

        # Use the active program phases for MP logic.
        # (This avoids accidentally using programs[0] when a different program is active.)
        phases = program.phases
        #
        # (If you need a specific programID, we can extend later.)

        phases_dict: dict[int, list[dict]] = {}

        for p_idx, ph in enumerate(phases):
            state = ph.state  # string; one char per signal index
            # Map: from_edge -> set(to_edges)
            from_to: dict[str, set[str]] = {}

            # signal_group_index matches position in 'state'
            for sg_idx, sg_links in enumerate(controlled_links):
                if sg_idx >= len(state):
                    break
                if not _is_green(state[sg_idx]):
                    continue

                # For each link permitted by this signal group:
                for from_lane, to_lane, _via_lane in sg_links:
                    if not from_lane or not to_lane:
                        continue
                    try:
                        from_edge = traci.lane.getEdgeID(from_lane)
                        to_edge = traci.lane.getEdgeID(to_lane)
                    except traci.TraCIException:
                        continue

                    # Ignore internal edges
                    if not from_edge or from_edge.startswith(":"):
                        continue
                    if not to_edge or to_edge.startswith(":"):
                        continue

                    from_to.setdefault(from_edge, set()).add(to_edge)

            # Convert grouped movements into list expected by choose_phase()
            movements: list[dict] = []
            for from_edge, to_edges_set in from_to.items():
                to_edges = sorted(list(to_edges_set))
                if not to_edges:
                    continue
                k = len(to_edges)
                movements.append(
                    {
                        "up_edge": from_edge,
                        "down_edges": to_edges,
                        "turn_probs": [1.0 / k] * k,  # equal split
                    }
                )

            phases_dict[p_idx] = movements

        tls_config[tls_id] = {"phases": phases_dict}

    return tls_config


def choose_phase_for_tls(tls_id: str, tls_config: dict, now_t: float) -> int:
    cfg = tls_config[tls_id]
    phase_to_movements = {}

    for phase_idx, movement_defs in cfg["phases"].items():
        movements = []
        for m in movement_defs:
            q_up = edge_queue(m["up_edge"])
            q_down_list = [edge_queue(e) for e in m["down_edges"]]
            rho_list = m["turn_probs"]
            movements.append({"q_up": q_up, "q_down_list": q_down_list, "rho_list": rho_list})
        phase_to_movements[phase_idx] = movements

    return choose_phase(phase_to_movements)


def _assert_tls_config_is_valid(tls_config: dict):
    sim_tls_ids = set(traci.trafficlight.getIDList())
    sim_edge_ids = set(traci.edge.getIDList())

    if not tls_config:
        raise RuntimeError("Auto-built TLS_CONFIG is empty. Check that your network has TLS and routes loaded.")

    for tls_id, cfg in tls_config.items():
        if tls_id not in sim_tls_ids:
            raise ValueError(f"TLS_ID '{tls_id}' not found in simulation. Check your cfg/network.")
        for phase_idx, movs in cfg["phases"].items():
            for m in movs:
                if m["up_edge"] not in sim_edge_ids:
                    raise ValueError(f"up_edge '{m['up_edge']}' not found (TLS {tls_id}, phase {phase_idx})")
                for de in m["down_edges"]:
                    if de not in sim_edge_ids:
                        raise ValueError(f"down_edge '{de}' not found (TLS {tls_id}, phase {phase_idx})")


# ----------------
# Main
# ----------------
def run():
    sumo_binary = checkBinary("sumo-gui" if USE_GUI else "sumo")

    traci.start(
        [
            sumo_binary,
            "-c",
            SUMO_CFG,
            "--step-length",
            str(STEP_LENGTH),
            "--no-step-log",
            "true",
        ]
    )

    try:
        # Decide which TLS to control
        all_tls = traci.trafficlight.getIDList()
        if TLS_IDS_ENV:
            requested = [x.strip() for x in TLS_IDS_ENV.split(",") if x.strip()]
            tls_ids = [t for t in requested if t in all_tls]
            missing = [t for t in requested if t not in all_tls]
            if missing:
                raise ValueError(f"These TLS IDs are not in the loaded network: {missing}")
        else:
            tls_ids = list(all_tls)

        # Auto-build config for the selected TLS IDs
        tls_config = build_tls_config_from_network(tls_ids)
        _assert_tls_config_is_valid(tls_config)

        # State for min-green enforcement
        controlled_tls_ids = list(tls_config.keys())
        current_phase = {tls_id: traci.trafficlight.getPhase(tls_id) for tls_id in controlled_tls_ids}
        time_in_phase = {tls_id: 0.0 for tls_id in controlled_tls_ids}

        print(f"[mp_controller] Controlling {len(controlled_tls_ids)} TLS: {controlled_tls_ids}")

        while traci.simulation.getTime() < MAX_SIM_TIME and traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()

            for tls_id in controlled_tls_ids:
                time_in_phase[tls_id] += STEP_LENGTH
                if time_in_phase[tls_id] < MIN_GREEN:
                    continue

                best_phase = choose_phase_for_tls(tls_id, tls_config, t)
                if best_phase != current_phase[tls_id]:
                    traci.trafficlight.setPhase(tls_id, best_phase)
                    current_phase[tls_id] = best_phase
                    time_in_phase[tls_id] = 0.0

    finally:
        traci.close()


if __name__ == "__main__":
    run()
