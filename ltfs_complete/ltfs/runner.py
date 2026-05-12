"""
runner.py — CLI entry point for the LTFS simulation framework.

Usage:
    python -m ltfs.runner --variant A0 --scenario S1 --seed 1
    python -m ltfs.runner --variant A8 --scenario S1 --seed 2 --sim-time 5400
    python -m ltfs.runner --freeflow --variant A2
    python -m ltfs.runner --metrics results/A0_S1_seed1_trips.csv

The runner handles:
  1. SUMO launch with the correct cfg / route files / seed / scale
  2. Wiring up the correct controllers/routing/gating per the variant
  3. Per-step tick: scenario events, routing, gating, signals
  4. Per-vehicle trip logging (for metrics)
  5. Cleanup and optional immediate metrics
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Set

# SUMO_HOME handling — essential for traci imports on most installs
if "SUMO_HOME" in os.environ:
    tools = os.path.join(os.environ["SUMO_HOME"], "tools")
    if tools not in sys.path:
        sys.path.append(tools)

import traci
from sumolib import checkBinary

from . import config as C
from .controllers import (
    build_tls_config,
    FixedTimeController,
    ActuatedController,
    MaxPressureController,
    PWMPController,
)
from .routing import StaticRouter, DynamicTTRouter, QRouter
from .gating import GateTTPredictor, RejectionHandler, TSTGate, UWAGate
from .scenarios import get_scenario
from .variants.ablation import get_variant, Variant
from .net_utils import (
    build_express_edges,
    build_gate_edges,
    build_exit_gate_edges,
    express_capacity_from_net,
    tls_ids_within_radius,
    print_network_summary,
)
from .ltfs_blocks import update_occupancy
from .metrics import triplog_to_freeflow, freeflow_name_for_variant


# ============================================================================
# Argument parsing
# ============================================================================

def build_arg_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="LTFS simulation runner (Paper 1 baseline).")
    ap.add_argument("--variant", default="A0", help="Variant ID (A0..A8)")
    ap.add_argument("--scenario", default="S1", help="Scenario ID (S0, S1, S2, S3)")
    ap.add_argument("--seed", type=int, default=1, help="Random seed")
    ap.add_argument("--sim-time", type=float, default=C.MAX_SIM_TIME_S,
                    help=f"Total simulation time (default {C.MAX_SIM_TIME_S:.0f}s = {C.WARMUP_S:.0f}s warmup + {C.SIM_BODY_S:.0f}s body)")
    ap.add_argument("--warmup", type=float, default=C.WARMUP_S, help="Warmup time (trips logged only if depart > warmup)")
    ap.add_argument("--use-gui", action="store_true", help="Launch sumo-gui instead of sumo")
    ap.add_argument("--sumo-cfg", default=str(C.SUMO_CFG), help="SUMO config file path")
    ap.add_argument("--net-file", default=str(C.NET_FILE), help="Network file path")
    ap.add_argument("--results-dir", default=str(C.RESULTS_DIR), help="Output directory for trip logs")
    ap.add_argument("--routing", default="static", choices=["static", "dynamic_tt", "q_routing"],
                    help="Intra-layer routing strategy (Sec VI)")
    ap.add_argument("--signal-override", default=None, choices=["fixed", "actuated", "mp", "pwmp"],
                    help="Override the variant's signal controller (for sensitivity)")
    ap.add_argument("--freeflow", action="store_true",
                    help="Run in free-flow calibration mode (stretch departures, produce freeflow_*.csv)")
    ap.add_argument("--quiet", action="store_true", help="Suppress per-step logging")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="Multiply demand by this factor (<1.0 reduces congestion)")
    ap.add_argument("--w-max", type=float, default=None,
                    help="Override PWMP_W_MAX (sensitivity sweep). Default = config value.")
    ap.add_argument("--theta-s", type=float, default=None,
                    help="Override THETA_S (gating threshold). Default = config value.")
    ap.add_argument("--theta-u-s", type=float, default=None,
                    help="Override THETA_U_S (urgency threshold). Default = config value.")
    ap.add_argument("--lambda-gating", type=float, default=None,
                    help="Override LAMBDA_GATING. Default = config value.")
    ap.add_argument("--lambda-signal", type=float, default=None,
                    help="Override LAMBDA_SIGNAL. Default = config value.")
    return ap


# ============================================================================
# Free-flow: temp cfg with stretched departures
# ============================================================================

def _stretch_route_file(src_path: str, stretch: float) -> str:
    tree = ET.parse(src_path)
    root = tree.getroot()
    for v in list(root.findall("vehicle")) + list(root.findall("person")):
        dep = v.get("depart")
        if dep is None:
            continue
        try:
            v.set("depart", str(float(dep) * stretch))
        except ValueError:
            pass
    tmp = tempfile.NamedTemporaryFile(delete=False, suffix=".rou.xml")
    tree.write(tmp.name, encoding="utf-8", xml_declaration=True)
    return tmp.name


def _make_freeflow_cfg(sumo_cfg: str, stretch: float) -> str:
    """
    Build a temp sumocfg with all route files stretched by `stretch` and any
    GUI-only references removed (so we can run headless even if the original
    referenced view settings).
    """
    tree = ET.parse(sumo_cfg)
    root = tree.getroot()

    gui_node = root.find("gui_only")
    if gui_node is not None:
        root.remove(gui_node)

    input_node = root.find("input")
    if input_node is None:
        raise ValueError(f"No <input> in {sumo_cfg}")

    rf_node = input_node.find("route-files")
    if rf_node is None or "value" not in rf_node.attrib:
        raise ValueError(f"No <route-files> in {sumo_cfg}")

    base_dir = Path(sumo_cfg).resolve().parent
    route_files = [p.strip() for p in rf_node.attrib["value"].split(",") if p.strip()]
    stretched: List[str] = []
    for rf in route_files:
        p = Path(rf)
        if not p.is_absolute():
            p = base_dir / p
        stretched.append(_stretch_route_file(str(p), stretch))
    rf_node.attrib["value"] = ",".join(stretched)

    # Make all other file references absolute
    for tag in ["net-file", "additional-files"]:
        node = input_node.find(tag)
        if node is not None and "value" in node.attrib:
            parts = [p.strip() for p in node.attrib["value"].split(",") if p.strip()]
            fixed: List[str] = []
            for p in parts:
                pp = Path(p)
                if not pp.is_absolute():
                    pp = (base_dir / pp).resolve()
                fixed.append(str(pp))
            node.attrib["value"] = ",".join(fixed)

    tmp = tempfile.NamedTemporaryFile(delete=False, suffix=".sumocfg")
    tree.write(tmp.name, encoding="utf-8", xml_declaration=True)
    return tmp.name


# ============================================================================
# Main run
# ============================================================================

@dataclass
class RunContext:
    variant: Variant
    scenario_id: str
    seed: int
    sim_time: float
    warmup: float
    use_gui: bool
    net_file: str
    sumo_cfg: str
    results_dir: Path
    routing_strategy: str
    signal_override: Optional[str]
    freeflow: bool
    quiet: bool
    scale: float = 1.0
    w_max_override: Optional[float] = None
    theta_s_override: Optional[float] = None
    theta_u_s_override: Optional[float] = None
    lambda_gating_override: Optional[float] = None
    lambda_signal_override: Optional[float] = None

def run_sim(ctx: RunContext) -> Path:
    # Apply w_max override if set on context (sensitivity sweep)
    if getattr(ctx, "w_max_override", None) is not None:
        C.PWMP_W_MAX = float(ctx.w_max_override)
        print(f"[runner] w_max override active: PWMP_W_MAX = {C.PWMP_W_MAX}")
    ctx.results_dir.mkdir(parents=True, exist_ok=True)
    if getattr(ctx, "theta_s_override", None) is not None:
        C.THETA_S = float(ctx.theta_s_override)
        print(f"[runner] THETA_S override active: {C.THETA_S}")
    if getattr(ctx, "theta_u_s_override", None) is not None:
        C.THETA_U_S = float(ctx.theta_u_s_override)
        print(f"[runner] THETA_U_S override active: {C.THETA_U_S}")
    if getattr(ctx, "lambda_gating_override", None) is not None:
        C.LAMBDA_GATING = float(ctx.lambda_gating_override)
        print(f"[runner] LAMBDA_GATING override active: {C.LAMBDA_GATING}")
    if getattr(ctx, "lambda_signal_override", None) is not None:
        C.LAMBDA_SIGNAL = float(ctx.lambda_signal_override)
        print(f"[runner] LAMBDA_SIGNAL override active: {C.LAMBDA_SIGNAL}")
    # -------------------------------------------------------------------------
    # Network-level setup (before traci.start to allow scenario to consult)
    # -------------------------------------------------------------------------
    if not ctx.variant.use_express_layer:
        express_edges: Set[str] = set()
        gate_entry: Set[str] = set()
        gate_exit: Set[str] = set()
        express_capacity = 1.0  # unused
    else:
        express_edges = build_express_edges(
            ctx.net_file,
            C.EXPRESS_MIN_SPEED_MPS,
            C.EXPRESS_MIN_Z,
            C.EXPRESS_MAX_Z,
            C.EXPRESS_KEEP_COMPONENTS,
        )
        gate_entry = build_gate_edges(ctx.net_file, express_edges)
        gate_exit = build_exit_gate_edges(ctx.net_file, express_edges)
        express_capacity = express_capacity_from_net(
            ctx.net_file, express_edges, C.VEHICLE_SPACING_M
        )

    scenario = get_scenario(ctx.scenario_id, express_edges if not ctx.freeflow else set())

    # S0 (surface-only) overrides variant settings: behaves as A0.
    if scenario.force_surface_only and ctx.variant.use_express_layer:
        print("[runner] S0 scenario forces surface-only; disabling express layer for this run.")
        express_edges = set()
        gate_entry = set()
        gate_exit = set()

    # -------------------------------------------------------------------------
    # SUMO launch
    # -------------------------------------------------------------------------
    if ctx.freeflow:
        cfg = _make_freeflow_cfg(ctx.sumo_cfg, C.FREEFLOW_DEPART_STRETCH)
        sim_end = ctx.sim_time + C.FREEFLOW_SIM_END_BUFFER_S
    else:
        cfg = ctx.sumo_cfg
        sim_end = ctx.sim_time

    sumo_args = [
        checkBinary("sumo-gui" if ctx.use_gui else "sumo"),
        "-c", cfg,
        "--step-length", str(C.STEP_LENGTH_S),
        "--no-step-log", "true",
        "--seed", str(ctx.seed),
        "--ignore-route-errors", "true",
        "--end", str(sim_end),
    ]
    effective_scale = scenario.demand_scale * ctx.scale
    if effective_scale != 1.0:
        sumo_args += ["--scale", str(effective_scale)]
    sumo_args += list(scenario.extra_sumo_args)
    traci.start(sumo_args)

    # -------------------------------------------------------------------------
    # TLS footprint
    # -------------------------------------------------------------------------
    try:
        tls_pool = tls_ids_within_radius(ctx.net_file, C.YANAN_ANCHOR_TLS, C.YANAN_RADIUS_M)
    except Exception as e:
        print(f"[runner] TLS footprint fallback: {e}")
        tls_pool = list(traci.trafficlight.getIDList())

    all_tls = set(traci.trafficlight.getIDList())
    tls_ids = [t for t in tls_pool if t in all_tls]
    if not tls_ids:
        tls_ids = list(all_tls)
    tls_config = build_tls_config(tls_ids)
    tls_ids = list(tls_config.keys())

    print_network_summary(ctx.net_file, express_edges, gate_entry, gate_exit,
                          express_capacity, tls_ids)

    # -------------------------------------------------------------------------
    # Wire up controllers
    # -------------------------------------------------------------------------
    sig_choice = ctx.signal_override or ctx.variant.signal_controller
    if sig_choice == "fixed":
        signal_ctrl = FixedTimeController()
    elif sig_choice == "actuated":
        signal_ctrl = ActuatedController(C.ACTUATED_GAP_S, C.MIN_GREEN_S, C.MAX_GREEN_S)
    elif sig_choice == "pwmp":
        signal_ctrl = PWMPController(ctx.variant, express_edges, gate_entry, gate_exit)
    else:
        signal_ctrl = MaxPressureController()

    if ctx.routing_strategy == "dynamic_tt":
        router = DynamicTTRouter()
    elif ctx.routing_strategy == "q_routing":
        router = QRouter()
    else:
        router = StaticRouter()
    router.initialize()

    # Gating — only active if variant uses it
    tst_gate = TSTGate() if ctx.variant.gate_rule == "tst" else None
    uwa_gate = UWAGate(ctx.variant) if ctx.variant.gate_rule == "uwa" else None
    predictor: Optional[GateTTPredictor] = None
    rejection: Optional[RejectionHandler] = None
    if ctx.variant.uses_gating() and express_edges:
        predictor = GateTTPredictor(ctx.net_file, express_edges, gate_entry, gate_exit, router)
        rejection = RejectionHandler(ctx.net_file, express_edges)

    # -------------------------------------------------------------------------
    # Trip logging setup
    # -------------------------------------------------------------------------
    tag = "freeflow" if ctx.freeflow else f"{ctx.variant.id}_{ctx.scenario_id}_seed{ctx.seed}"
    if ctx.freeflow:
        tag = f"freeflow_{sig_choice.upper()}_{(ctx.variant.gate_rule or 'none').upper()}"
    out_csv = ctx.results_dir / f"{tag}_trips.csv"

    veh_depart: Dict[str, float] = {}
    veh_edges: Dict[str, List[str]] = {}
    veh_class: Dict[str, str] = {}
    veh_admitted: Dict[str, bool] = {}
    veh_gate_evaluated: Dict[str, bool] = {}
    veh_on_express_s: Dict[str, float] = {}
    prev_on_express: Set[str] = set()
    o_X: float = 0.0
    gate_admit_count = 0
    gate_deny_count = 0

    # Rolling state
    current_phase: Dict[str, int] = {tls_id: traci.trafficlight.getPhase(tls_id) for tls_id in tls_ids}
    time_in_phase: Dict[str, float] = {tls_id: 0.0 for tls_id in tls_ids}
    next_gate_eval_t: float = 0.0

    # Route key maker — stable endpoint-based keying (route_key.make_route_key)
    from .route_key import make_route_key

    try:
        with open(out_csv, "w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow([
                "vehID", "routeKey", "class", "depart", "arrival",
                "travel_time", "distance_m", "on_express_s", "admitted", "denied",
            ])

            loop_end = sim_end if ctx.freeflow else ctx.sim_time
            while traci.simulation.getTime() < loop_end:
                traci.simulationStep()
                t = traci.simulation.getTime()

                # Scenario events
                scenario.tick(t)

                # Router step
                router.step(t)

                # Vehicle departures
                for vid in traci.simulation.getDepartedIDList():
                    veh_depart[vid] = t
                    try:
                        veh_edges[vid] = list(traci.vehicle.getRoute(vid))
                    except traci.TraCIException:
                        veh_edges[vid] = []
                    try:
                        veh_class[vid] = traci.vehicle.getTypeID(vid)
                    except traci.TraCIException:
                        veh_class[vid] = ""
                    veh_admitted[vid] = False
                    veh_gate_evaluated[vid] = False
                    veh_on_express_s[vid] = 0.0

                # Vehicle arrivals — write trip log (only if depart > warmup or freeflow)
                for vid in traci.simulation.getArrivedIDList():
                    dep = veh_depart.pop(vid, None)
                    edges = veh_edges.pop(vid, [])
                    cls = veh_class.pop(vid, "")
                    admitted = veh_admitted.pop(vid, False)
                    gate_eval = veh_gate_evaluated.pop(vid, False)
                    on_express_s = veh_on_express_s.pop(vid, 0.0)

                    if dep is None:
                        continue
                    if not ctx.freeflow and dep < ctx.warmup:
                        continue  # Exclude warmup trips

                    tt = t - dep
                    # Sum edge lengths as distance proxy (SUMO doesn't easily give us full trip distance post-hoc here)
                    dist = 0.0
                    for e in edges:
                        if e.startswith(":"):
                            continue
                        try:
                            dist += traci.lane.getLength(f"{e}_0")
                        except traci.TraCIException:
                            pass
                    rkey = make_route_key(edges)
                    denied = (rejection is not None) and rejection.is_denied(vid)
                    writer.writerow([
                        vid, rkey, cls, f"{dep:.2f}", f"{t:.2f}",
                        f"{tt:.2f}", f"{dist:.2f}", f"{on_express_s:.2f}",
                        str(bool(admitted)), str(bool(denied)),
                    ])
                    if rejection:
                        rejection.cleanup([vid])

                # Track vehicles on express edges — for occupancy and on-express-time
                current_on_express: Set[str] = set()
                if express_edges:
                    for e in express_edges:
                        try:
                            for v in traci.edge.getLastStepVehicleIDs(e):
                                current_on_express.add(v)
                        except traci.TraCIException:
                            continue
                    # Increment on-express-time
                    for vid in current_on_express:
                        if vid in veh_on_express_s:
                            veh_on_express_s[vid] += C.STEP_LENGTH_S

                    # Update occupancy (N4)
                    inflow = len(current_on_express - prev_on_express)
                    outflow = len(prev_on_express - current_on_express)
                    o_X = update_occupancy(o_X, inflow, outflow, express_capacity)
                    prev_on_express = current_on_express

                # Gating decisions
                if (ctx.variant.uses_gating() and predictor is not None and
                        t >= next_gate_eval_t and gate_entry):
                    next_gate_eval_t = t + C.GATE_EVAL_PERIOD_S

                    # In "open" mode (A1), admit every eligible vehicle without threshold check
                    if ctx.variant.gate_rule == "open":
                        for gate_edge in gate_entry:
                            try:
                                for vid in traci.edge.getLastStepVehicleIDs(gate_edge):
                                    if not veh_gate_evaluated.get(vid, False):
                                        veh_gate_evaluated[vid] = True
                                        veh_admitted[vid] = True
                                        gate_admit_count += 1
                            except traci.TraCIException:
                                continue
                    else:
                        for gate_edge in gate_entry:
                            try:
                                gate_vids = list(traci.edge.getLastStepVehicleIDs(gate_edge))
                            except traci.TraCIException:
                                continue
                            for vid in gate_vids:
                                if veh_gate_evaluated.get(vid, False):
                                    continue
                                if rejection and rejection.is_denied(vid):
                                    veh_gate_evaluated[vid] = True
                                    continue

                                try:
                                    route = traci.vehicle.getRoute(vid)
                                except traci.TraCIException:
                                    continue
                                if not route:
                                    continue
                                dest_edge = route[-1]

                                delta_t = predictor.delta_t(gate_edge, dest_edge)

                                if ctx.variant.gate_rule == "tst" and tst_gate:
                                    admit = tst_gate.decide(delta_t, o_X)
                                elif ctx.variant.gate_rule == "uwa" and uwa_gate:
                                    admit = uwa_gate.decide(vid, delta_t, o_X)
                                else:
                                    admit = False

                                veh_gate_evaluated[vid] = True
                                veh_admitted[vid] = admit
                                if admit:
                                    gate_admit_count += 1
                                else:
                                    gate_deny_count += 1
                                    if rejection:
                                        rejection.mark_and_reroute(vid, router)

                # Push current o_X into PWMP controller (A6-A8 use it in Eq 10)
                if isinstance(signal_ctrl, PWMPController):
                    signal_ctrl.set_occupancy(o_X)

                # Signal decisions
                for tls_id in tls_ids:
                    time_in_phase[tls_id] += C.STEP_LENGTH_S
                    if time_in_phase[tls_id] < C.MIN_GREEN_S:
                        continue
                    decision = signal_ctrl.decide(tls_id, t, tls_config)
                    if decision is None:
                        continue
                    if decision != current_phase[tls_id]:
                        try:
                            traci.trafficlight.setPhase(tls_id, decision)
                            current_phase[tls_id] = decision
                            time_in_phase[tls_id] = 0.0
                        except traci.TraCIException:
                            pass

                # Periodic log
                if not ctx.quiet and int(t) % 300 == 0 and abs(t - int(t)) < C.STEP_LENGTH_S:
                    print(f"[t={t:6.0f}] occ={o_X:.2f} admit={gate_admit_count} deny={gate_deny_count} "
                          f"on_express={len(current_on_express)}")

    finally:
        try:
            traci.close()
        except Exception:
            pass

    # Post-processing for free-flow: collapse to per-(routeKey, class) minima
    if ctx.freeflow:
        ff_out = ctx.results_dir / freeflow_name_for_variant(ctx.variant)
        n_pairs = triplog_to_freeflow(str(out_csv), str(ff_out))
        print(f"[freeflow] wrote {ff_out} with {n_pairs} (routeKey, class) pairs.")

    print(f"[runner] Trip log: {out_csv}")
    print(f"[runner] Gate stats — admitted: {gate_admit_count}, denied: {gate_deny_count}")
    return out_csv


# ============================================================================
# Entry point
# ============================================================================

def main(argv: Optional[List[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)

    variant = get_variant(args.variant)
    ctx = RunContext(
        variant=variant,
        scenario_id=args.scenario,
        seed=args.seed,
        sim_time=args.sim_time,
        warmup=args.warmup,
        use_gui=args.use_gui,
        net_file=str(Path(args.net_file).resolve()),
        sumo_cfg=str(Path(args.sumo_cfg).resolve()),
        results_dir=Path(args.results_dir).resolve(),
        routing_strategy=args.routing,
        signal_override=args.signal_override,
        freeflow=args.freeflow,
        quiet=args.quiet,
        scale=args.scale,
        w_max_override=args.w_max,
        theta_s_override=args.theta_s,
        theta_u_s_override=args.theta_u_s,
        lambda_gating_override=args.lambda_gating,
        lambda_signal_override=args.lambda_signal,
    )

    print(f"[runner] variant={variant.id}  scenario={args.scenario}  seed={args.seed}  "
          f"sim_time={args.sim_time:.0f}  warmup={args.warmup:.0f}  routing={args.routing}")
    print(f"[runner] variant: {variant.description}")

    out_csv = run_sim(ctx)
    print(f"[runner] done: {out_csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
