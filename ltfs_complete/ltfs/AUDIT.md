# LTFS Paper ↔ Code Audit

Mapping of every substantive mismatch between the Paper 1 LaTeX draft and the
prior code in `2025-12-02-16-58-29/`. Categories: **Wrong** (code runs but
contradicts paper), **Missing** (paper specifies, code lacks), **Unnecessary**
(code has it, paper doesn't need it, dead or redundant), **Ambiguous** (paper
underspecifies, code must choose; flag for supervisor).

Paper citations use section number and equation number from the current LaTeX
draft. Code citations use `file.py:line`.

---

## WRONG (code contradicts paper)

### W1. `phase_pressure` applies weight to full pressure, not split-positive form

**Paper:** Eq 11 (body), Eq 18 (Sec V-D), N8 (Appendix A). PWMP requires
`π^PWMP = w·[π]⁺ + [π]⁻` so weights only amplify accumulating upstream demand
and never suppress backpressure against downstream spillback. Paper also
relies on this for the "strict generalization" claim (when `w ≡ 1`, PWMP
reduces exactly to classical MP).

**Code:** `standard_blocks.py:54-62` — `w * p` where `p` is the full signed
movement pressure. This means weights suppress *both* positive and negative
pressure, breaking the split-positive property. Also breaks the A0–A5 vs A6–A8
"strict generalization" comparison the paper builds on.

**Fix:** Rewrite `phase_pressure` to split at the movement level. New file:
`controllers/max_pressure.py` and `controllers/pwmp.py` both use the corrected
form.

### W2. UWA branch is dead code

**Paper:** Eq 15 (Sec III-D), Eq 26 (Sec VII-C). UWA admission rule:
`u_v(t) · ΔT_v(t) ≥ Θ_u` AND `o_X(t) ≤ κ`.

**Code:** `mp_ltfs1.0.1_wt.py:647-648` references `GATE_MODE`, `THETA_U`, and
`vehicle_combined_urgency` — none of these symbols are defined anywhere in the
codebase. The `if GATE_MODE == "UWA":` branch would raise `NameError` if
reached, but in practice the code only runs TST because `GATE_MODE` is never
set. So UWA has never executed a single run.

**Fix:** Implement properly in `gating/uwa.py` with `u_v(t)` from Eq 5
(combined gating urgency, A+C) as defined in `ltfs_blocks.py`.

### W3. Express layer is a speed bonus, not a layer

**Paper:** Sec III-A, III-E. Express layer X is a subset of edges with its own
occupancy state `o_X(t)`; gates are controllable entry/exit points; admission
respects capacity/safety (N3, N4).

**Code:** `mp_ltfs1.0.1_wt.py:660-670`. Admitted vehicles get their
`speedFactor` multiplied by `EXPRESS_SPEED_FAST = 1.3` while on express edges;
denied vehicles run at normal speed. There is no capacity enforcement beyond
the soft `o_X ≤ κ` gate check, and the speed bonus on top of SUMO's own
speed limit is a simulation artifact — it doesn't model physical express-layer
behaviour.

**Fix:** Remove the speed bonus entirely. The network already has higher speed
limits on express edges (measured 27.78 m/s on the Yan'an Elevated vs ~14 m/s
on surface arterials). Admission is enforced purely by routing: admitted
vehicles retain their original route through express edges; denied vehicles
are routed around express edges (rejection handling per W8).

### W4. Rejection handling lets denied vehicles retry downstream

**Paper:** Sec VII-D. "v does not attempt admission at any subsequent gate on
this trip." This explicitly rules out oscillation and cascade pathologies.

**Code:** `mp_ltfs1.0.1_wt.py:629-657`. The `gate_decided[vid]` flag is set per
vehicle but the logic evaluates only vehicles currently on the specific
`gate_edge`. A vehicle denied at gate A then routed around can encounter gate
B on its detour and have `gate_decided[vid]` re-checked against B's gate edge.
The dict key isn't per-gate either, so a second gate arrival would be skipped
— but because denied vehicles are rerouted via `setEffort` away from ALL
express edges (not just gate A), the downstream retry is obscured. In
practice the code's behaviour depends on routing luck.

**Fix:** Maintain an explicit `rejected_vehicles: set[str]` at the gating
controller. First rejection writes the vid to this set; on every subsequent
gating decision, vids in this set are automatically denied without threshold
evaluation. Reroute happens exactly once via a one-shot `traci.simulation.findRoute`
with express edges weighted out.

### W5. Free-flow reference uses per-path hash, not per (route, class)

**Paper:** Eq 28 (Sec IX-A). `TT_ff^π(r, c) = min{TT_low^π(r, c)}` where `r` is
a route (origin-destination, not a specific path) and `c` is a vehicle class.

**Code:** `route_key.py:22-35` returns a hash of the full edge sequence. So a
vehicle going from O to D via path P1 and another O→D vehicle via path P2 get
different `routeKey` values and different free-flow references, even though
paper defines them as the same `(r, c)`.

**Consequence:** `metrics_wt_all.py:107-108` keeps minimum per-hash; in a
congested run many trips have `routeKey` values that never appeared in the
free-flow run (different paths under different congestion), so they are
*dropped* from WT computation. The "skipped" count in the existing logs is
likely large.

**Fix:** New `route_key.py` that keys on `(first_edge, last_edge, vehicle_type)`
with optional finer granularity only if the network has multiple O-D pairs
sharing endpoints. Fall back gracefully when free-flow lookup misses.

### W6. Queue proxy is edge vehicle count, not movement queue

**Paper:** Eq 7, Eq 30–31 (S1–S2). `q_{i,m}(t)` is the movement-level queue;
standard MP uses upstream movement queues minus weighted downstream queues.

**Code:** `mp_controller.py:75`, `mp_single_baseline_wt.py:212-214`,
`mp_ltfs1.0.1_wt.py:194-196` — `traci.edge.getLastStepVehicleNumber(edge_id)`
returns vehicle count on the *whole edge*, not the lanes corresponding to a
specific movement. Multi-lane approaches where one lane serves a turn and
another serves through traffic are measured as one queue.

**Fix:** Compute per-movement queue via `traci.lane.getLastStepVehicleNumber`
summed over the lanes that actually serve that movement, which comes from
`getControlledLinks`. Implemented in `controllers/max_pressure.py`.

### W7. `MAX_SIM_TIME = 7200 s` contradicts paper's experimental protocol

**Paper:** Sec VIII-E. Warmup 900 s, simulation 5400 s. Total 6300 s.

**Code:** `mp_ltfs1.0.1_wt.py:41`, `mp_single_baseline_wt.py:32`,
`mp_controller.py:42` — all set 7200 s with no warmup separation. Logged
trips include the warmup period, inflating mean travel time with unstable
initial conditions.

**Fix:** Config constant `WARMUP_S = 900`, `SIM_BODY_S = 5400`, and trip
logging only accepts vehicles whose depart is after warmup.

### W8. `maybe_reroute_denied_vehicle` uses `setEffort` but MP doesn't consume it

**Paper:** N/A — paper specifies dynamic routing uses smoothed TT (Eq 21).

**Code:** `mp_ltfs1.0.1_wt.py:502-514` sets edge `effort` to 1e6 before
`findRoute`, then resets to 0.0. But `traci.simulation.findRoute` uses
travel-time by default, not effort; `setEffort` is only consumed by
`findRoute` when called with `routingMode=1` (ROUTING_MODE_AGGREGATED_CUSTOM).
As written, the penalty has no effect and the reroute returns the same path.

**Fix:** Either use `setAdaptedTraveltime` with a large value (which IS
consumed by default `findRoute`), or compute the alternative route with edges
on the express layer pruned from the network graph via `sumolib.net`. The
rewrite uses the pruned-graph approach (deterministic, no state pollution).

### W9. Hard-coded Windows absolute paths everywhere

**Paper:** N/A — paper's reproducibility claim (Contribution 4) implies a
portable codebase.

**Code:** `mp_ltfs1.0.1_wt.py:35-36`, `mp_single_baseline_wt.py:26-27`,
`mp_controller.py:37`, `mp_freeflow.py:39-40`, `list_yanan_tls_ids.py:10`,
`mp_freeflow_hardcoded.py:31-32`. All hardcoded to
`C:\Users\akinw\Desktop\thesis\2025-12-02-16-58-29\`.

**Fix:** Paths become CLI args defaulting to the current working directory;
`config.py` holds them as `pathlib.Path` objects relative to project root.

### W10. `vehicle_u` / `edge_mean_u` in baseline reference undefined symbols

**Code:** `mp_single_baseline_wt.py:171-209` defines `vehicle_u` and
`edge_mean_u` using `PWMP_EDGE_U_PERIOD`, `ROUTE_L_REF`, `URGENCY_LAM`,
`get_edge_length` — none defined in this file. `mp_controller.py:53-72` has
the same dead code. Neither function is ever called from the baseline run
loop (lines 380–419 don't reference them). They would `NameError` at first
invocation.

**Fix:** Delete. These belong in `controllers/pwmp.py` only, with the
constants in `config.py`.

### W11. Turning probabilities are always equal-split

**Paper:** Eq 7 — `ρ_{m,e}` are the turning ratios from movement m to
downstream link e. Paper does not specify how these are estimated but the
equal-split assumption heavily influences MP behaviour.

**Code:** `mp_ltfs1.0.1_wt.py:264`, `mp_single_baseline_wt.py:286`,
`mp_controller.py:173` — `[1.0 / k] * k` for all movements.

**Paper ambiguity surfaces here** (see Ambiguity A1). Equal-split is a
defensible assumption in the absence of observed turning counts. Fix is to
document this choice explicitly in `controllers/max_pressure.py` and add a
TODO for future empirical estimation.

---

## MISSING (paper specifies, code lacks)

### M1. A0–A8 ablation variants (Table I, Sec VIII-D)

**Paper:** Full ablation suite — surface-only (A0), LTFS no-gate (A1), TST
(A2), UWA class-only (A3), UWA route-only (A4), UWA combined (A5), PWMP with
indicators (A6), PWMP + upstream urgency (A7), PWMP occupancy-guarded (A8).

**Code:** None. The existing code always runs TST with MP signals — that's
one point in the space, not the suite.

**Where it lives:** `variants/ablation.py` — a `Variant` dataclass with
boolean flags for layers, gate rule, urgency components, signal controller,
and PWMP indicators. The runner applies variant flags to enable/disable
components.

### M2. Scenarios S0–S3 (Sec VIII-B)

**Paper:** Single-layer (S0), LTFS baseline (S1), incident (S2), demand surge
(S3).

**Code:** None. Only one `osm.sumocfg` with one set of route files.

**Where it lives:** `scenarios/` package. Each scenario produces a SUMO config
at runtime by (a) scaling or filtering the base route files and (b) scheduling
TraCI-injected events (lane closures, speed reductions, edge closures). Base
demand already exists in `osm.*.rou.xml`; scenarios manipulate it at depart
time.

### M3. Priority-Weighted Max Pressure (PWMP) signal control

**Paper:** Eq 10 (weight construction), Eq 11 (split-positive pressure), Eq
18 (phase pressure). Central novel contribution (N8).

**Code:** `ltfs_blocks.py:171-211` has a standalone `pwmp_weight_full`
function, but no controller ever imports it or plugs it into phase selection.
Neither `mp_controller.py` nor `mp_ltfs1.0.1_wt.py` instantiate PWMP.

**Where it lives:** `controllers/pwmp.py` — fully wired. Reads variant flags
to toggle indicators `I_gate`, `I_dis`, `I_exp`, `u_up` individually for
A6/A7/A8. Occupancy guard on gate-feed (A8 vs A7) is a config flag.

### M4. Vehicle class urgency with `u_min > 0` floor (Eq 2)

**Paper:** Eq 2 — `u^class_v ∈ [u_min, 1]` with `u_min > 0` (default 0.1) to
prevent any class being permanently locked out.

**Code:** `ltfs_blocks.py:140-150` — `urgency_from_type_id` clamps to [0,1]
but has no nonzero lower bound. A type mapped to 0.0 would violate Eq 2.

**Where it lives:** `config.py` — `U_MIN = 0.1`. `ltfs_blocks.py` enforces
`max(u_min, value)` in `urgency_from_type_id`.

### M5. Route-based urgency (Eq 3) — per-vehicle remaining-length lookup

**Paper:** Eq 3 — `u^route_v(t) = clip(L_rem_v(t) / L_ref)` with `L_ref > 0`.

**Code:** `ltfs_blocks.py:152-161` has the helper function, but no controller
calls it with live vehicle remaining length. `mp_single_baseline_wt.py:181-184`
attempts it but inside the dead `vehicle_u` function (W10).

**Where it lives:** `ltfs_blocks.py` keeps the pure helper; new
`gating/tt_predict.py` computes remaining length for gate decisions; new
`controllers/pwmp.py` uses it for upstream urgency aggregation.

### M6. Wait-based urgency for signals (Eq 4) — `u^wait_v(t)`

**Paper:** Eq 4 — separate urgency component for signal control based on
accumulated wait time `W_v(t) / W_ref`, with `W_ref = 60 s`. Paper explicitly
separates this from route urgency because signal control responds to local
starvation, not remaining distance.

**Code:** None.

**Where it lives:** `ltfs_blocks.py` adds `wait_urgency(W_v, W_ref)`.
`controllers/pwmp.py` queries `traci.vehicle.getAccumulatedWaitingTime(vid)`
to populate `W_v`.

### M7. Signal-oriented urgency (Eq 6) and movement-level aggregation (Eq 8)

**Paper:** Eq 6 combines class + wait urgency for signals:
`u^signal_v(t) = clip(λ_s · u^class_v + (1-λ_s) · u^wait_v(t))`.
Eq 8 aggregates this over all vehicles on the upstream link of a movement:
`ū_up(m,t) = mean over V_m(t)`.

**Code:** None. `ltfs_blocks.py:163-169` has `combined_urgency` but it
combines class + route (which is for gating — Eq 5), not class + wait
(which is for signals — Eq 6). The existing function is correct for Eq 5,
so paper-aligned naming is `combined_urgency_gating`.

**Where it lives:** `ltfs_blocks.py` adds `combined_urgency_signal`
(class + wait, for Eq 6). `controllers/pwmp.py` computes ū_up per controlled
approach.

### M8. Gate admission rule uses actual T̂_S and T̂_X (Eq 23, 24), not free-flow heuristics

**Paper:** Eq 23–24 in Sec VII-A. T̂_S and T̂_X use smoothed link travel-time
estimates maintained by the dynamic routing module (Eq 21).

**Code:** `mp_ltfs1.0.1_wt.py:467-473` computes ΔT using fixed free-flow
speeds (`V_SURFACE_FF = 10.0`, `V_EXPRESS_FF = 20.0`) and the remaining
route's surface/express length. It doesn't use smoothed observations and it
doesn't compute the per-path predicted times the paper defines.

**Where it lives:** `routing/dynamic_tt.py` maintains smoothed τ̂_e(t) via
exponential smoothing (Eq 21). `gating/tt_predict.py` computes T̂_S via
shortest-path on surface-only subgraph weighted by τ̂_e, and T̂_X via
shortest-path through the express layer using τ̂_e for all edges plus gate
transfer times. Computation is cached per vehicle per gate-decision point.

### M9. Express-layer occupancy `o_X(t)` as a dynamic state (N4)

**Paper:** N4 — occupancy is updated from inflows and outflows, constrained
to [0,1].

**Code:** `mp_ltfs1.0.1_wt.py:618-627` computes occupancy from set diff of
vehicles on express edges between steps. `update_occupancy` in
`ltfs_blocks.py:85-118` is the normalizer but its `capacity` parameter is
`EXPRESS_CAPACITY = 400.0` with no justification — 400 what? Vehicles? 
Vehicle-equivalents? The paper gives no number.

**Paper ambiguity surfaces here** (see Ambiguity A2). The rewrite uses total
vehicle count divided by summed express-lane capacity (computed from
network), giving a unit-consistent occupancy in [0,1].

**Where it lives:** `ltfs_blocks.py` cleaned-up `update_occupancy` plus
`express_capacity_from_net` derived from edge lengths and lane counts.

### M10. Dynamic travel-time routing (Sec VI-B, Eq 21)

**Paper:** Exponential smoothing `τ̂_e(t) = ξ·τ_obs + (1-ξ)·τ̂_e(t-1)` with
periodic shortest-path updates (every 30 s).

**Code:** None. SUMO's default static routing runs because no TraCI routing
override is applied.

**Where it lives:** `routing/dynamic_tt.py` — maintains τ̂_e, triggers
rerouting for active vehicles every `REROUTE_PERIOD` seconds via
`traci.vehicle.rerouteTraveltime`.

### M11. Tabular Q-routing (Sec VI-C, Eq 22)

**Paper:** `Q(i,n,d) ← (1-η)Q + η(t_obs + min_{n'} Q(n,n',d))`.

**Code:** None.

**Where it lives:** `routing/q_routing.py`. Because the network has ~3500
edges and 60k+ vehicles, full per-junction per-destination Q-routing would
explode. The rewrite implements it at a coarser granularity: Q-tables are
keyed on (junction, neighbor, destination_zone) where destination_zones are
the exit edges of the network. Vehicles reroute at junction entries every
30 s via `traci.vehicle.setRoute`.

### M12. Free-flow runs executed under each controller logic

**Paper:** Eq 28. One free-flow run per controller π, with all signal/gate
logic active but demand stretched so no congestion interactions occur.

**Code:** `mp_freeflow.py:255-280` does execute both MP and LTFS variants,
but only the TST-hardcoded LTFS. There's no free-flow run for PWMP-A7 or
PWMP-A8 separately, even though Eq 28's "same controller logic" requirement
means each PWMP variant technically needs its own reference.

**Scoping decision** (flagged in AUDIT): in practice, PWMP weights with no
congestion (occupancy ≈ 0, queues ≈ 0) collapse to `w ≡ 1` (classical MP).
So one PWMP free-flow run suffices. Separately, MP and UWA/TST free-flow
runs differ because gating still applies under low demand (Θ still filters).
The rewrite produces: `freeflow_MP.csv`, `freeflow_MP_TST.csv`,
`freeflow_MP_UWA.csv`, `freeflow_PWMP.csv`. See Ambiguity A4.

**Where it lives:** `metrics/freeflow.py` — runner with `--freeflow` flag
replays the scenario with `DEPART_STRETCH = 10` under the specified variant.

### M13. Fixed-time (Webster) controller (Sec V-A)

**Paper:** Standard baseline — cycle length from Webster's formula, splits
proportional to critical flows.

**Code:** None.

**Where it lives:** `controllers/fixed_time.py`. For the Yan'an network,
Webster requires saturation-flow and lane-volume estimates per phase, neither
of which are in the network XML directly. The rewrite implements
**SUMO-native fixed time**: just don't override the TLS program that SUMO
loads from the network. This matches "fixed time as defined in the network"
and is what the paper's fixed-time baseline effectively does in practice.
Full Webster optimization is a separate task, flagged as TODO.

### M14. Actuated controller (Sec V-B)

**Paper:** Gap-based extension of green within min/max bounds.

**Code:** None.

**Where it lives:** `controllers/actuated.py`. Uses SUMO's native actuated
TL type if the network defines it; otherwise runs a minimal gap-based logic
via TraCI. The Yan'an network uses `type="static"` TLSs, so the TraCI
implementation is active by default.

### M15. Class-wise metric reporting (Sec IX, Fig 5)

**Paper:** Sec IX — "class-wise TT/WT by SUMO vehicle type"; Fig 5 is the
class-wise fairness plot.

**Code:** `metrics_wt_all.py` aggregates over all trips; no class breakdown.
Gini is computed over raw travel times, not over class-wise means (Sec IX-D
says "over class wise mean delays or travel times").

**Where it lives:** `metrics/compute.py` — separate reports per vehicle type
plus a Gini computed over class-mean travel times (paper-correct).

### M16. Vehicle-hours and vehicle-kilometres metrics (Sec IX-B)

**Paper:** Sec IX-B — traffic reduction requires *both* mean TT decrease *and*
total vehicle-hours decrease. This is the "traffic reduction is defensible"
guard clause.

**Code:** None. Only mean TT reported.

**Where it lives:** `metrics/compute.py` — computes and reports both;
prints a warning when mean TT decreases but vehicle-hours increases
(the "redistribution, not reduction" failure mode).

### M17. Gate admission rate and express-usage metrics (Sec IX-A)

**Paper:** End of Sec IX-A — admission rate at gates, express usage
(distance/time), occupancy exceedance statistics (fraction of time
`o_X > κ`).

**Code:** None.

**Where it lives:** `metrics/compute.py` plus live logging in
`gating/uwa.py` and `gating/tst.py`.

### M18. Incident injector (S2, Sec VIII-B)

**Paper:** Lane drop or local blockage.

**Code:** None.

**Where it lives:** `scenarios/s2_incident.py` — scheduled TraCI events:
`traci.lane.setDisallowed(lane_id, ["passenger"])` for lane closure,
`traci.edge.setMaxSpeed(edge_id, 2.0)` for speed-reduction incidents.

### M19. Signal constraint enforcement (Sec IV-B)

**Paper:** Min green, max green, intergreen, clearance.

**Code:** `mp_ltfs1.0.1_wt.py:674-681` enforces `MIN_GREEN = 10.0` but no max
green, no intergreen. `traci.trafficlight.setPhase` skips yellow transitions
if the target phase has no amber phase defined in the program.

**Where it lives:** `controllers/base.py` — `PhaseController` class with
`min_green`, `max_green`, `intergreen_s`. Switching always goes through the
network's defined yellow/all-red phases.

---

## UNNECESSARY (dead / redundant)

### U1. `mp_controller.py` — superseded by `mp_single_baseline_wt.py`

277 lines. Same logic minus trip logging. Also has dead `edge_mean_u` with
undefined constants (W10 applies here too). Nothing imports from it.
**Delete.**

### U2. `mp_freeflow_hardcoded.py` — superseded by `mp_freeflow.py`

224 lines. Alternative implementation of free-flow that calls SUMO as a
subprocess. The main `mp_freeflow.py` runs controllers in-process via
`importlib`. Two implementations of the same concept, neither fully correct.
**Delete both; replace with `metrics/freeflow.py`.**

### U3. `metrics_baseline_single.py` — references non-existent functions

51 lines. Imports `from standard_blocks import buffer_index, gini` — neither
function exists in `standard_blocks.py`. Also reads
`baseline_singlelayer_trips.csv` which no controller produces. **Delete.**

### U4. `metrics_wt_compare.py` — superseded by `metrics_wt_all.py`

188 lines. Older single-file implementation of WT metrics, keyed on `routeID`
rather than `routeKey`. `routeID` is rarely populated by duarouter so this
script would match almost no trips. **Delete.**

### U5. `plot_wt_graphs.py` — imports non-existent symbols

91 lines. Imports `FREE_FLOW_CSV`, `BASELINE_CSV`, `LTFS_CSV` from
`metrics_wt_all` — those names don't exist there (it has `FREEFLOW_MP_CSV`,
`FREEFLOW_LTFS_CSV`, `BASELINE_TRIPS_CSV`, `LTFS_TRIPS_CSV`). Would raise
`ImportError`. **Delete; replace with `metrics/plots.py`.**

### U6. `sweep_ltfs_params.py` — imports non-existent symbols

201 lines. Same broken imports as U5 (`FREE_FLOW_CSV`, `BASELINE_CSV`,
`LTFS_CSV`). Would not run. Logic is fine, just the imports are stale.
**Salvage logic into `ltfs/sweep.py` using correct imports; delete original.**

### U7. `detect_route_mismatches.py` — one-off diagnostic

67 lines. Helpful when the `routeKey` mismatch bug (W5) was first detected,
but with W5 fixed the tool's job is done. **Delete or move to `tools/diagnostics.py`
as a dev aid.**

### U8. `check_net_elevation.py`, `list_tls.py`, `list_yanan_tls_ids.py`,
`routechecker.py`, `ltfspercent.py` — one-off exploration scripts

Together ~250 lines. Useful for exploring the network during development.
None are imported anywhere. **Move to `tools/` subfolder; none are required
for the main experiment pipeline.**

### U9. `build.bat`, `run.bat`, `python` (empty file), `desktop.ini`

Windows-specific or OS metadata. `run.bat` is one line (`sumo-gui -c osm.sumocfg`).
`python` is a zero-byte file, probably an accident. **Delete.**

### U10. Three `_freeflow_*_tmp.csv`, `tripinfos.xml`, `stats.xml`, etc.

Output artifacts of past runs checked into the folder. The audit categorizes
these as artifacts; they're fine to keep for comparison but shouldn't live
next to the code. **Move to `results/archive/`.**

### U11. `EXPRESS_SPEED_FAST = 1.3` multiplier

Embodies the "speed bonus ≠ layer" misconception (W3). **Delete.**

### U12. `EXPRESS_CAPACITY = 400.0` hardcoded constant

Numerical value with no justification (W3 / M9). **Replace with
`express_capacity_from_net()` which computes from edge-lane geometry.**

### U13. `REROUTE_DENIED` flag and `maybe_reroute_denied_vehicle`

The flag is `True` by default and the function is called. But with `setEffort`
not consumed by default `findRoute` (W8), the reroute is a no-op in practice.
Remove both; replace with proper pruned-graph fallback.

---

## AMBIGUOUS (paper underspecifies; rewrite makes a choice to flag)

### A1. Turning ratios `ρ_{m,e}` — how are they estimated?

**Paper:** Eq 7 uses them but the experimental protocol (Sec VIII) doesn't
say how to estimate them for the Yan'an network.

**Rewrite choice:** Equal-split across all downstream exits of each movement,
as in the existing code (W11). Documented at `controllers/max_pressure.py:~30`.
Flag to supervisor: acceptable for a first results pass; revisit if MP
underperforms relative to fixed-time and the bottleneck is poor turning
estimates.

### A2. Express-layer capacity for occupancy normalization

**Paper:** N4 says `o_X ∈ [0,1]` but doesn't specify the denominator.

**Rewrite choice:** `capacity = sum over express_edges (lane_count · length / (avg_vehicle_length + headway))`
where avg_vehicle_length + headway = 7.5 m (standard car-following gap). This
gives a physically meaningful "fraction of express-lane-space occupied" in
[0,1]. Implemented in `ltfs_blocks.py`.

### A3. Express layer detection for tunnel portion

**Paper:** Sec III-A says express layer is elevated + tunnel combined into
one functional class. But the detection heuristic in the existing code
(`z ≥ 6 m`) only catches elevated segments; the Yan'an Road Tunnel edges
have z ≈ -40 m, which the existing code excludes.

**Rewrite choice:** The express-edge detector uses `speed ≥ EXPRESS_MIN_SPEED
(16.7 m/s)` OR `median_z ≥ EXPRESS_MIN_Z (6 m)` OR `median_z ≤ EXPRESS_MAX_Z
(-5 m)`. This catches both elevated and underground segments. Lane speed on
the tunnel is typically 16.7 m/s, so it should be caught by the speed
criterion alone on the Yan'an network. **Flag to supervisor: verify tunnel
edges are in the `EXPRESS_EDGES` set by printing the set on startup.**

### A4. Free-flow reference per variant vs per-controller-class

**Paper:** Eq 28 specifies "same policy π" but policy is an overloaded term
that could mean (a) the signal controller (MP vs PWMP), (b) the gating rule
(none/TST/UWA), (c) the full variant identity (A0–A8).

**Rewrite choice:** Compute free-flow per (signal-controller, gate-rule)
pair, not per-variant. Rationale: with stretched departures, occupancy ≈ 0
and queues ≈ 0, so PWMP indicators and UWA urgency all collapse to their
baseline behaviour. Distinct free-flow runs: `MP_nogate`, `MP_TST`, `MP_UWA`,
`PWMP_UWA`. Any remaining negative-WT trips after this matching are flagged
in the metrics output. **Flag to supervisor: verify by comparing, e.g., A6
(PWMP) trips against PWMP free-flow and against MP free-flow; if the
distinction is statistically invisible we can use one reference per gating
rule.**

### A5. Vehicle-class to urgency mapping

**Paper:** Eq 2 says `f_class: C → [u_min, 1]` is a deterministic mapping but
doesn't specify concrete values for cars/buses/trucks.

**Rewrite choice:** Keep the existing mapping from `ltfs_blocks.py`:

```
bus_bus: 0.9, truck_truck: 0.7, veh_passenger: 0.4,
motorcycle_motorcycle: 0.35, bike_bicycle: 0.2, ped_pedestrian: 0.1
```

with enforced floor `u_min = 0.1`. **Flag to supervisor: these are the
paper's implicit convention from the prior work, but justifying them in
Sec III-C would strengthen the paper.**

### A6. PWMP weight parameters `α, β, η, γ, w_max`

**Paper:** Sec V-D, Eq 10 — parameters are free. Sensitivity sweep (Sec
VIII-G) requires `w_max ∈ {1.2, 1.5, 2.0}`.

**Rewrite choice:** Defaults `α=0.5, β=0.7, η=0.2, γ=0.6, w_max=1.5`. These
mirror the standalone function in the old `ltfs_blocks.py` and are reasonable
first guesses (discharge protection `β` largest, express upstream `η`
smallest). **Flag to supervisor: these should be justified with a short
sensitivity analysis before final results submission.**

### A7. Gating thresholds `Θ, Θ_u, κ`

**Paper:** Sec VII — parameters are free. Sensitivity sweep (Sec VIII-G)
exists.

**Rewrite choice:** Defaults `Θ = 60 s`, `Θ_u = 30 s` (chosen so that a
vehicle with `u_v = 0.5` faces effective threshold 60 s, matching TST), 
`κ = 0.85`. **Flag: these inherit from the existing code and are plausible
but not tuned.**

### A8. Reference lengths `L_ref` and `W_ref`

**Paper:** `W_ref = 60 s` stated as default (Eq 4). `L_ref` has no default.

**Rewrite choice:** `W_ref = 60 s` as per paper. `L_ref` = median remaining
route length at departure across all trips in the scenario (computed at
startup). This normalizes so the median trip has `u_route ≈ 0.5`. **Flag
to supervisor: should `L_ref` be fixed per paper or computed per run? The
paper's text "reference distance constant" reads as fixed but doesn't
specify a value.**

---

## Summary counts

- **Wrong:** 11 items
- **Missing:** 19 items
- **Unnecessary:** 13 items
- **Ambiguous (flagged):** 8 items

Lines deleted from old code (dead or superseded): approximately 2400 of 3522.
Lines added (paper-aligned): approximately 1800.

The net effect is a smaller, paper-aligned codebase with no dead symbols, no
hardcoded absolute paths, explicit variant flags, and all nine ablation
variants selectable from the CLI.
