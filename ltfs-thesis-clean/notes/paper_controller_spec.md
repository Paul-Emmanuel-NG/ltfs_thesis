# Paper controller specification

## Purpose
This file is the Paper 1 source-of-truth specification for code alignment and audit.

The goal of Paper 1 is not to introduce RL. It is to establish a reproducible non-RL LTFS baseline on a real-city Shanghai subnetwork with clear standard-vs-novel separation, strong baselines, and a simulation protocol that later papers can build on.

---

## 1. Network and LTFS structure

### Two-layer network
The modeled road network is a directed graph with:
- surface layer `S`: signalized urban streets
- express layer `X`: higher-speed limited-access links
- gate nodes connecting the two layers

Vehicles may:
- enter the express layer only at designated entry gates
- exit only at defined exit gates
- otherwise remain layer-consistent

### Layer consistency
Feasible LTFS paths must respect:
- Surface -> Express -> Surface
- no illegal jumps between layers
- no express access except through gates

### Baseline scope
Paper 1 is strictly non-RL.
It establishes:
- signal baselines
- routing baselines
- gating baselines
- evaluation methodology

---

## 2. Vehicle urgency model

Each vehicle has combined urgency:

`u_v(t) = λ u_class_v + (1 - λ) u_route_v(t)`

where:
- `u_class_v` is a normalized class urgency from vehicle type
- `u_route_v(t)` is normalized route-based urgency from remaining route length
- `λ ∈ [0,1]`

### Route urgency
Route urgency is based on remaining route length:
`u_route_v(t) = clip[0,1](L_rem_v(t) / L_ref)`

### Mean upstream urgency for a movement
For movement `m`, upstream urgency is the mean urgency of vehicles currently queued on that movement:
- if the movement queue is empty, upstream urgency is 0

---

## 3. Surface-layer signal controllers

Paper 1 baseline set includes four surface-layer controllers:
1. Fixed Time (Webster style)
2. Actuated
3. Classical Max Pressure (MP)
4. Priority-Weighted Max Pressure (PWMP)

### Classical Max Pressure
For each movement `m` at intersection `i`:

`π_i,m(t) = q_i,m(t) - Σ_{e in down(m)} ρ_m,e q_e(t)`

Classical phase pressure:

`P_i^MP(φ,t) = Σ_{m in mov(φ)} π_i,m(t)`

Selected phase:

`φ_i*(t) ∈ arg max_{φ in Φ_i} P_i^MP(φ,t)`

subject to standard minimum-green and switching constraints.

### Important code-alignment requirement
If the active controller claims to implement PWMP, it must not remain plain unweighted MP.

---

## 4. Priority-Weighted Max Pressure (PWMP)

PWMP is a required Paper 1 concept, not optional wording.

### Movement indicators
Each movement may be labeled by deterministic indicators:
- `I_gate(m)`: gate-feed movement
- `I_dis(m)`: discharge movement from express back to surface
- `I_exp(m)`: movement whose upstream link is on the express layer

### LTFS movement weight
The bounded LTFS movement weight is:

`w_m(t) = clip[1,w_max](1 + α I_gate(m) I[o_X(t) ≤ κ] + β I_dis(m) + η I_exp(m) + γ u_up(m,t))`

where:
- `o_X(t)` is express occupancy
- `κ` is the occupancy cap
- `α, β, η, γ ≥ 0`
- `u_up(m,t)` is mean upstream urgency
- weights are clipped to `[1, w_max]`

### LTFS movement pressure
`π_i,m^LTFS(t) = w_m(t) π_i,m(t)`

### PWMP phase pressure
`P_i^PWMP(φ,t) = Σ_{m in mov(φ)} w_m(t) π_i,m(t)`

Selected phase:

`φ_i*(t) ∈ arg max_{φ in Φ_i} P_i^PWMP(φ,t)`

subject to standard switching constraints.

### Non-negotiable alignment rule
If Paper 1 alignment is claimed, the active LTFS controller must actually inject `w_m(t)` into phase selection.

Helper functions alone are not sufficient.

---

## 5. Inter-layer gate rules

Paper 1 requires two non-RL gate rules.

### TST gating
Predicted remaining travel-time saving for vehicle `v` at time `t`:

`ΔT_v(t) = T̂_S(v,t) - T̂_X(v,t)`

TST admission rule:
- admit if `ΔT_v(t) ≥ Θ`
- and `o_X(t) ≤ κ`

Otherwise the vehicle remains on the surface path.

### UWA gating
UWA admission rule:
- admit if `u_v(t) ΔT_v(t) ≥ Θ_u`
- and `o_X(t) ≤ κ`

### Important alignment rule
If UWA is present in active code, the active code must compute and use:
- `u_class`
- `u_route`
- combined urgency `u_v`
- the weighted gate test `u_v ΔT_v`

A dead or incomplete UWA branch does not count as implementation.

---

## 6. Occupancy and gate constraints

### Express occupancy
Express-layer occupancy `o_X(t)` is updated from inflow and outflow and constrained to `[0,1]`.

### Occupancy cap
Gate admission must respect `o_X(t) ≤ κ`.

### Gate safety
Gate decisions must also obey:
- merge capacity limits
- diverge capacity limits
- headway / safety constraints at entries

### Optional extension
Paper 1 allows, but does not require for the minimal core controller:
- quota / credit fairness mechanisms
- occupancy-penalized gate utility

These should be treated as optional or ablation features unless explicitly activated.

---

## 7. Routing baselines

Paper 1 baseline set includes:
1. static shortest path routing
2. dynamic travel-time routing with smoothed travel times
3. tabular Q-routing

### Dynamic routing
Smoothed link travel time:
`τ̂_e(t) = ξ τ_e^obs(t) + (1 - ξ) τ̂_e(t-1)`

### Q-routing
Tabular Q-routing update is part of the baseline comparison set, but Paper 1 remains non-deep and non-RL in controller design.

---

## 8. Scenarios and ablation structure

### Main scenarios
Paper 1 defines:
- `S0`: single-layer baseline, no express layer
- `S1`: LTFS baseline
- `S2`: incident variant
- `S3`: demand-surge variant

### Ablation suite
Paper 1 defines the ablation structure A0 to A8, including:
- surface-only MP
- LTFS with no gating
- LTFS with TST + MP
- LTFS with UWA + MP under different urgency definitions
- LTFS with UWA + PWMP
- occupancy-guarded full LTFS variants

### Important alignment rule
The code should be able to represent at least the logic needed for:
- MP baseline
- TST + MP
- UWA + MP
- UWA + PWMP
- occupancy-guarded PWMP variant

If these cannot be instantiated, Paper 1 alignment is incomplete.

---

## 9. Evaluation protocol requirements

### Minimum experiment structure
A typical experiment run should include:
1. initialization / warm-up
2. fixed simulation period
3. per-vehicle and per-link logging
4. repeated runs across seeds

### Required output dimensions
Paper 1 evaluation must cover:
- efficiency
- reliability
- fairness
- robustness
- gate admission behavior
- express-layer usage

### Required metrics
Core reported metrics include:
- mean travel time
- median travel time
- P95 travel time
- buffer index
- mean wasted time
- Gini over class-wise TT or delay
- gate admission
- express use

---

## 10. Policy-conditioned free-flow benchmark

Paper 1 requires policy-conditioned free-flow benchmarking.

### Wasted time definition
For policy `π`:

`WT_π = TT_π - TT_π^ff`

### Policy-conditioned free-flow
`TT_π^ff(r,c)` must be computed from uncongested runs under the same policy logic `π`, not from:
- a signal-free reference
- a speed-limit-only reference
- another policy’s free-flow run

### Important alignment rule
WT reporting is only paper-aligned if the free-flow reference is policy-consistent.

---

## 11. Safe controller labels for audit

Use these labels during code audit:

### Fully paper-aligned LTFS
Use only if all of the following are true:
- active PWMP weighting is wired into phase selection
- TST is correctly implemented
- UWA is correctly implemented if activated
- occupancy update and cap are active
- layer consistency is respected
- policy-conditioned free-flow benchmarking is used
- scenario and ablation logic can be represented

### Partially aligned LTFS
Use if:
- MP + TST + occupancy are active
- but PWMP or UWA is incomplete
- or experiment structure cannot represent the paper design cleanly

### Not paper-aligned
Use if active code materially contradicts the equations or experiment structure above.

---

## 12. Audit instructions for Codex

When auditing code against this file:
- mark each concept as:
  - fully implemented
  - partially implemented
  - intentionally optional
  - missing
  - ambiguous
- distinguish helper-code availability from active-controller wiring
- do not treat a branch as implemented unless it is callable and complete in the active controller
- do not claim Paper 1 alignment if PWMP exists only in helper functions
- do not claim Paper 1 alignment if UWA is incomplete but presented as active
