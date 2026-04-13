# Paper controller specification

## Purpose
This note is the text source of truth for Codex when auditing paper-to-code alignment.

Important:
- Treat this file as a **working specification** for the current thesis repo.
- Items marked **Confirmed in code** are grounded in the current implementation.
- Items marked **Verify against paper** should be checked against the actual thesis wording before any claim is made in the paper.

---

## 1. Baseline controller

### Intended role
The baseline is a **single-layer Max-Pressure traffic signal controller** on the surface network.

### Confirmed in code
- Phase selection is based on **Max-Pressure**.
- Movement pressure is defined as:

  `P = q_up - Σ_i (rho_i * q_down_i)`

- Phase pressure is the sum of movement pressures.
- The selected phase is the phase with the highest pressure.
- If movement weights are not provided, phase selection is effectively standard unweighted Max-Pressure.

### Current implementation notes
- `standard_blocks.py` supports an optional movement weight `w`.
- In the active LTFS controller path, movement weight `w` is injected for PWMP-enabled phase selection.
- The baseline controller remains plain Max-Pressure unless a controller explicitly injects weights.

### Verify against paper
- Whether the paper defines the baseline strictly as plain Max-Pressure or as a stronger baseline variant.
- Whether the baseline should include dynamic rerouting or only signal control.

---

## 2. LTFS controller

### Intended role
The LTFS controller extends the baseline by introducing a logical **express / elevated layer** and a gate decision for admission into that layer.

### Confirmed in code
- The surface network remains controlled by Max-Pressure.
- An **express layer** is auto-detected from the network file.
- **Gate edges** are auto-detected as surface edges feeding into the express layer.
- Vehicles approaching gate edges are evaluated for admission.
- Denied vehicles may be rerouted away from the express layer.
- Admitted vehicles currently receive an express-layer speed-factor bonus while on express edges.

### Current implementation notes
- Express edges are inferred from network geometry and speed:
  - high speed
  - elevated z-value
  - connected express components retained
- Gate decisions are evaluated periodically, not continuously every simulation micro-step.
- The controller keeps a global scalar express occupancy estimate `o_X`.
- Express speed bonus is **optional** and **OFF by default** for Paper 1 default runs; if enabled, it is an ablation toggle.

### Verify against paper
- Whether the paper intends a logical express layer only, or a more explicit multi-layer infrastructure abstraction.
- Whether the express speed-factor bonus is explicitly justified in the paper or is only an implementation convenience.
- Whether denied-vehicle rerouting is part of the intended LTFS logic or an added heuristic.

---

## 3. TST logic

### Intended meaning
TST is a **Time Saving Threshold** admission rule.

### Confirmed in code
- Estimated time saving is:

  `ΔT = τ_surface - τ_express`

- Admission rule is:

  `admit = (ΔT >= Θ) and (o_X <= κ)`

where:
- `Θ` is the minimum required time saving threshold
- `κ` is the express occupancy cap
- `o_X` is express occupancy in `[0, 1]`

### Current implementation notes
- Positive `ΔT` means the express option is beneficial.
- `ΔT` is currently estimated from simplified surface and express travel-time approximations.
- The current implementation appears to use constant free-flow-like speed assumptions for the estimate, not a full predictive congestion model.

### Verify against paper
- Whether the paper defines `τ_surface` and `τ_express` using:
  - free-flow estimates,
  - current measured traffic state,
  - or a richer prediction model.
- Whether `Θ` should be fixed, scenario-dependent, or class-dependent.

---

## 4. UWA logic

### Intended meaning
UWA appears to be an urgency-weighted admission mode.

### Confirmed in code
- UWA-related helper concepts exist in the repo, but UWA is not active in the default Paper 1 runtime path.

### Current implementation notes
- The current repo contains urgency helper functions in `ltfs_blocks.py`.
- For **Paper 1 default runs**, UWA is **optional** and **inactive by default**.
- Any UWA-enabled run must be treated as an **explicit ablation**, not the default Paper 1 controller.

### Verify against paper
- Whether UWA is a required part of the paper contribution or only an optional extension.
- The exact urgency formula.
- Whether urgency should affect only gate admission or also signal control.

---

## 5. Priority-weighted Max-Pressure

### Intended meaning
Priority-weighted Max-Pressure means movement weights should modify phase pressure, not just gate admission.

### Paper 1 default requirement
- For Paper 1 LTFS-alignment claims, PWMP in the active LTFS phase-selection path is **required**.

### Confirmed in code
- `standard_blocks.py` supports weighted movement pressure through an optional movement field `w`.
- `ltfs_blocks.py` contains urgency and weight helper functions, including:
  - class urgency
  - route urgency
  - combined urgency
  - a bounded PWMP movement weight function

### Current implementation notes
- `standard_blocks.py` supports movement weight `w` and `ltfs_blocks.py` provides urgency/weight helpers.
- The active LTFS controller must inject these movement weights into `choose_phase()` for Paper 1-alignment claims.

### Verify against paper
- Whether the paper claims full PWMP in the main controller.
- Whether weights are required for all intersections or only for selected movements near express gates.
- Whether PWMP is core to Paper 1 or reserved for later work.

---

## 6. Occupancy update

### Intended meaning
The express layer should have a bounded occupancy state to prevent overloading.

### Confirmed in code
- Occupancy update is:

  `o_next = clamp01(o_prev + (inflow - outflow) / capacity)`

- Occupancy is clamped to `[0, 1]`.
- Inflow and outflow are derived from changes in the set of vehicles currently on express edges.

### Current implementation notes
- This is a simple corridor-level occupancy tracker.
- It is not currently a lane-level, edge-level, or corridor-segment-level state.

### Verify against paper
- Whether the paper intends global express occupancy or corridor-specific occupancy.
- Whether the paper requires a more detailed state definition than the current scalar implementation.

---

## 7. Express / elevated network logic

### Confirmed in code
- Express edges are auto-detected from the network using:
  - lane/edge speed
  - z-elevation
  - connected-component filtering
- Gate edges are auto-detected as surface-to-express feeders using:
  - incoming topology
  - controlled traffic-light movements

### Verify against paper
- Whether the paper expects manual corridor definition rather than automatic edge detection.
- Whether the Yan'an elevated corridor is meant to be the primary express corridor in the paper experiments.
- Whether all express entries should be treated equally or categorized by corridor / direction.

---

## 8. Rerouting behavior

### Confirmed in code
- Denied vehicles can be rerouted to avoid the express layer.
- Gate decisions are stored per vehicle so that admission is not re-decided blindly every step.

### Paper 1 default requirement
- Denied-vehicle rerouting is **optional** and **OFF by default** for Paper 1 default runs.
- If enabled, it must be declared and reported as an **ablation toggle**.

### Verify against paper
- Whether rerouting denied vehicles is a required part of the LTFS design.
- Whether rerouting should also occur for admitted vehicles under downstream congestion.
- Whether rerouting frequency and adaptation interval should be part of the reported experiment design.

---

## 9. Current controller classification

Based on the current repo, the implementation is best described as:

**LTFS-lite / partial LTFS**

This means:
- baseline Max-Pressure is active
- express-layer detection is active
- gate admission using TST is active
- occupancy constraint is active
- denied-vehicle rerouting is optional and OFF by default unless ablation-enabled
- express speed bonus is optional and OFF by default unless ablation-enabled
- UWA is optional and inactive by default
- PWMP in active LTFS phase selection is required for Paper 1 LTFS-alignment claims

Use this wording unless a later audit proves stronger paper-to-code alignment.

---

## 10. Experiment design expectations

### Minimum comparison set
The intended thesis comparison should include at least:
- Baseline single-layer Max-Pressure
- LTFS-enabled controller under the same demand and network conditions

### Reproducible run IDs
- Scenario matrix is codified in `notes/paper1_run_matrix.json` with IDs:
  - `S0`, `S1`, `S2`, `S3`
- Ablation matrix is codified in `notes/paper1_run_matrix.json` with IDs:
  - `A0` to `A8`
- Single-command runner:
  - `sim/core/paper1_run.py --scenario <ID>`
  - `sim/core/paper1_run.py --ablation <ID>`
- All non-core toggles used in ablations must be explicit in run config.

### Minimum outputs to report
- mean travel time
- median travel time
- high-percentile travel time such as T95
- throughput / completed trips
- fairness measure if used, such as Gini
- express usage rate:
  - share of trips that touch express edges
  - share admitted at gates
  - denied vs admitted counts

All required Paper 1 outputs (including express usage share and gate admitted/rejected counts) must be emitted in one reproducible reporting step/script.

### Important interpretation rule
Do not claim LTFS improvement unless:
- baseline and LTFS use comparable demand and network conditions
- elevated-route usage is non-trivial
- any parameter tuning is reported honestly

### Verify against paper
- exact scenario names
- exact demand levels
- exact required metrics
- whether robustness, fairness, and ablations are mandatory in Paper 1

---

## 11. Non-negotiable requirements for paper alignment

Before saying the controller matches the paper, verify that all of the following are true:
1. The baseline Max-Pressure formulation matches the paper equations.
2. The LTFS gate rule matches the paper definition of TST and any occupancy constraint.
3. Any claimed urgency-weighted or priority-weighted logic is actually wired into the active controller.
4. Any behavior that materially affects results, such as express speed bonuses or rerouting, is explicitly justified in the paper.
5. The experiment comparisons use the same scenario logic described in the paper.
6. Any non-core toggles (e.g., UWA, denied rerouting, express speed bonus) are explicitly declared and reported as ablations.

---

## 12. Codex instructions for paper-to-code audit

When auditing paper-to-code alignment:
- mark each concept as:
  - fully implemented
  - partially implemented
  - missing
  - ambiguous
- separate:
  - confirmed implementation facts
  - inferred behavior
  - paper requirements still needing verification
- do not upgrade the controller description beyond "LTFS-lite / partial LTFS" unless the audit proves it.
