# Paper controller specification

## Purpose
This file is the **Paper 1 source-of-truth specification for code alignment and audit**.

Paper 1 is a **non-RL** study. Its role is to establish a reproducible LTFS-vs-baseline evaluation protocol on the current SUMO network and demand setup, with explicit defaults and explicit ablations.

Use this file to distinguish:
- what exists in repository code,
- what is active by default in Paper 1 runs,
- what is optional/ablation-only,
- what is not active in the current Paper 1 runtime path.

---

## 1. Scope and baseline policy

### Confirmed in code
- Baseline controller is a signal-control controller using classical Max-Pressure phase selection (unweighted in active baseline path).
- LTFS controller extends the baseline path with express-layer detection, gate logic, occupancy tracking, and PWMP movement weighting in LTFS phase choice.

### Paper 1 default active behavior
- Paper 1 runtime is non-RL.
- Baseline run and LTFS run are both executed in the reproducible runner workflow unless explicitly skipped.

### Not active / out of scope for default Paper 1 path
- RL controllers are not part of active Paper 1 controller path.

---

## 2. Network and LTFS structure

### Confirmed in code
- Logical express layer is auto-detected from the network file using speed/elevation thresholds plus connected-component filtering.
- Gate edges are auto-detected as surface-to-express feeders from topology plus TLS-controlled movements.
- Occupancy is tracked as a bounded scalar state for express usage control.

### Paper 1 default active behavior
- Gate decisions use the detected express/gate sets in runtime.
- Express occupancy constraint is active in admission logic.

### Notes
- This is a logical layering over one SUMO network, not a separate physical simulator layer object.

---

## 3. Surface signal control equations

### Classical Max-Pressure (MP)
For movement \(m\) at intersection \(i\):
\[
\pi_{i,m}(t) = q_{i,m}(t) - \sum_{e \in down(m)} \rho_{m,e} q_e(t)
\]

Phase pressure:
\[
P_i^{MP}(\phi,t) = \sum_{m \in mov(\phi)} \pi_{i,m}(t)
\]

Selected phase:
\[
\phi_i^*(t) \in \arg\max_{\phi \in \Phi_i} P_i^{MP}(\phi,t)
\]
subject to minimum-green/switching constraints.

### Confirmed in code
- Baseline active path uses unweighted MP phase scoring.
- LTFS path modifies movement pressure with PWMP weights before phase selection.

---

## 4. PWMP (Priority-Weighted Max-Pressure)

### LTFS movement weight form
\[
w_m(t)=clip_{[1,w_{\max}]}\left(1+\alpha I_{gate}(m)I[o_X(t)\le\kappa]+\beta I_{dis}(m)+\eta I_{exp}(m)+\gamma u_{up}(m,t)\right)
\]

Weighted movement pressure:
\[
\pi^{LTFS}_{i,m}(t)=w_m(t)\,\pi_{i,m}(t)
\]

PWMP phase pressure:
\[
P_i^{PWMP}(\phi,t)=\sum_{m\in mov(\phi)} w_m(t)\pi_{i,m}(t)
\]

### Confirmed in code
- LTFS active phase selection injects `w` into movement dictionaries passed to phase chooser.
- Weight terms include gate-feed/discharge/express-up indicators and urgency-related term.

### Paper 1 default active behavior
- PWMP wiring in LTFS phase choice is active.

### Important limitation (current implementation detail)
- Active LTFS urgency in PWMP is based on edge-mean class urgency from vehicle types on upstream edges.
- Route-length urgency helpers exist in repo but are not fully wired as active movement urgency in LTFS default runtime.

---

## 5. Vehicle urgency model

### Feature exists in code
- Helper functions exist for:
  - class urgency,
  - route urgency from remaining route length,
  - combined urgency \(u_v(t)=\lambda u_{class,v}+(1-\lambda)u_{route,v}(t)\).

### Paper 1 default active behavior
- In LTFS phase weighting, active urgency usage is class-based edge mean (via active LTFS path).
- Full per-vehicle combined urgency is not the active default phase-weight input.

### Alignment rule
- Do not claim full combined-urgency active control unless route-based urgency is explicitly wired into active controller decisions.

---

## 6. Inter-layer gate rules

### TST gating (active default)
Predicted remaining travel-time saving:
\[
\Delta T_v(t)=\hat T_S(v,t)-\hat T_X(v,t)
\]

Admission:
- admit if \(\Delta T_v(t)\ge \Theta\)
- and \(o_X(t)\le \kappa\)

### UWA gating
Conceptual UWA rule:
- admit if \(u_v(t)\Delta T_v(t)\ge \Theta_u\)
- and \(o_X(t)\le \kappa\)

### Confirmed in code
- Active LTFS runtime **forces gate mode to TST** for Paper 1 defaults.
- UWA mode request is not executed as active gate mode in current default controller path.

### Paper 1 default active behavior
- TST is active.
- UWA is not active in default Paper 1 runtime path.

---

## 7. Occupancy and gate constraints

### Confirmed in code
- Express occupancy is updated from inflow/outflow and clamped to \([0,1]\).
- Admission checks occupancy cap \(o_X \le \kappa\).

### Paper 1 default active behavior
- Occupancy-constrained gate admission is active.

### Not claimed as active by default
- Explicit merge/diverge/headway micro-safety gate constraints are not separately modeled as dedicated gate-constraint modules in current Paper 1 path.

---

## 8. Optional result-affecting behaviors (must be declared)

### Denied-vehicle rerouting
- Feature exists in code.
- **Default: OFF** in Paper 1 runs.
- If enabled, treat as explicit ablation.

### Express speed bonus for admitted vehicles
- Feature exists in code.
- **Default: OFF** in Paper 1 runs.
- If enabled, treat as explicit ablation.

### Alignment rule
Any result-affecting optional toggle must be declared in run config and reported.

---

## 9. Reproducible scenarios and ablations

### Confirmed in code artifacts
- Scenario/ablation matrix is codified in `notes/paper1_run_matrix.json` (S0–S3, A0–A8).
- Runner `sim/core/paper1_run.py` selects scenario/ablation, applies env overrides, runs baseline+LTFS (+metrics unless skipped), validates required outputs, and archives run outputs.

### Paper 1 default active behavior
- Defaults are those from controller files plus any selected matrix env overrides.
- Non-core toggles are OFF unless a matrix entry enables them.

### Important wording constraint
- Do not over-interpret matrix IDs as paper-final semantic labels unless validated against thesis text.

---

## 10. Policy-conditioned free-flow and wasted time (WT)

### Definition
For policy \(\pi\):
\[
WT_\pi = TT_\pi - TT_\pi^{ff}
\]

### Alignment requirement
- \(TT_\pi^{ff}\) must come from uncongested runs under the same policy logic \(\pi\), not from unrelated references.

### Confirmed in code
- Dedicated free-flow generation script exists and metrics consume policy-conditioned free-flow CSVs.
- Metrics script computes TT/WT and reports required LTFS debug outputs (including express usage and gate admit/reject counts) when available.

---

## 11. Current controller classification label

Use this default label unless stronger evidence is established:

## **LTFS-lite / partial LTFS (Paper 1 runtime)**

Meaning in current default path:
- Baseline MP active (unweighted baseline path).
- LTFS express detection active.
- TST gating active.
- Occupancy update/cap active.
- PWMP movement weighting active in LTFS phase selection.
- Denied rerouting optional (default OFF).
- Express speed bonus optional (default OFF).
- UWA not active in default runtime path (TST enforced).

---

## 12. Audit instructions

When auditing code against this file:
- classify each concept as:
  - fully implemented,
  - partially implemented,
  - intentionally optional,
  - missing,
  - ambiguous.
- separate:
  - confirmed implementation facts,
  - inferred behavior,
  - paper-text assumptions requiring thesis confirmation.
- distinguish helper-function existence from active-controller wiring.
- do not claim full Paper 1 alignment for features that are present but not active in runtime path.