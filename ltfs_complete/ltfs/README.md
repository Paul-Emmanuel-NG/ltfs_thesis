# LTFS Simulation Framework — Paper-Aligned Build

Complete simulation framework for Paper 1 (Layered Urban Traffic Flow System).
Replaces the prior `2025-12-02-16-58-29/` scaffolding. Network files
(`yanan_elevated.net.xml`, `osm.*.rou.xml`, `osm.sumocfg`) are unchanged.

## What's in the box

```
ltfs/
  config.py               # All paper parameters, equation-cited
  standard_blocks.py      # S1-S9: classical MP with split-positive form (Eq 11/18)
  ltfs_blocks.py          # N1-N9: urgency, PWMP weight, occupancy, admission
  net_utils.py            # Express + gate detection (elevated AND tunnel)
  route_key.py            # Per (O, D, class) keying (Eq 28 fix)
  runner.py               # Single-run CLI
  run_experiment.py       # Batch runner with presets
  report.py               # Aggregation → Table II + Fig 3, Fig 5

  controllers/
    base.py               # Controller interface + TLS config builder
    fixed_time.py         # Sec V-A
    actuated.py           # Sec V-B
    max_pressure.py       # Sec V-C
    pwmp.py               # Sec V-D — fully wired, per-variant indicator flags

  routing/
    static_dijkstra.py    # Sec VI-A (no-op, SUMO default)
    dynamic_tt.py         # Sec VI-B, Eq 21
    q_routing.py          # Sec VI-C, Eq 22

  gating/
    tt_predict.py         # Sec VII-A, Eq 23-24 (via sumolib shortest path)
    tst.py                # Sec VII-B, Eq 25
    uwa.py                # Sec VII-C, Eq 15/26 (with all urgency modes)
    rejection.py          # Sec VII-D (deterministic, no retry)

  scenarios/
    base.py               # S0/S1/S2/S3 with incident scheduler

  variants/
    ablation.py           # A0..A8 Variant dataclasses

  metrics/
    compute.py            # TT, WT, BI, Gini, class-wise, vehicle-hours
    freeflow.py           # Policy-conditioned Eq 28
```

## Setup

Put the `ltfs/` directory at the same level as `osm.sumocfg`:

```
<project-root>/
  osm.sumocfg
  yanan_elevated.net.xml
  osm.passenger.rou.xml
  ...
  ltfs/
    config.py
    ...
```

Requires: `sumo`, `sumolib`, `traci`, `matplotlib` (for report figures).

## Running

### 1. Pipeline sanity check (do this first)

```bash
python -m ltfs.runner --variant A0 --scenario S1 --seed 1 --sim-time 1800
```

30 minutes of sim time, no gating, plain MP. If this produces a non-empty
`results/A0_S1_seed1_trips.csv`, the pipeline works.

### 2. Free-flow calibrations

Run automatically by `run_experiment.py`, but you can also run them directly:

```bash
python -m ltfs.runner --variant A0 --scenario S1 --seed 1 --freeflow  # MP, no gate
python -m ltfs.runner --variant A2 --scenario S1 --seed 1 --freeflow  # MP + TST
python -m ltfs.runner --variant A5 --scenario S1 --seed 1 --freeflow  # MP + UWA
python -m ltfs.runner --variant A8 --scenario S1 --seed 1 --freeflow  # PWMP + UWA
```

Each produces a `results/freeflow_*.csv` keyed on `(routeKey, class)`.

### 3. Main experiment

Three presets based on compute budget:

```bash
# Tight: A0, A2, A8 × S1 × 3 seeds + 3 freeflows  (~12 runs, defensible for Paper 1)
python -m ltfs.run_experiment --preset tight

# Full: all A0..A8 × S1 × 3 seeds + 5 freeflows   (~32 runs, main claim)
python -m ltfs.run_experiment --preset full

# All: A0..A8 × S1,S2,S3 × 3 seeds                (~86 runs, full ablation + robustness)
python -m ltfs.run_experiment --preset all

# Custom:
python -m ltfs.run_experiment --variants A0 A2 A8 --scenarios S1 --seeds 1 2 --sim-time 3600
```

### 4. Aggregate results

```bash
python -m ltfs.report
```

Produces `results/results_table.csv` (Table II structure) and
`results/figures/*.png` (Figs 3 and 5).

## Key paper-alignment facts

- **PWMP uses the split-positive form** `w·[π]⁺ + [π]⁻` (Eq 11/18). When w≡1,
  it collapses exactly to classical MP. This makes A0–A5 vs A6–A8 a clean
  strict generalization. (AUDIT W1 — most consequential fix.)
- **UWA is fully implemented** with four urgency modes (A3 class-only, A4
  route-only, A5 class+route, "none" reduces to TST). (AUDIT W2.)
- **The express layer is not a speed bonus.** Admission affects routing,
  not `speedFactor`. (AUDIT W3.)
- **Denied vehicles commit to surface paths** for the remainder of the
  trip, no retry at downstream gates. (AUDIT W4, Sec VII-D.)
- **Route keys are per (O, D)**, not per-exact-path. Free-flow references
  match actual congested trips. (AUDIT W5.)
- **Movement queues use the lanes that serve the movement**, not whole-edge
  counts. (AUDIT W6.)
- **Traffic-reduction claim is gated on both mean TT ↓ and vehicle-hours ↓**
  per Sec IX-B; `report.py` prints warnings when they disagree.

## Ambiguities flagged (AUDIT.md items A1–A8)

These are places the paper underspecifies. I made a choice in code and noted
the rationale in the file where the choice lives. Review before final
submission:

| # | Ambiguity | Resolution |
|---|-----------|------------|
| A1 | Turning ratios ρ_m,e | Equal-split across downstream exits |
| A2 | Express capacity | `total_lane_length / 7.5m` (vehicle + headway) |
| A3 | Tunnel detection | `z ≤ -5` OR `z ≥ 6` with speed ≥ 16.7 m/s |
| A4 | Free-flow per policy | One per `(signal_controller, gate_rule)` |
| A5 | Class urgency values | bus=0.9, truck=0.7, passenger=0.4, moto=0.35, bike=0.2, ped=0.1 |
| A6 | PWMP params α,β,η,γ,w_max | 0.5, 0.7, 0.2, 0.6, 1.5 |
| A7 | Θ, Θ_u, κ | 60s, 30s, 0.85 |
| A8 | L_ref | 3000 m |

## Known gaps (vs paper)

- **Detector-based actuated control** (Sec V-B): the Yan'an network has no
  detectors, so `ActuatedController` uses a minimal TraCI-based gap logic.
  Functional but not SCATS/SCOOT-grade.
- **Q-routing** runs at coarse granularity (updates every 30 s via
  `rerouteTraveltime`, not per-junction per-vehicle) to stay tractable on
  60k+ vehicles. See `routing/q_routing.py` header.
- **Sensitivity sweeps** (Sec VIII-G): infrastructure is there (variant +
  CLI flags + `config.py`) but there's no one-shot sweep script. Add one
  if reviewer asks.
- **Credit-based fairness** (N5, Sec VII-F): paper marks this optional;
  not implemented.

See `AUDIT.md` for the full mapping of paper ↔ code.
