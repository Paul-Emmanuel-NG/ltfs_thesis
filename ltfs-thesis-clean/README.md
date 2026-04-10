# LTFS Thesis Simulation Project

This repository contains the current simulation assets for the thesis paper on a layered traffic flow system using SUMO and Python.

## Structure
- `paper/` thesis paper PDF
- `sim/core/` main controllers, metrics, and experiment scripts
- `sim/network/` SUMO configuration and network files
- `sim/routes/` route demand files
- `sim/utils/` diagnostics and helper scripts
- `outputs/raw/` generated CSV outputs from prior runs
- `outputs/figures/` generated plots from prior runs
- `archive/` old or non-core files kept for reference
- `notes/` space for experiment logs and thesis notes

## Current core scripts
- `mp_single_baseline_wt.py`: baseline Max-Pressure run with trip logging
- `mp_ltfs1.0.1_wt.py`: LTFS-enabled run with trip logging
- `mp_freeflow.py`: free-flow reference generation
- `metrics_wt_all.py`: TT and WT summary metrics
- `sweep_ltfs_params.py`: LTFS parameter sweep

## Known cleanup still needed before reruns
- Several scripts still contain hard-coded Windows paths.
- `sweep_ltfs_params.py` appears inconsistent with names used in `metrics_wt_all.py`.
- `freeflow_ltfs.csv` is not present in the current outputs bundle.

## Codex guidance
Start with diagnosis only. Do not refactor the research logic until the project is made portable.
