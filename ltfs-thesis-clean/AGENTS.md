# AGENTS.md

## Project goal
This repository contains thesis simulations for a layered traffic flow system using SUMO and Python.

## Working rules
- Do not change research logic unless explicitly asked.
- Prefer minimal, high-confidence edits.
- Preserve controller structure unless the task is specifically to align code with the paper.
- Keep filenames stable unless a rename is required.
- Save raw CSV outputs in `outputs/raw/`.
- Save plots in `outputs/figures/`.
- When working on controller logic, read `notes/paper_controller_spec.md` first.
- Distinguish clearly between:
  - code bugs
  - portability/workflow issues
  - paper-to-code mismatches
- Do not optimize results unfairly. Any performance-improving edit must preserve fair comparison with the baseline.
- For paper-to-code alignment, read `notes/paper_controller_spec.md` first and treat it as the working text specification.
- For paper-to-code alignment, read `notes/paper_controller_spec.md` first.
- Do not optimize results until controller alignment is complete.
- Keep each commit focused and reviewable.
- After every coding task, run the relevant validation checks and summarize remaining gaps.
