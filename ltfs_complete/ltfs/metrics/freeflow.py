"""
metrics/freeflow.py — Eq 28.

Policy-conditioned free-flow generator: runs the same controller logic with
demand stretched so congestion interactions vanish, then takes the minimum
observed travel time per (routeKey, class) pair as the reference.

Ambiguity A4 resolution: we produce ONE reference per (signal_controller,
gate_rule) pair because PWMP weights collapse to 1 under zero occupancy and
zero queues. Four references suffice:

    freeflow_MP_nogate.csv      — A0, A1
    freeflow_MP_TST.csv         — A2
    freeflow_MP_UWA.csv         — A3, A4, A5
    freeflow_PWMP_UWA.csv       — A6, A7, A8

The runner accepts a --freeflow flag that stretches departs at the scenario
level (see scenarios/base.py::Scenario.demand_scale would be wrong here —
we actually need TIME stretching, not scale reduction, to preserve the full
route coverage).
"""

from __future__ import annotations
import csv
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Dict, Tuple


def triplog_to_freeflow(trips_csv: str, out_csv: str) -> int:
    """
    Read a trips CSV produced by the runner's --freeflow mode and collapse
    to per-(routeKey, class) minima per Eq 28.

    Returns the number of unique (routeKey, class) pairs written.
    """
    mins: Dict[Tuple[str, str], float] = {}
    p = Path(trips_csv)
    if not p.exists():
        raise FileNotFoundError(f"Expected free-flow trips at {p}")

    with open(p, newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            rk = (row.get("routeKey") or "").strip()
            cls = (row.get("class") or "").strip()
            if not rk:
                continue
            try:
                tt = float(row.get("travel_time", "nan"))
            except ValueError:
                continue
            if tt <= 0:
                continue
            key = (rk, cls)
            if key not in mins or tt < mins[key]:
                mins[key] = tt

    outp = Path(out_csv)
    outp.parent.mkdir(parents=True, exist_ok=True)
    with open(outp, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["routeKey", "class", "TT_freeflow"])
        for (rk, cls), tt in mins.items():
            w.writerow([rk, cls, tt])
    return len(mins)


def freeflow_name_for_variant(variant) -> str:
    """
    Map a variant onto the appropriate free-flow reference filename
    (Ambiguity A4).
    """
    sig = "PWMP" if variant.uses_pwmp() else "MP"
    gate = variant.gate_rule
    if gate == "none" or gate == "open":
        gate = "nogate"
    elif gate == "tst":
        gate = "TST"
    elif gate == "uwa":
        gate = "UWA"
    return f"freeflow_{sig}_{gate}.csv"
