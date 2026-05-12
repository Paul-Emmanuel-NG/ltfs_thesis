# mp_freeflow.py
# Policy-conditioned free-flow generator (Option 1A + Option B) cincin
#
# What this script does (no env vars, run directly):
#   python mp_freeflow.py
#
# It generates two free-flow references under *the same control policy* used in evaluation:
#   1) freeflow_mp.csv   : uncongested run under baseline Max-Pressure policy
#   2) freeflow_ltfs.csv : uncongested run under LTFS policy
#
# Uncongested condition is achieved by stretching all departure times (keeps ALL vehicles, avoids dropping trips).
# Traffic signals are NOT disabled (signal physics preserved).
#
# Output CSV schema (both files):
#   routeKey,class,TT_freeflow
#
# These files are intended to be consumed by metrics_wt_all.py (policy-conditioned WT).

from __future__ import annotations

import os
import csv
import tempfile
import importlib
import importlib.util
import importlib.machinery
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Tuple

import pandas as pd


# =========================
# HARD-CODED SETTINGS
# =========================

# Run from your experiment folder (same folder as osm.sumocfg)
SUMO_CFG_IN = r"C:\Users\akinw\Desktop\thesis\2025-12-02-16-58-29\osm.sumocfg"
NET_FILE = r"C:\Users\akinw\Desktop\thesis\2025-12-02-16-58-29\yanan_elevated.net.xml"

# Uncongested via departure stretching (recommended 10–20)
DEPART_STRETCH = 10.0

# Buffer added to the latest stretched depart time to ensure everyone arrives
SIM_END_BUFFER = 20000.0

# Where to write policy-conditioned free-flow references
OUT_MP = "freeflow_mp.csv"
OUT_LTFS = "freeflow_ltfs.csv"

# Temporary trip logs produced by the controllers
TMP_TRIPS_MP = "_freeflow_mp_trips_tmp.csv"
TMP_TRIPS_LTFS = "_freeflow_ltfs_trips_tmp.csv"

# Controller modules (local files in the folder)
BASELINE_CONTROLLER_FILE = "mp_single_baseline_wt.py"
LTFS_CONTROLLER_FILE = "mp_ltfs1.0.1_wt.py"


# =========================
# Utilities: SUMO cfg parsing / rewriting
# =========================

def _read_sumocfg_paths(sumocfg_path: str) -> Tuple[str, List[str]]:
    """
    Returns:
      net_file (as in sumocfg),
      route_files list (split by comma)
    """
    tree = ET.parse(sumocfg_path)
    root = tree.getroot()

    # SUMO sumocfg structure: <configuration><input>...</input></configuration>
    input_node = root.find("input")
    if input_node is None:
        raise ValueError(f"No <input> section found in {sumocfg_path}")

    net_node = input_node.find("net-file")
    if net_node is None or "value" not in net_node.attrib:
        raise ValueError(f"No <net-file value='...'> found in {sumocfg_path}")
    net_file = net_node.attrib["value"].strip()

    rf_node = input_node.find("route-files")
    if rf_node is None or "value" not in rf_node.attrib:
        raise ValueError(f"No <route-files value='...'> found in {sumocfg_path}")
    route_files = [x.strip() for x in rf_node.attrib["value"].split(",") if x.strip()]

    return net_file, route_files


def _stretch_route_file(route_file: str, stretch: float) -> Tuple[str, float]:
    """
    Creates a temporary .rou.xml with all depart times multiplied by stretch.
    Returns (tmp_path, max_depart_stretched).
    """
    tree = ET.parse(route_file)
    root = tree.getroot()

    max_depart = 0.0

    # Vehicles
    for veh in root.findall("vehicle"):
        if "depart" in veh.attrib:
            d = float(veh.attrib["depart"])
            d2 = d * stretch
            veh.attrib["depart"] = str(d2)
            if d2 > max_depart:
                max_depart = d2

    # Persons (if any)
    for person in root.findall("person"):
        if "depart" in person.attrib:
            d = float(person.attrib["depart"])
            d2 = d * stretch
            person.attrib["depart"] = str(d2)
            if d2 > max_depart:
                max_depart = d2

    tmp = tempfile.NamedTemporaryFile(delete=False, suffix=".rou.xml")
    tree.write(tmp.name, encoding="utf-8", xml_declaration=True)
    return tmp.name, max_depart


def _write_temp_sumocfg(sumocfg_in: str, stretched_route_files: List[str]) -> str:
    """
    Writes a temporary sumocfg identical to sumocfg_in except route-files are replaced.

    IMPORTANT for Windows + temp cfg:
      If the original .sumocfg references GUI/view settings with a *relative* path (e.g. osm.view.xml),
      SUMO will try to resolve that path relative to the *temporary cfg directory* and fail.
      Because this free-flow script forces USE_GUI=False for controller runs, we safely strip <gui_only>
      from the temporary cfg to avoid any view-settings dependency.

    Returns tmp_cfg_path.
    """
    tree = ET.parse(sumocfg_in)
    root = tree.getroot()

    # 1) Strip GUI-only section so SUMO won't try to load relative view settings from temp dir
    gui_node = root.find("gui_only")
    if gui_node is not None:
        root.remove(gui_node)

    input_node = root.find("input")
    if input_node is None:
        raise ValueError(f"No <input> section found in {sumocfg_in}")

    # 2) Replace route-files with stretched temp route files
    rf_node = input_node.find("route-files")
    if rf_node is None:
        raise ValueError(f"No <route-files> found in {sumocfg_in}")
    rf_node.attrib["value"] = ",".join(stretched_route_files)

    # 3) Make other file paths absolute (defensive) so temp cfg is self-contained
    base_dir = Path(sumocfg_in).resolve().parent

    def _abspath_list(value: str) -> str:
        parts = [p.strip() for p in value.split(",") if p.strip()]
        fixed = []
        for p in parts:
            pp = Path(p)
            if not pp.is_absolute():
                cand = (base_dir / pp).resolve()
                if cand.exists():
                    fixed.append(str(cand))
                    continue
            fixed.append(p)
        return ",".join(fixed)

    for tag in ["net-file", "additional-files", "route-files"]:
        node = input_node.find(tag)
        if node is not None and "value" in node.attrib:
            node.attrib["value"] = _abspath_list(node.attrib["value"])

    tmp_cfg = tempfile.NamedTemporaryFile(delete=False, suffix=".sumocfg")
    tree.write(tmp_cfg.name, encoding="utf-8", xml_declaration=True)
    return tmp_cfg.name
# =========================
# Utilities: import controller modules by filename
# =========================

def _load_module_from_file(py_file: str, module_name: str):
    """
    Load a Python module from an arbitrary .py file path.
    """
    loader = importlib.machinery.SourceFileLoader(module_name, py_file)
    spec = importlib.util.spec_from_loader(loader.name, loader)
    if spec is None:
        raise ImportError(f"Could not load spec for {py_file}")
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)  # type: ignore
    return module


# =========================
# Build TT_ff lookups from controller trip logs
# =========================

def _triplog_to_freeflow_csv(trips_csv: str, out_csv: str) -> None:
    """
    trips_csv expected columns:
      routeKey, class, travel_time (plus others)
    We compute TT_freeflow(routeKey,class) = min travel_time and write:
      routeKey,class,TT_freeflow
    """
    df = pd.read_csv(trips_csv)
    needed = {"routeKey", "class", "travel_time"}
    if not needed.issubset(df.columns):
        raise ValueError(f"{trips_csv} missing required columns {needed}. Found {list(df.columns)}")

    ff = (
        df.groupby(["routeKey", "class"], as_index=False)["travel_time"]
          .min()
          .rename(columns={"travel_time": "TT_freeflow"})
    )
    ff.to_csv(out_csv, index=False)
    print(f"Wrote {out_csv}: {len(ff)} unique (routeKey,class) pairs")


# =========================
# Main: run both policies automatically
# =========================

def run():
    sumocfg_in = SUMO_CFG_IN
    if not Path(sumocfg_in).exists():
        raise FileNotFoundError(f"Could not find {sumocfg_in}. Run this from the folder containing it.")

    # Read original route files
    _, route_files = _read_sumocfg_paths(sumocfg_in)
    if not route_files:
        raise ValueError("No route files found in sumocfg.")

    # Stretch departures
    stretched_files = []
    latest_depart = 0.0
    for rf in route_files:
        tmp_rf, max_d = _stretch_route_file(rf, DEPART_STRETCH)
        stretched_files.append(tmp_rf)
        if max_d > latest_depart:
            latest_depart = max_d

    # Create a temp sumocfg pointing to stretched route files
    tmp_cfg = _write_temp_sumocfg(sumocfg_in, stretched_files)

    # Compute a safe end time (controllers may ignore this, but SUMO respects the cfg end if present;
    # we also patch controller MAX_SIM_TIME / --end where available is not guaranteed, so we prefer
    # to keep a generous buffer through the departure stretch itself.)
    sim_end = latest_depart + SIM_END_BUFFER
    print(f"Departure stretch={DEPART_STRETCH}; latest stretched depart≈{latest_depart:.1f}s; recommended end≈{sim_end:.1f}s")
    print(f"Temporary sumocfg: {tmp_cfg}")

    # ---- Run baseline MP (free-flow under MP policy) ----
    base_mod = _load_module_from_file(BASELINE_CONTROLLER_FILE, "mp_baseline_freeflow")
    # Patch controller globals for this free-flow run
    base_mod.SUMO_CFG = tmp_cfg
    base_mod.USE_GUI = True
    base_mod.LOG_CSV = TMP_TRIPS_MP

    # If controller has MAX_SIM_TIME or similar, patch it (best-effort)
    if hasattr(base_mod, "MAX_SIM_TIME"):
        base_mod.MAX_SIM_TIME = sim_end

    print("Running policy-conditioned free-flow: Baseline MP ...")
    base_mod.run()
    _triplog_to_freeflow_csv(TMP_TRIPS_MP, OUT_MP)

    # ---- Run LTFS (free-flow under LTFS policy) ----
    ltfs_mod = _load_module_from_file(LTFS_CONTROLLER_FILE, "mp_ltfs_freeflow")
    ltfs_mod.SUMO_CFG = tmp_cfg
    ltfs_mod.USE_GUI = True
    ltfs_mod.LOG_CSV = TMP_TRIPS_LTFS
    if hasattr(ltfs_mod, "MAX_SIM_TIME"):
        ltfs_mod.MAX_SIM_TIME = sim_end

    print("Running policy-conditioned free-flow: LTFS ...")
    ltfs_mod.run()
    _triplog_to_freeflow_csv(TMP_TRIPS_LTFS, OUT_LTFS)

    # Cleanup note: temp stretched route files and sumocfg are left on disk to allow debugging.
    # They are in your OS temp directory.
    print("Done. Generated:", OUT_MP, "and", OUT_LTFS)


if __name__ == "__main__":
    run()
