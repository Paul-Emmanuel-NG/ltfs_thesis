"""
config.py — All LTFS paper parameters in one place.

Equation numbers refer to the LaTeX draft of Paper 1. Section numbers are
paper sections.

Nothing here is a magic number without a paper citation. If you change a
value, also update the relevant Ambiguity note in AUDIT.md.
"""

from __future__ import annotations
from pathlib import Path


# ============================================================================
# Paths (all relative to project root — no hardcoded absolute paths)
# ============================================================================

PROJECT_ROOT = Path(__file__).resolve().parent.parent
SUMO_CFG = PROJECT_ROOT / "osm.sumocfg"
NET_FILE = PROJECT_ROOT / "yanan_elevated.net.xml"
RESULTS_DIR = PROJECT_ROOT / "results"


# ============================================================================
# Simulation timing (Sec VIII-E)
# ============================================================================

STEP_LENGTH_S = 1.0          # Simulation step length
WARMUP_S = 900.0             # Sec VIII-E: warmup period
SIM_BODY_S = 5400.0          # Sec VIII-E: experiment duration
MAX_SIM_TIME_S = WARMUP_S + SIM_BODY_S


# ============================================================================
# TLS footprint (Yan'an interchange)
# ============================================================================

YANAN_ANCHOR_TLS = "cluster_479314640_850287516"
YANAN_RADIUS_M = 400.0


# ============================================================================
# Signal control — common
# ============================================================================

MIN_GREEN_S = 10.0           # Sec IV-B (signal constraints)
MAX_GREEN_S = 60.0           # Sec IV-B — prevents indefinite green under MP
INTERGREEN_S = 3.0           # Yellow / all-red clearance

# Actuated controller (Sec V-B)
ACTUATED_GAP_S = 3.0         # Gap threshold for green extension
ACTUATED_EXTEND_S = 2.0      # Extension granularity


# ============================================================================
# Vehicle class urgency u^class (Eq 2, Ambiguity A5)
# ============================================================================

U_MIN = 0.1                  # Eq 2: strict lower bound > 0

# SUMO vehicle-type ID -> urgency. Ambiguity A5: these are the existing
# convention, inherited from the prior code.
CLASS_URGENCY = {
    "bus_bus": 0.9,
    "truck_truck": 0.7,
    "veh_passenger": 0.4,
    "motorcycle_motorcycle": 0.35,
    "bike_bicycle": 0.2,
    "ped_pedestrian": 0.1,
}
CLASS_URGENCY_DEFAULT = 0.4  # For unknown vehicle types


# ============================================================================
# Route & wait urgency (Eq 3, 4)
# ============================================================================

L_REF_M = 3000.0             # Eq 3 reference distance (Ambiguity A8)
                             # NB: runner can override this with scenario-median
                             # remaining route length; this is the fallback.
W_REF_S = 60.0               # Eq 4 reference wait time (paper default)


# ============================================================================
# Urgency combiners (Eq 5, 6)
# ============================================================================

LAMBDA_GATING = 0.6          # Eq 5: weight on class urgency for gating
LAMBDA_SIGNAL = 0.5          # Eq 6: weight on class urgency for signals


# ============================================================================
# Gating rules (Eq 15, 25, 26)
# ============================================================================

THETA_S = 5.0                # TST threshold (seconds of time saving) - tuned for compact Yan'an network
THETA_U_S = 3.0              # UWA threshold (urgency-weighted) - scaled proportionally
KAPPA = 0.85                 # Occupancy cap for gate admission


# ============================================================================
# PWMP weight construction (Eq 10, Ambiguity A6)
# ============================================================================

PWMP_ALPHA = 0.5             # Gate-feed weight
PWMP_BETA = 0.7              # Discharge weight (largest — prevents spillback)
PWMP_ETA = 0.2               # Express-layer weight
PWMP_GAMMA = 0.6             # Upstream urgency weight
PWMP_W_MAX = 1.5             # Default w_max; sensitivity sweep uses {1.2, 1.5, 2.0}


# ============================================================================
# Express layer detection (Ambiguity A3)
# ============================================================================

EXPRESS_MIN_SPEED_MPS = 16.7   # 60 km/h
EXPRESS_MIN_Z = 6.0            # Elevated segments
EXPRESS_MAX_Z = -5.0           # Tunnel segments (negative elevation)
EXPRESS_KEEP_COMPONENTS = 4    # Keep top-N connected components (elevated + tunnel arms)


# ============================================================================
# Express layer occupancy (N4, Ambiguity A2)
# ============================================================================

VEHICLE_SPACING_M = 7.5      # Average vehicle length + headway


# ============================================================================
# Routing (Sec VI-B, VI-C)
# ============================================================================

# Eq 21: exponential smoothing for dynamic TT
DYN_TT_XI = 0.3              # Smoothing factor
DYN_TT_UPDATE_PERIOD_S = 30.0   # How often to recompute TTs and reroute

# Eq 22: Q-routing
Q_LEARNING_RATE = 0.2
Q_EPSILON = 0.05             # Exploration probability
Q_UPDATE_PERIOD_S = 30.0


# ============================================================================
# Gate evaluation
# ============================================================================

GATE_EVAL_PERIOD_S = 5.0     # How often to re-evaluate pending gate vehicles


# ============================================================================
# Objective weights (Eq 16, Sec IV-A, with sensitivity A6)
# ============================================================================

W_EFF = 0.6
W_REL = 0.3
W_FAIR = 0.1


# ============================================================================
# Free-flow calibration (Eq 28, Sec IX-A)
# ============================================================================

FREEFLOW_DEPART_STRETCH = 10.0   # Multiply all depart times by this
FREEFLOW_SIM_END_BUFFER_S = 36000.0


# ============================================================================
# Seeds
# ============================================================================

DEFAULT_SEEDS = [1, 2, 3]
