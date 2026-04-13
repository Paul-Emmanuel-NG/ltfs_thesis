import traci
from pathlib import Path
from sumolib import checkBinary

UTILS_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = UTILS_DIR.parents[1]
SUMO_CFG = str(PROJECT_ROOT / "sim" / "network" / "osm.sumocfg")

def main():
    traci.start([
        checkBinary("sumo-gui"),
        "-c", SUMO_CFG,
        "--start", "true",
        "--no-step-log", "true",
    ])
    tls_ids = traci.trafficlight.getIDList()
    print("Traffic light IDs in this network:")
    for tls_id in tls_ids:
        print("  ", tls_id)
    traci.close()

if __name__ == "__main__":
    main()
