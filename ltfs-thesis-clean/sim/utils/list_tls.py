import traci
from sumolib import checkBinary

SUMO_CFG = r"C:\Users\YOURNAME\SUMO\2025-12-02-xx-xx-xx\osm.sumocfg"

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
