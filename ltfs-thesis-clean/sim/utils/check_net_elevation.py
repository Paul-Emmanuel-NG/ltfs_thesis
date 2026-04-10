import sys
import xml.etree.ElementTree as ET
from statistics import median
from collections import Counter

def parse_lane_shape_z(shape_str: str):
    zs = []
    if not shape_str:
        return zs
    for pt in shape_str.strip().split():
        parts = pt.split(",")
        if len(parts) >= 3:
            try:
                zs.append(float(parts[2]))
            except ValueError:
                pass
    return zs

def main(net_file: str):
    root = ET.parse(net_file).getroot()

    z_meds = []
    for edge in root.findall("edge"):
        eid = edge.get("id", "")
        if not eid or eid.startswith(":"):
            continue

        zs_all = []
        for lane in edge.findall("lane"):
            zs_all.extend(parse_lane_shape_z(lane.get("shape", "")))

        if zs_all:
            z_meds.append(float(median(zs_all)))

    rounded = [int(round(z)) for z in z_meds]
    c = Counter(rounded)

    print(f"Edges with z: {len(z_meds)}")
    print(f"Unique rounded z levels: {len(c)}")
    print("Top 20 most common rounded z levels (meters):")
    for z, n in c.most_common(20):
        print(f"  z={z:>4}  count={n}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python z_levels.py <net.xml>")
        sys.exit(1)
    main(sys.argv[1])
