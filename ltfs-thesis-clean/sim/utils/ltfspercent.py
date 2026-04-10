import xml.etree.ElementTree as ET
import statistics
from collections import defaultdict

NET = "yanan_elevated.net.xml"
ROUTES = [
    "osm.passenger.rou.xml",
    "osm.bus.rou.xml",
    "osm.truck.rou.xml",
    "osm.motorcycle.rou.xml",
    "osm.bicycle.rou.xml",
]

def parse_lane_shape_z(shape_str):
    zs=[]
    if not shape_str: return zs
    for pt in shape_str.strip().split():
        parts=pt.split(",")
        if len(parts)>=3:
            try: zs.append(float(parts[2]))
            except: pass
    return zs

def extract_edge_info(net_file):
    tree=ET.parse(net_file); root=tree.getroot()
    edge_z_values={}
    edge_speed={}
    edge_ends={}
    for edge in root.findall("edge"):
        eid=edge.get("id","")
        if not eid or eid.startswith(":"):
            continue
        fr=edge.get("from"); to=edge.get("to")
        if fr and to:
            edge_ends[eid]=(fr,to)
        max_spd=edge_speed.get(eid,0.0)
        z_list=edge_z_values.get(eid,[])
        for lane in edge.findall("lane"):
            spd=lane.get("speed")
            if spd:
                try: max_spd=max(max_spd,float(spd))
                except: pass
            shape=lane.get("shape")
            if shape:
                zs=parse_lane_shape_z(shape)
                if zs: z_list.extend(zs)
        edge_speed[eid]=max_spd
        edge_z_values[eid]=z_list
    edge_z={eid:(float(statistics.median(zs)) if zs else 0.0) for eid,zs in edge_z_values.items()}
    return edge_ends, edge_speed, edge_z

def build_express_edges(net_file, min_speed=16.7, min_z=6.0, keep_components=2):
    edge_ends, edge_speed, edge_z = extract_edge_info(net_file)

    node_out=defaultdict(set)
    node_in=defaultdict(set)
    for eid,(fr,to) in edge_ends.items():
        node_out[fr].add(eid)
        node_in[to].add(eid)

    def neighbors(eid):
        fr,to=edge_ends.get(eid,(None,None))
        if fr is None: return set()
        nbr=set(node_out.get(to,set()))
        nbr |= set(node_in.get(fr,set()))
        nbr.discard(eid)
        return nbr

    candidates={eid for eid,z in edge_z.items() if edge_speed.get(eid,0.0)>=min_speed and z>=min_z}
    if not candidates:
        candidates={eid for eid,spd in edge_speed.items() if spd>=min_speed}

    visited=set(); comps=[]
    for eid in candidates:
        if eid in visited: continue
        stack=[eid]; visited.add(eid); comp=set()
        while stack:
            cur=stack.pop(); comp.add(cur)
            for nb in neighbors(cur):
                if nb in candidates and nb not in visited:
                    visited.add(nb); stack.append(nb)
        comps.append(comp)

    comps.sort(key=len, reverse=True)
    kept=comps[:max(1,int(keep_components))]
    return set().union(*kept) if kept else set()

EXP = build_express_edges(NET)

for rf in ROUTES:
    root = ET.parse(rf).getroot()
    total = 0
    touch = 0
    for v in root.findall("vehicle"):
        r = v.find("route")
        if r is None:
            continue
        edges = [e for e in r.get("edges","").split() if e and not e.startswith(":")]
        total += 1
        if any(e in EXP for e in edges):
            touch += 1
    print(rf, "routes:", total, "touch_express:", touch, "share:", (touch/total if total else 0.0))
