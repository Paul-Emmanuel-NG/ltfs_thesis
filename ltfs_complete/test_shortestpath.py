import sumolib
n = sumolib.net.readNet("yanan_elevated.net.xml")
edges = list(n.getEdges())
print("sumolib version:", sumolib.version.gitDescribe() if hasattr(sumolib, "version") else "?")
print("testing getShortestPath signature...")
# Try default call
o = edges[0]
# find a destination a few hops away
for d in edges[1:100]:
    try:
        path, cost = n.getShortestPath(o, d)
        if path and cost and cost < 1e9:
            print("OK default:", o.getID(), "->", d.getID(), "cost=", cost, "hops=", len(path))
            break
    except Exception as e:
        print("default failed:", e)
        break
# Try with edgeWeightFunction (what my code uses)
try:
    def w(e): return 1.0
    path2, cost2 = n.getShortestPath(o, d, edgeWeightFunction=w)
    print("edgeWeightFunction OK:", cost2)
except TypeError as e:
    print("edgeWeightFunction TypeError:", e)
except Exception as e:
    print("edgeWeightFunction other error:", e)
# Try vClass-based
try:
    path3, cost3 = n.getShortestPath(o, d, vClass="passenger")
    print("vClass OK:", cost3)
except Exception as e:
    print("vClass error:", e)
