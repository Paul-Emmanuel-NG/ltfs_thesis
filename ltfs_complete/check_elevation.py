import sumolib
n = sumolib.net.readNet("yanan_elevated.net.xml")
z_values = []
for e in n.getEdges():
    if e.getID().startswith(":"): continue
    shape = e.getShape3D()
    if shape:
        for x, y, z in shape:
            z_values.append(z)

if not z_values:
    print("No 3D shapes found in network. Network is FLAT.")
else:
    import statistics
    print(f"z range: {min(z_values):.1f} to {max(z_values):.1f}")
    print(f"unique z levels (rounded): {sorted(set(round(z) for z in z_values))[:20]}")
    print(f"n edges with non-zero z: {sum(1 for z in z_values if abs(z) > 0.1)} / {len(z_values)} points")
