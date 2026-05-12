import sumolib
n = sumolib.net.readNet("yanan_elevated.net.xml")

import statistics
discrepancies = []
for e in n.getEdges():
    if e.getID().startswith(":"): continue
    declared_length = e.getLength()
    shape = e.getShape3D()
    if not shape or len(shape) < 2: continue
    
    # Compute 2D length from x,y
    twod_len = 0
    for i in range(len(shape)-1):
        x1,y1,_ = shape[i]
        x2,y2,_ = shape[i+1]
        twod_len += ((x2-x1)**2 + (y2-y1)**2) ** 0.5
    
    # Compute 3D length from x,y,z
    threed_len = 0
    for i in range(len(shape)-1):
        x1,y1,z1 = shape[i]
        x2,y2,z2 = shape[i+1]
        threed_len += ((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2) ** 0.5
    
    if abs(declared_length - twod_len) > 1.0:  # >1m difference
        discrepancies.append((e.getID(), declared_length, twod_len, threed_len))

print(f"Total edges checked: {len(list(n.getEdges()))}")
print(f"Edges where declared length differs from 2D shape length: {len(discrepancies)}")
if discrepancies[:5]:
    print("\nFirst 5 examples:")
    for eid, dl, td, t3 in discrepancies[:5]:
        print(f"  {eid}: declared={dl:.1f}m, 2D shape={td:.1f}m, 3D shape={t3:.1f}m")
