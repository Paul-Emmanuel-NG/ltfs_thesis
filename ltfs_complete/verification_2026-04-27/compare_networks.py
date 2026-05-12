import sumolib
import statistics
import collections

OLD = r'C:\work\thesis\ltfs_complete\yanan_elevated.net.xml'
NEW = r'C:\work\thesis\verification_2026-04-27\yanan_regenerated.net.xml'

def analyze(path, label):
    n = sumolib.net.readNet(path)
    edges = [e for e in n.getEdges() if not e.getID().startswith(':')]
    junctions = [j for j in n.getNodes() if not j.getID().startswith(':')]
    tls = list(n.getTrafficLights())

    z_means = []
    for e in edges:
        shape = e.getShape3D()
        if shape:
            z_means.append(statistics.mean(z for x,y,z in shape))

    lengths = [e.getLength() for e in edges]
    lane_counts = collections.Counter(len(e.getLanes()) for e in edges)

    print('=== ' + label + ' ===')
    print('  Edges: ' + str(len(edges)))
    print('  Junctions: ' + str(len(junctions)))
    print('  Traffic lights: ' + str(len(tls)))
    print('  Edge lengths: min={:.1f}m, max={:.1f}m, mean={:.1f}m, total={:.1f}km'.format(
        min(lengths), max(lengths), statistics.mean(lengths), sum(lengths)/1000))
    if z_means:
        print('  Z range: {:.1f} to {:.1f}m'.format(min(z_means), max(z_means)))
        print('  Edges with non-zero z (|z|>0.5): ' + str(sum(1 for z in z_means if abs(z) > 0.5)))
        print('  Edges with elevated z (z>=5): ' + str(sum(1 for z in z_means if z >= 5)))
        print('  Edges with tunnel z (z<=-5): ' + str(sum(1 for z in z_means if z <= -5)))
    print('  Lane counts: ' + str(dict(sorted(lane_counts.items()))))
    return {'edges': len(edges), 'junctions': len(junctions), 'tls': len(tls)}

old = analyze(OLD, 'OLD (current working network)')
print('')
new = analyze(NEW, 'NEW (regenerated)')
print('')
print('=== Diff ===')
print('  Edges:     old={}, new={}, diff={:+}'.format(old['edges'], new['edges'], new['edges']-old['edges']))
print('  Junctions: old={}, new={}, diff={:+}'.format(old['junctions'], new['junctions'], new['junctions']-old['junctions']))
print('  TLs:       old={}, new={}, diff={:+}'.format(old['tls'], new['tls'], new['tls']-old['tls']))

old_n = sumolib.net.readNet(OLD)
new_n = sumolib.net.readNet(NEW)
old_eids = set(e.getID() for e in old_n.getEdges() if not e.getID().startswith(':'))
new_eids = set(e.getID() for e in new_n.getEdges() if not e.getID().startswith(':'))
common = old_eids & new_eids
only_old = old_eids - new_eids
only_new = new_eids - old_eids
print('')
print('  Edge ID overlap:')
print('    Common to both: {} ({:.1f}% of old)'.format(len(common), 100*len(common)/len(old_eids)))
print('    Only in OLD:    ' + str(len(only_old)))
print('    Only in NEW:    ' + str(len(only_new)))
