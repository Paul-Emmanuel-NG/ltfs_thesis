import csv, statistics, glob, os, re, math
from collections import defaultdict

def stats_with_ci(values):
    n = len(values)
    if n < 1: return float('nan'), float('nan')
    m = statistics.mean(values)
    if n < 2: return m, 0.0
    sd = statistics.stdev(values)
    return m, 1.96 * sd / math.sqrt(n)

def aggregate_by_class(folder, scenario):
    agg = defaultdict(list)
    for fp in sorted(glob.glob(os.path.join(folder, 'A?_' + scenario + '_seed*_trips.csv'))):
        m = re.match(r'(A\d)_' + scenario + r'_seed(\d+)_trips\.csv', os.path.basename(fp))
        if not m: continue
        v = m.group(1)
        with open(fp) as f:
            rows = list(csv.DictReader(f))
        if not rows: continue
        by_class = defaultdict(list)
        for r in rows:
            by_class[r['class']].append(float(r['travel_time']))
        rec = {c: statistics.mean(tts) for c, tts in by_class.items()}
        agg[v].append(rec)
    return agg

for label, folder, scen in [('Light S1', 'results', 'S1'),
                            ('Corridor-pen S1', 'results/corridor_pen', 'S1'),
                            ('Light S2', 'results', 'S2')]:
    agg = aggregate_by_class(folder, scen)
    print('')
    print('=== ' + label + ' ===')
    classes = ['bus_bus', 'truck_truck', 'veh_passenger']
    print('{:4} {:>5}  {:>18}  {:>18}  {:>18}'.format('Var', 'n', 'bus', 'truck', 'passenger'))
    print('-' * 75)
    for vid in sorted(agg.keys()):
        runs = agg[vid]
        if not runs: continue
        line = '{:4} {:>5}'.format(vid, len(runs))
        for c in classes:
            vals = [r[c] for r in runs if c in r]
            m, ci = stats_with_ci(vals)
            line += '  {:>8.1f}'.format(m) + '+/-' + '{:>5.1f}'.format(ci) + 's'
        print(line)
