import csv
with open('results/A0_S1_seed1_trips.csv') as f:
    rows = list(csv.DictReader(f))
same = sum(1 for x in rows if x['routeKey'].split('->')[0] == x['routeKey'].split('->')[1])
diff = len(rows) - same
print('total rows:', len(rows))
print('single-edge routes:', same)
print('multi-edge routes:', diff)
tts = [float(x['travel_time']) for x in rows]
print('mean TT:', round(sum(tts)/len(tts), 1), 'max:', max(tts), 'min:', min(tts))
for x in rows:
    o, d = x['routeKey'].split('->')
    if o != d:
        print('example multi-edge:', x['vehID'], x['routeKey'], 'tt=' + x['travel_time'], 'dist=' + x['distance_m'])
        break
