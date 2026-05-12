import hashlib

for v in ["A5","A6","A7","A8"]:
    f = f"results/{v}_S2_seed1_trips.csv"
    with open(f, "rb") as fp:
        data = fp.read()
    print(f"{v}: {len(data)} bytes, md5={hashlib.md5(data).hexdigest()[:12]}")
