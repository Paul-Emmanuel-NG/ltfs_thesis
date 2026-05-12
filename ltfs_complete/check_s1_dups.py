import hashlib
for sc in ["S1"]:
    for s in [1,2,3]:
        for v in ["A5","A6","A7","A8"]:
            f = f"results/{v}_{sc}_seed{s}_trips.csv"
            try:
                with open(f, "rb") as fp:
                    data = fp.read()
                print(f"{v}_{sc}_seed{s}: {len(data)} bytes, md5={hashlib.md5(data).hexdigest()[:12]}")
            except FileNotFoundError:
                print(f"{v}_{sc}_seed{s}: MISSING")
