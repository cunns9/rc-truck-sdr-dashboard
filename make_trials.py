"""Make the shuffled trial order for docs/TRIAL_PLAN.md -> data/trial_order.csv.

60 person trials (3 distances x 2 lighting x 2 person types x 5 repeats) and 20 object
trials (chair, box, bag, cup x 5, each at a random distance and lighting), shuffled
together. The seed is fixed so the list can be rebuilt exactly.

    python3 make_trials.py            # writes data/trial_order.csv (refuses to overwrite)
    python3 make_trials.py --force    # overwrite
"""
import csv
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "data", "trial_order.csv")
SEED = 20261113
DISTANCES = (1, 3, 5)
LIGHTING = ("bright", "dim")
PEOPLE = ("person_standing", "person_walking")
OBJECTS = ("chair", "box", "bag", "cup")
REPEATS = 5


def build():
    rng = random.Random(SEED)
    rows = [(d, l, p) for d in DISTANCES for l in LIGHTING for p in PEOPLE for _ in range(REPEATS)]
    for obj in OBJECTS:
        # Each object visits every distance and both lightings at least once.
        dists = list(DISTANCES) + [rng.choice(DISTANCES) for _ in range(REPEATS - len(DISTANCES))]
        lights = list(LIGHTING) * (REPEATS // 2) + [rng.choice(LIGHTING)] * (REPEATS % 2)
        rng.shuffle(dists)
        rng.shuffle(lights)
        rows += [(d, l, obj) for d, l in zip(dists, lights)]
    rng.shuffle(rows)
    return [{"trial": i + 1, "distance_ft": d, "lighting": l, "target": t} for i, (d, l, t) in enumerate(rows)]


def main():
    if os.path.exists(OUT) and "--force" not in sys.argv:
        sys.exit(f"{OUT} already exists (use --force to overwrite)")
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    rows = build()
    with open(OUT, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    people = sum(r["target"].startswith("person") for r in rows)
    print(f"wrote {len(rows)} trials ({people} person, {len(rows) - people} object) to {OUT}")


if __name__ == "__main__":
    main()
