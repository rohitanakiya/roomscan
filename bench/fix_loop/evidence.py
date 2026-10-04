"""Evidence for the fix-loop root cause: footprint vs room overlap, and wall-face offsets, between captures.

    python bench/fix_loop/evidence.py out/bench/lidar  > bench/fix_loop/evidence.txt
"""
import json
import sys
from pathlib import Path

import matplotlib
import numpy as np
from shapely.geometry import Polygon
from shapely.ops import unary_union

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from compare import match_rooms, register  # noqa: E402

root = Path(sys.argv[1] if len(sys.argv) > 1 else "out/bench/lidar")
png = sys.argv[2] if len(sys.argv) > 2 else None
pairs = [("floor_only", "with_ceiling"), ("floor_only", "single_room"), ("with_ceiling", "single_room")]


def faces(res, R=np.eye(2), t=np.zeros(2)):
    F = []
    for r in res["rooms"]:
        for w in r["walls"]:
            if not w["face_observed"]:
                continue
            s0, s1 = np.array(w["start"]) @ R.T + t, np.array(w["end"]) @ R.T + t
            o = "H" if abs(s1[1] - s0[1]) < abs(s1[0] - s0[0]) else "V"
            c = (s0[1] + s1[1]) / 2 if o == "H" else (s0[0] + s1[0]) / 2
            lo, hi = sorted([s0[0], s1[0]] if o == "H" else [s0[1], s1[1]])
            F.append((o, c, lo, hi))
    return F


for a, b in pairs:
    A = json.load(open(root / a / "result.json"))
    B = json.load(open(root / b / "result.json"))
    R, t, s = register(A, B)
    UA = unary_union([Polygon(r["polygon"]).buffer(0) for r in A["rooms"]])
    UB = unary_union([Polygon(np.array(r["polygon"]) @ R.T + t).buffer(0) for r in B["rooms"]])
    inter = UA.intersection(UB).area
    ious = [round(i, 2) for _, _, i in match_rooms(A, B, R, t, 0.1)]
    print(f"{a}~{b}: registration score {s:.3f}; footprint overlap/smaller {inter / min(UA.area, UB.area):.2f}; "
          f"room IoUs {ious}")
    FA, FB = faces(A), faces(B, R, t)
    d = []
    for o, c, lo, hi in FA:
        best = None
        for o2, c2, lo2, hi2 in FB:
            if o2 != o or min(hi, hi2) - max(lo, lo2) < 0.5 or abs(c - c2) > 0.10:
                continue
            if best is None or abs(c - c2) < abs(best):
                best = c2 - c
        if best is not None:
            d.append(abs(best))
    d = np.array(d)
    if len(d):
        print(f"   wall faces matched {len(d)}: median |offset| {100 * np.median(d):.1f} cm, "
              f"within 1 cm {100 * (d <= 0.01).mean():.0f}%, within 2 cm {100 * (d <= 0.02).mean():.0f}%")
    if png and (a, b) == pairs[0]:
        fig, ax = plt.subplots(figsize=(10, 10))
        for r in A["rooms"]:
            P = np.array(r["polygon"] + [r["polygon"][0]])
            ax.plot(P[:, 0], P[:, 1], "r-", lw=2)
            c = P[:-1].mean(0)
            ax.text(c[0], c[1], f"{a}:{r['id']} {r['floor_area']['value']:.1f}", color="r", fontsize=8)
        for r in B["rooms"]:
            P = np.array(r["polygon"] + [r["polygon"][0]]) @ R.T + t
            ax.plot(P[:, 0], P[:, 1], "g-", lw=2)
            c = P[:-1].mean(0)
            ax.text(c[0], c[1] - 0.3, f"{b}:{r['id']} {r['floor_area']['value']:.1f}", color="g", fontsize=8)
        ax.set_aspect("equal")
        ax.grid(True)
        fig.savefig(png, dpi=80)
