"""Evidence for fix-loop iteration 2: which face a wall snaps to vs which faces exist behind it.

    python bench/fix_loop/evidence_faces.py <capture> <result.json> <room_id>
Prints, for each wall of the room, the strongest vertical-surface layers within 0.8 m outward of the
fitted face (offset, point count, height range). A strong layer far outward with floor-to-high extent
behind a weak fitted face means the fitted face is furniture, not the wall.
"""
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
from roomscan.geometry.frame import PlanFrame  # noqa: E402
from roomscan.tiers.lidar import build_scene  # noqa: E402

cap, res_p, room_id = sys.argv[1:4]
sc = build_scene(cap, drift_correction=True, log=lambda *a: None)
F = PlanFrame.fit(sc.xyz, sc.normals)
q, nq = F.to_plan(sc.xyz), F.normals_to_plan(sc.normals)
res = json.load(open(res_p))
room = next(r for r in res["rooms"] if r["id"] == room_id)
cen = np.array(room["polygon"]).mean(0)
for w in room["walls"]:
    s, e = np.array(w["start"]), np.array(w["end"])
    o = "H" if abs(e[1] - s[1]) < abs(e[0] - s[0]) else "V"
    ax_n, ax_t = (1, 0) if o == "H" else (0, 1)
    c = s[ax_n]
    lo, hi = sorted([s[ax_t], e[ax_t]])
    if hi - lo < 0.5:
        continue
    inward = 1 if cen[ax_n] > c else -1
    sel = (np.abs(nq[:, ax_n]) > 0.8) & (q[:, ax_t] > lo + 0.1) & (q[:, ax_t] < hi - 0.1) & (np.abs(q[:, ax_n] - c) < 0.8)
    x = (q[sel, ax_n] - c) * (-inward)
    h = q[sel, 2]
    hist, ed = np.histogram(x, bins=np.arange(-0.3, 0.8, 0.02))
    print(f"{w['id']} length {w['length']['value']:.3f}")
    for i in np.argsort(hist)[-4:][::-1]:
        m = (x >= ed[i]) & (x < ed[i + 1])
        print(f"   layer {ed[i] * 100:+5.0f} cm outward: n={hist[i]:5d}, height {np.percentile(h[m], 5):.2f}-{np.percentile(h[m], 95):.2f} m")
