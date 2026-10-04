"""Score damage output against digitally staged ground truth.

    python bench/eval_damage.py out/<run>/result.json data/staged/<capture>/staged_truth.json
"""
from __future__ import annotations

import json
import sys

import numpy as np


def plan_of_world(xyz, frame):
    t = np.radians(frame["yaw_deg"])
    c, s = np.cos(t), np.sin(t)
    R = np.array([[c, s], [-s, c]])
    uv = R @ np.array([xyz[0], xyz[2]])
    return np.array([uv[0], uv[1], xyz[1] - frame["floor_world_y"]])


def evaluate(result, truth, tol=0.35):
    fr = result["plan_frame"]
    dets = []
    for r in result["rooms"]:
        for d in r["damage"]:
            hc = 0.5 * (d["bbox"].get("h_min_m", 0) + d["bbox"].get("h_max_m", d["bbox"].get("h_min_m", 0)))
            dets.append(dict(d, p=np.array([d["plan_xy"][0], d["plan_xy"][1], hc])))
    rows, used = [], set()
    for g in truth["truth"]:
        gp = plan_of_world(g["centre_world"], fr)
        best = None
        for j, d in enumerate(dets):
            if d["class"] != g["cls"] or j in used:
                continue
            dist = float(np.linalg.norm(d["p"] - gp))
            if dist < tol and (best is None or dist < best[1]):
                best = (j, dist)
        row = dict(id=g["id"], cls=g["cls"], detected=best is not None)
        if best:
            used.add(best[0])
            d = dets[best[0]]
            row["localisation_err_m"] = round(best[1], 3)
            if g["cls"] == "crack":
                row.update(truth_length_m=g["length_m"], est_length_m=d.get("length_m"),
                           rel_err=round(abs(d.get("length_m", 0) - g["length_m"]) / g["length_m"], 3))
            else:
                a = d["area"]
                row.update(truth_area_m2=round(g["area_m2"], 4), est_area_m2=a["value"], ci95=a["ci95"],
                           rel_err=round(abs(a["value"] - g["area_m2"]) / g["area_m2"], 3),
                           in_ci=a["ci95"][0] <= g["area_m2"] <= a["ci95"][1])
            row["concealed_rules"] = sorted({f["rule_id"] for r in result["rooms"] for f in r["concealed_damage_flags"]
                                             if f["damage_id"] == d["id"]})
        rows.append(row)
    fp = [d for j, d in enumerate(dets) if j not in used]
    return dict(items=rows, recall=sum(r["detected"] for r in rows) / max(len(rows), 1),
                false_positives=len(fp), detections=len(dets))


if __name__ == "__main__":
    res = json.load(open(sys.argv[1]))
    tr = json.load(open(sys.argv[2]))
    print(json.dumps(evaluate(res, tr), indent=1))
