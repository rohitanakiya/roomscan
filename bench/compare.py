"""Register two results of the same property and compare them dimension by dimension.

Used for: LiDAR repeatability across captures, tier-vs-reference tables (video/photo vs LiDAR),
and the drift ablation. Plans are each Manhattan-aligned, so the rigid transform between them is
a k*90° rotation plus a translation (plus a small residual yaw, refined by ICP on wall samples).
"""
from __future__ import annotations

import json
import sys

import numpy as np
from scipy.signal import fftconvolve
from shapely.geometry import Polygon
from shapely.validation import make_valid

RES = 0.05


def _wall_samples(res):
    pts = []
    for r in res["rooms"]:
        for w in r["walls"]:
            a, b = np.array(w["start"]), np.array(w["end"])
            n = max(2, int(np.linalg.norm(b - a) / 0.05))
            pts.append(a + (b - a) * np.linspace(0, 1, n)[:, None])
    return np.concatenate(pts) if pts else np.zeros((0, 2))


def _raster(pts, lo, shape):
    img = np.zeros(shape, np.float32)
    ij = ((pts - lo) / RES).astype(int)
    ok = (ij[:, 0] >= 0) & (ij[:, 0] < shape[1]) & (ij[:, 1] >= 0) & (ij[:, 1] < shape[0])
    img[ij[ok, 1], ij[ok, 0]] = 1
    from scipy import ndimage as ndi

    return ndi.binary_dilation(img > 0, iterations=1).astype(np.float32)


def _rot(k):
    c, s = [(1, 0), (0, 1), (-1, 0), (0, -1)][k % 4]
    return np.array([[c, -s], [s, c]], float)


def register(res_a, res_b):
    """Return (R, t, score): maps plan coords of B into plan coords of A."""
    A, B = _wall_samples(res_a), _wall_samples(res_b)
    ca = A.mean(0)
    best = None
    for k in range(4):
        Bk = (B - B.mean(0)) @ _rot(k).T + ca
        lo = np.minimum(A.min(0), Bk.min(0)) - 2
        hi = np.maximum(A.max(0), Bk.max(0)) + 2
        shape = tuple(np.ceil((hi - lo) / RES).astype(int))[::-1]
        ia, ib = _raster(A, lo, shape), _raster(Bk, lo, shape)
        c = fftconvolve(ia, ib[::-1, ::-1], mode="same")
        iy, ix = np.unravel_index(np.argmax(c), c.shape)
        dy, dx = iy - shape[0] // 2, ix - shape[1] // 2
        if shape[0] % 2 == 0:
            dy += 1
        if shape[1] % 2 == 0:
            dx += 1
        score = c.max() / np.sqrt(ia.sum() * ib.sum())
        t = ca - _rot(k) @ B.mean(0) + np.array([dx, dy]) * RES
        if best is None or score > best[2]:
            best = (_rot(k), t, float(score))
    R, t, score = best
    # ICP refinement (2D point-to-point, trimmed)
    from scipy.spatial import cKDTree

    tree = cKDTree(A)
    for _ in range(30):
        Bt = B @ R.T + t
        d, idx = tree.query(Bt)
        m = d < np.percentile(d, 70)
        P, Q = Bt[m], A[idx[m]]
        mp, mq = P.mean(0), Q.mean(0)
        H = (P - mp).T @ (Q - mq)
        U, _, Vt = np.linalg.svd(H)
        dR = Vt.T @ U.T
        if np.linalg.det(dR) < 0:
            Vt[1] *= -1
            dR = Vt.T @ U.T
        R = dR @ R
        t = dR @ (t - mp) + mq
    return R, t, score


def match_rooms(res_a, res_b, R, t, min_iou=0.4):
    pairs = []
    for ra in res_a["rooms"]:
        pa = make_valid(Polygon(ra["polygon"]))
        best = None
        for rb in res_b["rooms"]:
            pb = make_valid(Polygon(np.array(rb["polygon"]) @ R.T + t))   # thin tiers can emit self-touching outlines
            inter = pa.intersection(pb).area
            iou = inter / max(pa.union(pb).area, 1e-9)
            if iou >= min_iou and (best is None or iou > best[1]):
                best = (rb, iou)
        if best:
            pairs.append((ra, best[0], best[1]))
    return pairs


def match_walls(ra, rb, R, t, tol=0.25):
    out = []
    for wa in ra["walls"]:
        a0, a1 = np.array(wa["start"]), np.array(wa["end"])
        da = (a1 - a0) / max(np.linalg.norm(a1 - a0), 1e-9)
        ma = 0.5 * (a0 + a1)
        best = None
        for wb in rb["walls"]:
            b0, b1 = np.array(wb["start"]) @ R.T + t, np.array(wb["end"]) @ R.T + t
            db = (b1 - b0) / max(np.linalg.norm(b1 - b0), 1e-9)
            if abs(abs(da @ db) - 1) > 0.02:
                continue
            mb = 0.5 * (b0 + b1)
            d = np.linalg.norm(ma - mb)
            if d < tol + 0.25 * np.linalg.norm(a1 - a0) and (best is None or d < best[1]):
                best = (wb, d)
        if best:
            out.append((wa, best[0]))
    return out


def match_openings(ra, rb, R, t, tol=0.3):
    def centres(room, RR=np.eye(2), tt=np.zeros(2)):
        walls = {w["id"]: w for w in room["walls"]}
        res = []
        for o in room["openings"]:
            w = walls[o["wall_id"]]
            a, b = np.array(w["start"]), np.array(w["end"])
            d = (b - a) / max(np.linalg.norm(b - a), 1e-9)
            c = a + d * (o["offset_from_wall_start_m"] + o["width"]["value"] / 2)
            res.append((o, c @ RR.T + tt))
        return res
    A, B = centres(ra), centres(rb, R, t)
    out, used = [], set()
    for oa, ca in A:
        best = None
        for j, (ob, cb) in enumerate(B):
            if j in used or (oa["type"] == "window") != (ob["type"] == "window"):
                continue
            d = np.linalg.norm(ca - cb)
            if d < tol and (best is None or d < best[1]):
                best = (j, d)
        if best:
            used.add(best[0])
            out.append((oa, B[best[0]][0]))
    unmatched_a = [oa for oa, _ in A if all(oa is not p[0] for p in out)]
    unmatched_b = [ob for j, (ob, _) in enumerate(B) if j not in used]
    return out, unmatched_a, unmatched_b


def compare(res_a, res_b, label_a="A", label_b="B"):
    R, t, score = register(res_a, res_b)
    rows = []
    pairs = match_rooms(res_a, res_b, R, t)
    for ra, rb, iou in pairs:
        rows.append(dict(kind="floor_area", room_a=ra["id"], room_b=rb["id"], a=ra["floor_area"], b=rb["floor_area"]))
        if ra["ceiling_height"]["observed"] and rb["ceiling_height"]["observed"]:
            rows.append(dict(kind="ceiling_height", room_a=ra["id"], room_b=rb["id"], a=ra["ceiling_height"],
                             b=rb["ceiling_height"]))
        for wa, wb in match_walls(ra, rb, R, t):
            if wa["length"]["value"] < 0.5:
                continue
            rows.append(dict(kind="wall_length", room_a=ra["id"], room_b=rb["id"], id_a=wa["id"], id_b=wb["id"],
                             a=wa["length"], b=wb["length"], observed=wa["face_observed"] and wb["face_observed"]))
        om, ua, ub = match_openings(ra, rb, R, t)
        for oa, ob in om:
            rows.append(dict(kind="opening_width", room_a=ra["id"], room_b=rb["id"], id_a=oa["id"], id_b=ob["id"],
                             a=oa["width"], b=ob["width"]))
        for oa in ua:
            rows.append(dict(kind="opening_missed_in_b", room_a=ra["id"], id_a=oa["id"], a=oa["width"]))
        for ob in ub:
            rows.append(dict(kind="opening_extra_in_b", room_b=rb["id"], id_b=ob["id"], b=ob["width"]))
    for r in rows:
        if "a" in r and "b" in r:
            r["diff"] = r["b"]["value"] - r["a"]["value"]
            r["rel"] = r["diff"] / max(abs(r["a"]["value"]), 1e-9)
            # is A's value inside B's interval (calibration of B against reference A)?
            r["a_in_b_ci"] = r["b"]["ci95"][0] <= r["a"]["value"] <= r["b"]["ci95"][1]
    return dict(registration=dict(R=R.round(4).tolist(), t=t.round(3).tolist(), score=round(score, 3),
                                  yaw_deg=round(float(np.degrees(np.arctan2(R[1, 0], R[0, 0]))), 2)),
                room_pairs=[(ra["id"], rb["id"], round(iou, 3)) for ra, rb, iou in pairs], rows=rows,
                labels=[label_a, label_b])


if __name__ == "__main__":
    a, b = json.load(open(sys.argv[1])), json.load(open(sys.argv[2]))
    out = compare(a, b)
    print(json.dumps(out["registration"]), out["room_pairs"])
    for r in out["rows"]:
        if "diff" in r:
            print(f"{r['kind']:15s} {r.get('id_a', r['room_a']):10s} a={r['a']['value']:.3f} b={r['b']['value']:.3f} "
                  f"diff={100 * r['diff']:+.1f}cm rel={100 * r['rel']:+.2f}%")
        else:
            print(r["kind"], r.get("id_a", r.get("id_b")))
