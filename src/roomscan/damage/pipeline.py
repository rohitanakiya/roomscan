"""Damage: 2D candidates -> 3D lift onto fitted surfaces -> multi-view vote -> regions + rules + scope.

Multi-view voting is the specificity mechanism: a stain must be seen on the *same patch of the
same surface* from >= 2 viewpoints and in >= 40% of the views that saw that patch. Shadows,
specular highlights and screen glare move with the viewpoint; real damage does not.
"""
from __future__ import annotations

import subprocess
from pathlib import Path

import cv2
import numpy as np
from matplotlib.path import Path as MplPath
from scipy import ndimage as ndi

from ..measure import combine
from .detect import detect_ortho
from .rules import concealed_flags, scope_items

RES = 0.01          # surface orthomosaic resolution (m)
MIN_VIEWS = 3
MAX_VIEWS_PER_SURFACE = 15
MIN_RATIO = 0.5
MIN_AREA = {"water_stain": 0.010, "mould": 0.010, "crack": 0.0020}
MIN_CRACK_LEN = 0.15
# Bands excluded from detection: skirting / floor junction, cornice / ceiling junction, wall ends
# (corner shadows), ceiling perimeter. Floors are not assessed (tile grout and reflections were
# the dominant false positives on the sample captures).
SKIRT, CORNICE, WALL_END, CEIL_EDGE = 0.15, 0.10, 0.10, 0.15


def _surfaces(prop):
    S = []
    for r in prop.rooms:
        H = r.ceiling_height.value
        for w in r.walls:
            ax_t = 0 if w["orient"] == "H" else 1
            a, b = w["start"][ax_t], w["end"][ax_t]
            S.append(dict(id=f"S-{r.id}-W{w['index']}", ref=f"{r.id}-W{w['index']}", kind="wall", room=r,
                          orient=w["orient"], coord=w["coord"], t0=a, t1=b, H=H,
                          shape=(int(np.ceil(H / RES)) + 1, int(np.ceil(abs(b - a) / RES)) + 1)))
        P = r.polygon
        lo, hi = P.min(0), P.max(0)
        from shapely.geometry import Polygon

        inner = Polygon(P).buffer(-CEIL_EDGE)
        inner_path = MplPath(np.array(inner.exterior.coords)) if (not inner.is_empty and inner.geom_type == "Polygon") \
            else None
        for kind, h in (("floor", r.floor_h), ("ceiling", r.floor_h + H)):
            S.append(dict(id=f"S-{r.id}-{kind}", ref=r.id, kind=kind, room=r, h=h, lo=lo, path=MplPath(P),
                          inner=inner_path,
                          shape=(int(np.ceil((hi[1] - lo[1]) / RES)) + 1, int(np.ceil((hi[0] - lo[0]) / RES)) + 1)))
    return S


def _assign(q, S, tol=0.04):
    """q: (N,3) plan points -> (surface index or -1, row, col)."""
    sid = np.full(len(q), -1)
    rc = np.zeros((len(q), 2), int)
    best = np.full(len(q), np.inf)
    for k, s in enumerate(S):
        if s["kind"] == "wall":
            ax_n, ax_t = (1, 0) if s["orient"] == "H" else (0, 1)
            d = np.abs(q[:, ax_n] - s["coord"])
            t = q[:, ax_t]
            lo, hi = min(s["t0"], s["t1"]), max(s["t0"], s["t1"])
            m = (d < tol) & (t > lo) & (t < hi) & (q[:, 2] > -0.02) & (q[:, 2] < s["H"] + 0.02) & (d < best)
            if m.any():
                sid[m], best[m] = k, d[m]
                along = np.abs(t[m] - s["t0"])
                rc[m] = np.c_[(q[m, 2] / RES).astype(int), (along / RES).astype(int)]
        else:
            d = np.abs(q[:, 2] - s["h"])
            m = (d < tol) & (d < best)
            if m.any():
                idx = np.nonzero(m)[0]
                inside = s["path"].contains_points(q[idx, :2])
                idx = idx[inside]
                sid[idx], best[idx] = k, d[idx]
                rc[idx] = np.c_[((q[idx, 1] - s["lo"][1]) / RES).astype(int), ((q[idx, 0] - s["lo"][0]) / RES).astype(int)]
    return sid, rc


def _surface_normal_world(s, F):
    if s["kind"] != "wall":
        return np.array([0.0, 1.0, 0.0])
    n_plan = np.array([0.0, 1.0]) if s["orient"] == "H" else np.array([1.0, 0.0])
    nx, nz = n_plan @ F.R  # plan -> world (x,z)
    return np.array([nx, 0.0, nz])


def lidar_views(scene, work: Path, max_views=60, width=960):
    from ..io.stray import estimate_rgb_offset, load_stray

    cap = load_stray(scene.meta["capture"])
    off = estimate_rgb_offset(cap)  # video frame n <-> depth/pose row n + off
    kf = np.asarray(scene.meta["kf"])
    poses = scene.meta["poses"]
    sel = kf[np.linspace(0, len(kf) - 1, min(max_views, len(kf))).astype(int)]
    work.mkdir(parents=True, exist_ok=True)
    need = [i for i in sel if not (work / f"{int(cap.frame_ids[i]):06d}.jpg").exists()]
    if need:
        expr = "+".join(f"eq(n\\,{int(cap.frame_ids[i]) - off})" for i in need)
        tmp = work / "_tmp"
        tmp.mkdir(exist_ok=True)
        subprocess.check_call(["ffmpeg", "-v", "error", "-i", str(Path(scene.meta["capture"]) / "rgb.mp4"),
                               "-vf", f"select='{expr}',scale={width}:-2", "-vsync", "0", "-q:v", "2",
                               str(tmp / "%05d.jpg")])
        outs = sorted(tmp.glob("*.jpg"))
        for i, f in zip(sorted(need, key=lambda i: cap.frame_ids[i]), outs):
            f.rename(work / f"{int(cap.frame_ids[i]):06d}.jpg")
    s = width / 1920.0
    K = cap.K_rgb.copy()
    K[:2] *= s
    views = []
    for i in sel:
        p = work / f"{int(cap.frame_ids[i]):06d}.jpg"
        if not p.exists():
            continue
        fid = int(cap.frame_ids[i])

        def depth(shape, fid=fid):
            d = cap.load_depth(fid, min_conf=1)
            return cv2.resize(d, (shape[1], shape[0]), interpolation=cv2.INTER_LINEAR)
        views.append(dict(img=str(p), K=K, T=poses[i], depth=depth))
    return views


def run_damage(scene, prop, out_dir: Path, log=print, views=None, debug_dir=None):
    out_dir = Path(out_dir)
    if debug_dir is None and __import__("os").environ.get("ROOMSCAN_DEBUG"):
        debug_dir = out_dir / "ortho"
    if debug_dir is not None:
        Path(debug_dir).mkdir(parents=True, exist_ok=True)
    if views is None:
        if scene.tier == "lidar":
            views = lidar_views(scene, out_dir / "frames")
        else:
            views = scene.meta.get("damage_views", [])
    S = _surfaces(prop)
    ortho, seen_views = {}, {}
    F = prop.frame
    for v in views:
        img = cv2.imread(v["img"])
        if img is None:
            continue
        h, w = img.shape[:2]
        d = v["depth"]((h, w))
        K, T = v["K"], v["T"]
        vv, uu = np.mgrid[0:h, 0:w]
        z = d
        valid = (z > 0.2) & (z < 4.5)
        X = np.stack([(uu - K[0, 2]) / K[0, 0] * z, (vv - K[1, 2]) / K[1, 1] * z, z], -1)
        Xw = X.reshape(-1, 3) @ T[:3, :3].T + T[:3, 3]
        q = F.to_plan(Xw)
        sid, rc = _assign(q, S)
        sid = sid.reshape(h, w)
        sid[~valid] = -1
        rc = rc.reshape(h, w, 2)
        surf_mask = sid >= 0
        det_ok = np.zeros((h, w), bool)
        for k in np.unique(sid[surf_mask]):
            sk = S[k]
            m = sid == k
            r, c = rc[m, 0] * RES, rc[m, 1] * RES
            if sk["kind"] == "wall":
                L = abs(sk["t1"] - sk["t0"])
                ok = (r > SKIRT) & (r < sk["H"] - CORNICE) & (c > WALL_END) & (c < L - WALL_END)
            elif sk["kind"] == "ceiling":
                ok = sk["inner"].contains_points(np.c_[sk["lo"][0] + c, sk["lo"][1] + r]) if sk["inner"] else \
                    np.zeros(len(r), bool)
            else:
                ok = np.zeros(len(r), bool)
            det_ok[m] = ok
        # per-view orthographic sample of each surface (frontal, near views only)
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        cam = T[:3, 3]
        for k in np.unique(sid[det_ok]):
            m = det_ok & (sid == k)
            r, c = rc[m, 0], rc[m, 1]
            ok = (r >= 0) & (r < S[k]["shape"][0]) & (c >= 0) & (c < S[k]["shape"][1])
            if ok.sum() < 200:
                continue
            P = Xw.reshape(h, w, 3)[m][ok]
            ray = P - cam
            dist = np.linalg.norm(ray, axis=1)
            nrm = _surface_normal_world(S[k], F)
            cosi = np.abs(ray @ nrm) / np.maximum(dist, 1e-6)
            good = (cosi > 0.5) & (dist < 3.0)
            if good.sum() < 200:
                continue
            im = np.zeros(S[k]["shape"] + (3,), np.uint8)
            msk = np.zeros(S[k]["shape"], bool)
            im[r[ok][good], c[ok][good]] = lab[m][ok][good]
            msk[r[ok][good], c[ok][good]] = True
            msk = ndi.binary_closing(msk, iterations=1)
            score = float(good.sum() * cosi[good].mean())
            ortho.setdefault(k, []).append((score, im, msk))
    hits = {}
    for k, samples in ortho.items():
        samples = sorted(samples, key=lambda t: -t[0])[:MAX_VIEWS_PER_SURFACE]
        stack = np.stack([t[1] for t in samples]).astype(np.float32)
        msks = np.stack([t[2] for t in samples])
        stack[~msks] = np.nan
        nviews = msks.sum(0)
        valid = nviews >= MIN_VIEWS
        if valid.sum() < 100:
            continue
        with np.errstate(all="ignore"):
            import warnings

            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                mean = np.nanmedian(stack, axis=0)   # median: robust to ghosts / misregistration
        mean = np.nan_to_num(mean)
        if debug_dir is not None:
            vis = cv2.cvtColor(np.clip(mean, 0, 255).astype(np.uint8), cv2.COLOR_LAB2BGR)
            vis[~valid] = (255, 0, 255)
            cv2.imwrite(str(Path(debug_dir) / f"{S[k]['id']}.png"), vis[::-1])
        for cand in detect_ortho(mean, valid, RES):
            hits[(k, cand["cls"])] = hits.get((k, cand["cls"]), np.zeros(S[k]["shape"], bool)) | cand["mask"]
            seen_views[(k, cand["cls"])] = nviews
    shared = {a["via"][0]["room"] for a in prop.adjacency}  # rooms with neighbours (coarse)
    shared_walls = set()
    regions_by_room = {}
    n_reg = 0
    for (k, cls), good in hits.items():
        s = S[k]
        H = seen_views[(k, cls)]
        lab, n = ndi.label(good)
        for j in range(1, n + 1):
            m = lab == j
            area = m.sum() * RES * RES
            if area < MIN_AREA[cls]:
                continue
            rows, cols = np.nonzero(m)
            n_reg += 1
            rid = f"D{n_reg}"
            views_support = int(H[m].max())
            bbox = dict(width_m=round((np.ptp(cols) + 1) * RES, 3), height_m=round((np.ptp(rows) + 1) * RES, 3))
            if s["kind"] == "wall":
                bbox.update(along_wall_from_start_m=round(cols.min() * RES, 3), h_min_m=round(rows.min() * RES, 3),
                            h_max_m=round((rows.max() + 1) * RES, 3))
                ax_t = 0 if s["orient"] == "H" else 1
                sign = 1 if s["t1"] >= s["t0"] else -1
                t = s["t0"] + sign * (cols.mean() + 0.5) * RES
                xy = [t, s["coord"]] if s["orient"] == "H" else [s["coord"], t]
            else:
                bbox.update(h_min_m=round(s["h"], 3))
                xy = [s["lo"][0] + (cols.mean() + 0.5) * RES, s["lo"][1] + (rows.mean() + 0.5) * RES]
            # area uncertainty: boundary cells half in / half out + LiDAR/pose registration
            perim = m.sum() - ndi.binary_erosion(m).sum()
            am = combine(area, unit="m2", boundary=0.5 * perim * RES * RES, registration=0.1 * area)
            reg = dict(id=rid, surface_id=s["id"], **{"class": cls}, area=am.to_json(4), bbox=bbox,
                       plan_xy=[round(float(xy[0]), 3), round(float(xy[1]), 3)], views=views_support,
                       confidence=round(float(min(1.0, 0.5 + 0.1 * views_support)), 2))
            if cls == "crack":
                reg["length_m"] = round(float(np.hypot(bbox["width_m"], bbox["height_m"])), 3)
                if reg["length_m"] < MIN_CRACK_LEN:
                    n_reg -= 1
                    continue
            surf = dict(id=s["id"], kind=s["kind"], ref=s["ref"])
            flags = concealed_flags(reg, surf, s["room"], shared_walls)
            if s["kind"] == "wall":
                surf_area = abs(s["t1"] - s["t0"]) * s["H"]
            else:
                surf_area = s["room"].floor_area.value
            items = scope_items(reg, surf, surf_area, flags)
            room = s["room"]
            room.meta.setdefault("damage", []).append(reg)
            room.meta.setdefault("concealed", []).extend(flags)
            room.meta.setdefault("scope", []).extend(items)
    log(f"  damage: {len(views)} views, {n_reg} confirmed regions")
    return n_reg
