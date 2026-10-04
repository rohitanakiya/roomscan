"""Room segmentation by Manhattan cell decomposition (wall-line arrangement).

Why: the watershed segmentation depended on how far each capture's rays happened to reach, so
two captures of the same flat split rooms differently (floor-only vs with-ceiling registration
NCC 0.30, room areas off by 60%). Walls are what is stable across captures, so rooms are built
from walls:
  1. wall faces = Manhattan line segments fitted to vertical surface points
  2. every face's coordinate becomes an infinite line; the lines cut the plan into rectangles
  3. a rectangle is interior if the visibility raster says it was seen through
  4. neighbouring interior rectangles merge unless a wall segment covers their shared edge
  5. thin interior rectangles that sit *inside a wall* (jambs on both ends) are doorways:
     they connect rooms but belong to neither; their width is the opening width
Room polygons are unions of rectangles, so they are rectilinear and every edge already lies on
a fitted wall face.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class Segment:
    orient: str    # "H": constant v, runs along u ; "V": constant u, runs along v
    coord: float
    t0: float
    t1: float
    n: int


def _peaks_1d(x, bin_m=0.01, min_count=120, nms=0.04):
    if len(x) == 0:
        return []
    lo, hi = x.min() - 0.05, x.max() + 0.05
    h, e = np.histogram(x, bins=np.arange(lo, hi + bin_m, bin_m))
    hs = np.convolve(h, [1, 2, 3, 2, 1], mode="same") / 9 * 5
    cand = [i for i in range(1, len(hs) - 1) if hs[i] >= min_count / 3 and hs[i] >= hs[i - 1] and hs[i] >= hs[i + 1]]
    cand.sort(key=lambda i: -hs[i])
    kept = []
    for i in cand:
        c = 0.5 * (e[i] + e[i + 1])
        if all(abs(c - k) > nms for k in kept):
            kept.append(c)
    return sorted(kept)


def extract_segments(q, nq, min_len=0.30, gap=0.15, band=0.02, min_height_extent=0.6):
    segs = []
    vert = (np.abs(nq[:, 2]) < 0.3) & (q[:, 2] > 0.08) & (q[:, 2] < 3.4)
    for orient, ax_n, ax_t in (("H", 1, 0), ("V", 0, 1)):
        sel = vert & (np.abs(nq[:, ax_n]) > 0.9)
        P = q[sel]
        for c in _peaks_1d(P[:, ax_n]):
            m = np.abs(P[:, ax_n] - c) < band
            if m.sum() < 80:
                continue
            t, h = P[m, ax_t], P[m, 2]
            c_ref = float(np.median(P[m, ax_n]))
            b = 0.05
            lo = t.min()
            idx = ((t - lo) / b).astype(int)
            nb = idx.max() + 1
            cnt = np.bincount(idx, minlength=nb)
            hmax = np.full(nb, -9.0)
            hmin = np.full(nb, 9.0)
            np.maximum.at(hmax, idx, h)
            np.minimum.at(hmin, idx, h)
            occ = cnt >= 3
            # close small gaps
            runs, s = [], None
            last = -999
            for i in range(nb):
                if occ[i]:
                    if s is None or (i - last) * b > gap:
                        if s is not None:
                            runs.append((s, last))
                        s = i
                    last = i
            if s is not None:
                runs.append((s, last))
            for s0, s1 in runs:
                L = (s1 - s0 + 1) * b
                ext = hmax[s0:s1 + 1].max() - hmin[s0:s1 + 1].min()
                if L >= min_len and ext >= min_height_extent:
                    segs.append(Segment(orient, c_ref, lo + s0 * b, lo + (s1 + 1) * b, int(cnt[s0:s1 + 1].sum())))
    return segs


def _merge_coords(vals, tol=0.03):
    vals = sorted(vals)
    out = []
    for v in vals:
        if out and v - out[-1][-1] < tol:
            out[-1].append(v)
        else:
            out.append([v])
    return [float(np.mean(g)) for g in out]


def _coverage(segs, orient, coord, a, b, tol=0.03):
    """Fraction of [a,b] on line `coord` covered by wall segments of that orientation."""
    iv = [(max(s.t0, a), min(s.t1, b)) for s in segs if s.orient == orient and abs(s.coord - coord) < tol]
    iv = sorted([x for x in iv if x[1] > x[0]])
    tot, cur = 0.0, None
    for x0, x1 in iv:
        if cur is None or x0 > cur[1]:
            if cur:
                tot += cur[1] - cur[0]
            cur = [x0, x1]
        else:
            cur[1] = max(cur[1], x1)
    if cur:
        tot += cur[1] - cur[0]
    return tot / max(b - a, 1e-9)


def segment_cells(q, nq, rasters, free_thr=0.55, thin=0.30, block=0.5, min_room_area=1.0):
    G = rasters.grid
    segs = extract_segments(q, nq)
    U = _merge_coords([s.coord for s in segs if s.orient == "V"])
    V = _merge_coords([s.coord for s in segs if s.orient == "H"])
    lo = G.origin
    hi = G.origin + np.array([G.shape[1], G.shape[0]]) * G.res
    U = [lo[0]] + [u for u in U if lo[0] < u < hi[0]] + [hi[0]]
    V = [lo[1]] + [v for v in V if lo[1] < v < hi[1]] + [hi[1]]
    nu, nv = len(U) - 1, len(V) - 1
    # integral image of free raster
    F = rasters.free.astype(np.float64)
    I = np.pad(F.cumsum(0).cumsum(1), ((1, 0), (1, 0)))

    def frac(u0, u1, v0, v1):
        c0, c1 = int((u0 - lo[0]) / G.res), int((u1 - lo[0]) / G.res)
        r0, r1 = int((v0 - lo[1]) / G.res), int((v1 - lo[1]) / G.res)
        c1, r1 = max(c1, c0 + 1), max(r1, r0 + 1)
        s = I[r1, c1] - I[r0, c1] - I[r1, c0] + I[r0, c0]
        return s / ((r1 - r0) * (c1 - c0))

    interior = np.zeros((nv, nu), bool)
    for j in range(nv):
        for i in range(nu):
            interior[j, i] = frac(U[i], U[i + 1], V[j], V[j + 1]) > free_thr
    w = np.diff(U)[None, :].repeat(nv, 0)
    h = np.diff(V)[:, None].repeat(nu, 1)
    # blocked shared edges
    block_u = np.zeros((nv, nu + 1), bool)   # vertical edge at U[i] between cell i-1 and i
    block_v = np.zeros((nv + 1, nu), bool)   # horizontal edge at V[j]
    for j in range(nv):
        for i in range(1, nu):
            block_u[j, i] = _coverage(segs, "V", U[i], V[j], V[j + 1]) > block
    for j in range(1, nv):
        for i in range(nu):
            block_v[j, i] = _coverage(segs, "H", V[j], U[i], U[i + 1]) > block

    # doorway cells: thin interior cell with non-interior cells at both ends of its long axis
    connector = np.zeros_like(interior)
    for j in range(nv):
        for i in range(nu):
            if not interior[j, i]:
                continue
            if h[j, i] < thin and w[j, i] >= h[j, i]:      # thin in v -> lies in an H wall; ends along u
                ends = [(j, i - 1), (j, i + 1)]
            elif w[j, i] < thin and h[j, i] > w[j, i]:     # thin in u -> lies in a V wall; ends along v
                ends = [(j - 1, i), (j + 1, i)]
            else:
                continue
            ok = all(not (0 <= jj < nv and 0 <= ii < nu) or not interior[jj, ii] for jj, ii in ends)
            connector[j, i] = ok

    parent = {}

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    for j in range(nv):
        for i in range(nu):
            if interior[j, i] and not connector[j, i]:
                parent[(j, i)] = (j, i)
    for (j, i) in list(parent):
        if i + 1 < nu and (j, i + 1) in parent and not block_u[j, i + 1]:
            union((j, i), (j, i + 1))
        if j + 1 < nv and (j + 1, i) in parent and not block_v[j + 1, i]:
            union((j, i), (j + 1, i))
    groups = {}
    for c in parent:
        groups.setdefault(find(c), []).append(c)

    from shapely.geometry import box
    from shapely.ops import unary_union

    rooms = []
    for cells in groups.values():
        area = sum(w[j, i] * h[j, i] for j, i in cells)
        if area < min_room_area:
            continue
        poly = unary_union([box(U[i], V[j], U[i + 1], V[j + 1]) for j, i in cells]).buffer(0)
        if poly.geom_type != "Polygon":
            poly = max(poly.geoms, key=lambda p: p.area)
        rooms.append(dict(cells=set(cells), polygon=poly, area=area))
    rooms.sort(key=lambda r: -r["area"])
    # connectors -> doors between rooms (or absorbed into the single room they touch)
    cell_room = {}
    for k, r in enumerate(rooms):
        for c in r["cells"]:
            cell_room[c] = k
    doors = []
    for j in range(nv):
        for i in range(nu):
            if not connector[j, i]:
                continue
            nb = set()
            for jj, ii, blk in ((j, i - 1, block_u[j, i]), (j, i + 1, block_u[j, i + 1] if i + 1 <= nu else True),
                                (j - 1, i, block_v[j, i]), (j + 1, i, block_v[j + 1, i] if j + 1 <= nv else True)):
                if (jj, ii) in cell_room and not blk:
                    nb.add(cell_room[(jj, ii)])
            thin_u = w[j, i] < h[j, i]
            width = h[j, i] if thin_u else w[j, i]
            doors.append(dict(rooms=sorted(nb), width=float(width), cell=(U[i], V[j], U[i + 1], V[j + 1]),
                              orient="V" if thin_u else "H"))
    return dict(rooms=rooms, doors=doors, segments=segs, U=U, V=V, interior=interior, connector=connector)


def rasterize_rooms(result, grid):
    """Label image compatible with the rest of the pipeline."""
    import cv2

    lab = np.zeros(grid.shape, np.int32)
    for k, r in enumerate(result["rooms"], start=1):
        pts = np.array(r["polygon"].exterior.coords)
        px = ((pts - grid.origin) / grid.res).astype(np.int32)
        cv2.fillPoly(lab, [px], k)
        for hole in r["polygon"].interiors:
            hp = ((np.array(hole.coords) - grid.origin) / grid.res).astype(np.int32)
            cv2.fillPoly(lab, [hp], 0)
    return lab


def cells_from_labels(q, nq, rasters, labels, free_thr=0.45, min_cover=0.3):
    """Hybrid: room *identity* from the watershed labels (door-width splitting), room *geometry*
    from the wall-line arrangement. Each interior cell takes the label covering most of it, so
    room boundaries snap to fitted wall faces instead of ray-reach.
    """
    G = rasters.grid
    segs = extract_segments(q, nq)
    U = _merge_coords([s.coord for s in segs if s.orient == "V"])
    V = _merge_coords([s.coord for s in segs if s.orient == "H"])
    lo = G.origin
    hi = G.origin + np.array([G.shape[1], G.shape[0]]) * G.res
    U = [lo[0]] + [u for u in U if lo[0] < u < hi[0]] + [hi[0]]
    V = [lo[1]] + [v for v in V if lo[1] < v < hi[1]] + [hi[1]]
    out = np.zeros_like(labels)
    nl = labels.max()
    for j in range(len(V) - 1):
        r0, r1 = int((V[j] - lo[1]) / G.res), int((V[j + 1] - lo[1]) / G.res)
        if r1 <= r0:
            continue
        for i in range(len(U) - 1):
            c0, c1 = int((U[i] - lo[0]) / G.res), int((U[i + 1] - lo[0]) / G.res)
            if c1 <= c0:
                continue
            blk = labels[r0:r1, c0:c1]
            fr = rasters.free[r0:r1, c0:c1].mean()
            cnt = np.bincount(blk.ravel(), minlength=nl + 1)
            cnt[0] = 0
            k = int(cnt.argmax())
            if cnt[k] >= min_cover * blk.size and fr > free_thr:
                out[r0:r1, c0:c1] = k
    return out, segs
