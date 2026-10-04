"""Room layout extraction from a gravity-aligned point cloud + camera rays.

Pipeline (all in Manhattan-aligned plan coordinates u, v, h):
  1. free-space raster: union of per-keyframe visibility fans (what the camera saw through)
  2. wall raster: cells whose vertical-surface points span > `wall_min_extent` metres of height
  3. interior = free minus walls; rooms = watershed on distance transform, seeded by cores
     that survive a door-width erosion (doors < 2*door_half_width wide split rooms)
  4. rectilinear polygon per room, each edge snapped to the wall face fitted from raw points
"""
from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np
from scipy import ndimage as ndi
from skimage.segmentation import watershed

from .raster import Grid, disk, fill_small_holes


@dataclass
class LayoutParams:
    res: float = 0.02
    wall_min_extent: float = 0.9      # m of vertical coverage for a wall cell
    door_half_width: float = 0.42     # cores separated by passages narrower than 2x this
    min_room_area: float = 1.2        # m^2
    min_wall_support: float = 0.35    # fraction of room boundary that must touch walls
    max_depth: float = 5.0
    snap_window: float = 0.15         # m search either side of the raster edge
    snap_outward: float = 0.50        # m beyond the raster edge searched for the room-side wall face
    wall_floor_reach: float = 0.45    # m: a wall face's lowest 5% of points start below this height
    min_edge: float = 0.25            # m, shorter rectilinear jogs are merged away
    grow_to_walls: float = 0.10
    cell_geometry: bool = True        # snap room geometry to the wall-line arrangement
    door_closures: bool = True        # close door-width gaps between collinear wall faces before splitting rooms       # m, geodesic growth of room masks up to wall faces


@dataclass
class Rasters:
    grid: Grid
    free: np.ndarray
    wall: np.ndarray
    wall_ext: np.ndarray
    traj_px: np.ndarray


def visibility_raster(views, frame, grid: Grid):
    """views: iterable of (camera_centre_world (3,), points_world (M,3)).

    Each view contributes its 2D visibility fan (camera -> farthest return per 1 deg azimuth):
    everything inside the fan was seen through, so it is free space in plan.
    """
    free = np.zeros(grid.shape, np.uint8)
    for cam, pw in views:
        pq = frame.to_plan(pw)
        pq = pq[(pq[:, 2] > -0.1) & (pq[:, 2] < 4.0)]  # incl. ceiling returns: the ray to them crossed free space
        if len(pq) < 50:
            continue
        c = frame.to_plan(np.asarray(cam)[None])[0, :2]
        rel = pq[:, :2] - c
        az = np.arctan2(rel[:, 1], rel[:, 0])
        r = np.hypot(rel[:, 0], rel[:, 1])
        b = ((az + np.pi) / (2 * np.pi) * 360).astype(int) % 360
        far = np.zeros(360)
        np.maximum.at(far, b, r)
        bins = np.nonzero(far)[0]
        if len(bins) < 3:
            continue
        ang = (bins + 0.5) / 360 * 2 * np.pi - np.pi
        pts = c + np.c_[np.cos(ang), np.sin(ang)] * far[bins, None]
        pc_px = grid.to_px(c[None])[0]
        px = grid.to_px(pts)
        for k in range(len(bins) - 1):
            if bins[k + 1] - bins[k] <= 2:
                cv2.fillConvexPoly(free, np.array([pc_px, px[k], px[k + 1]], np.int32), 1)
    return free


def wall_extent_raster(q, nq, grid: Grid):
    vert = (np.abs(nq[:, 2]) < 0.3) & (q[:, 2] > 0.05) & (q[:, 2] < 2.6)
    ij = grid.to_px(q[vert, :2])
    ok = (ij[:, 0] >= 0) & (ij[:, 0] < grid.shape[1]) & (ij[:, 1] >= 0) & (ij[:, 1] < grid.shape[0])
    ij, h = ij[ok], q[vert, 2][ok]
    lin = ij[:, 1] * grid.shape[1] + ij[:, 0]
    hmax = np.full(grid.shape[0] * grid.shape[1], -1.0)
    hmin = np.full_like(hmax, 9.0)
    np.maximum.at(hmax, lin, h)
    np.minimum.at(hmin, lin, h)
    ext = (hmax - hmin).reshape(grid.shape)
    ext[hmax.reshape(grid.shape) < 0] = 0
    return ndi.maximum_filter(ext, 3)


def build_rasters(views, cam_centres, frame, q, nq, P: LayoutParams) -> Rasters:
    grid = Grid.around(q[:, :2], P.res, pad=0.4)
    free = visibility_raster(views, frame, grid)
    ext = wall_extent_raster(q, nq, grid)
    wall = ext > P.wall_min_extent
    traj = frame.to_plan(np.asarray(cam_centres))[:, :2]
    return Rasters(grid, free.astype(bool), wall, ext, grid.to_px(traj))


def segment_rooms(R: Rasters, P: LayoutParams, barrier=None):
    """barrier: optional extra blocking raster (virtual doorway closures) — segmentation only."""
    res = R.grid.res
    W = R.wall if barrier is None else (R.wall | barrier)
    I = R.free & ~(cv2.dilate(W.astype(np.uint8), disk(1)) > 0)
    I = fill_small_holes(I, int(2.0 / res**2))
    I = cv2.morphologyEx(I.astype(np.uint8), cv2.MORPH_OPEN, disk(2)) > 0
    dist = ndi.distance_transform_edt(I) * res
    cores, n = ndi.label(dist > P.door_half_width)
    lab = watershed(-dist, cores, mask=I)
    # filter: area, wall support (rejects ray leaks through doors/windows into exterior)
    Wd = cv2.dilate(W.astype(np.uint8), disk(3)) > 0
    keep = []
    for k in range(1, n + 1):
        m = lab == k
        area = m.sum() * res**2
        if area < P.min_room_area:
            continue
        edge = m & ~(cv2.erode(m.astype(np.uint8), disk(1)) > 0)
        support = (edge & Wd).sum() / max(edge.sum(), 1)
        if support < P.min_wall_support:
            continue
        keep.append((k, area, support))
    out = np.zeros_like(lab)
    stats = []
    for new, (k, area, sup) in enumerate(sorted(keep, key=lambda t: -t[1]), start=1):
        out[lab == k] = new
        stats.append(dict(label=new, raster_area_m2=float(area), wall_support=float(sup)))
    return out, stats, dist


# ---------------------------------------------------------------- polygons
def _rectilinearize(pts: np.ndarray, min_edge: float):
    """pts: (N,2) closed polygon (no repeat). Returns axis-aligned polygon (M,2)."""
    n = len(pts)
    segs = []
    for i in range(n):
        a, b = pts[i], pts[(i + 1) % n]
        d = b - a
        L = np.hypot(*d)
        if L < 1e-9:
            continue
        o = "H" if abs(d[0]) >= abs(d[1]) else "V"  # H: constant v
        c = (a[1] + b[1]) / 2 if o == "H" else (a[0] + b[0]) / 2
        segs.append([o, c, L])
    # merge consecutive same orientation (length-weighted)
    changed = True
    while changed and len(segs) > 4:
        changed = False
        merged = []
        for s in segs:
            if merged and merged[-1][0] == s[0]:
                m = merged[-1]
                m[1] = (m[1] * m[2] + s[1] * s[2]) / (m[2] + s[2])
                m[2] += s[2]
                changed = True
            else:
                merged.append(list(s))
        if len(merged) > 1 and merged[0][0] == merged[-1][0]:
            m, s = merged[0], merged.pop()
            m[1] = (m[1] * m[2] + s[1] * s[2]) / (m[2] + s[2])
            m[2] += s[2]
            changed = True
        segs = merged
        # drop the shortest segment if below min_edge, then re-merge
        if len(segs) > 4:
            corners = _corners(segs)
            lens = [np.hypot(*(corners[(i + 1) % len(segs)] - corners[i])) for i in range(len(segs))]
            j = int(np.argmin(lens))
            if lens[j] < min_edge:
                segs.pop((j + 1) % len(segs))  # remove the segment between corner j and j+1
                changed = True
    return _corners(segs), segs


def _corners(segs):
    """Corner i is the intersection of seg i-1 and seg i."""
    out = []
    n = len(segs)
    for i in range(n):
        a, b = segs[i - 1], segs[i]
        if a[0] == "H" and b[0] == "V":
            out.append([b[1], a[1]])
        elif a[0] == "V" and b[0] == "H":
            out.append([a[1], b[1]])
        else:  # degenerate (same orientation) — fallback midpoint
            out.append([b[1], a[1]] if b[0] == "V" else [a[1], b[1]])
    return np.array(out, float)


def grow_to_walls(labels: np.ndarray, wall: np.ndarray, res: float, dist_m: float):
    """Geodesic growth of each room label into unlabelled, non-wall cells (furniture shadows,
    unobserved strips along walls) so room boundaries land on wall faces, not furniture."""
    out = labels.copy()
    blocked = wall
    k = np.ones((3, 3), np.uint8)
    for _ in range(int(round(dist_m / res))):
        grown = cv2.dilate(out.astype(np.float32), k).astype(out.dtype)
        cand = (out == 0) & ~blocked & (grown > 0)
        # avoid letting a label leak where two different labels compete
        mn = -cv2.dilate(-np.where(out > 0, out, 10**6).astype(np.float32), k)
        cand &= (mn.astype(out.dtype) == grown)
        if not cand.any():
            break
        out[cand] = grown[cand]
    return out


def room_polygon(mask: np.ndarray, grid: Grid, min_edge: float):
    m = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_CLOSE, disk(6))
    cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    c = max(cs, key=cv2.contourArea)
    ap = cv2.approxPolyDP(c, 4, True)[:, 0, :].astype(float)
    uv = grid.origin + (ap + 0.5) * grid.res
    poly, segs = _rectilinearize(uv, min_edge)
    return poly, segs


@dataclass
class WallFit:
    orient: str          # "H" (constant v) or "V" (constant u)
    coord: float         # fitted face coordinate
    sigma: float         # std error of the face position (m)
    rms: float           # point spread around face (m) — flatness / noise
    n: int
    raster_coord: float
    observed: bool


def snap_edge(orient, c0, a, b, inward_sign, q, nq, P: LayoutParams) -> WallFit:
    """Fit the room-side wall face behind a raster edge.

    orient H: edge at v=c0 spanning u in [a,b]; normal axis = v.
    inward_sign: +1 if the room interior is at larger coordinate.
    Candidates are surface layers whose (camera-oriented) normals face *into this room*, from
    slightly inside the raster edge to `P.snap_outward` beyond it. The chosen face is the
    *outermost* layer that is strong and reaches down to the floor: a desk or shelf front stands
    proud of the wall and stops short of the floor; the wall behind it does not. Faces of the
    neighbouring room point away from this room and are never candidates.
    """
    ax_n, ax_t = (1, 0) if orient == "H" else (0, 1)
    lo, hi = min(a, b) + 0.10, max(a, b) - 0.10
    if hi <= lo:
        lo, hi = min(a, b), max(a, b)
    out = -inward_sign
    near_in, far_out = 0.08, P.snap_outward
    s_lo = min(c0 + out * far_out, c0 - out * near_in)
    s_hi = max(c0 + out * far_out, c0 - out * near_in)
    sel = (
        (nq[:, ax_n] * inward_sign > 0.8)          # faces into this room
        & (q[:, ax_t] > lo) & (q[:, ax_t] < hi)
        & (q[:, ax_n] > s_lo) & (q[:, ax_n] < s_hi)
        & (q[:, 2] > 0.03) & (q[:, 2] < 2.6)
    )
    x = q[sel, ax_n]
    hgt = q[sel, 2]
    if len(x) < 30:
        return WallFit(orient, c0, 0.03, np.nan, int(len(x)), c0, False)
    e = np.arange(s_lo, s_hi + 0.01, 0.01)
    h, e = np.histogram(x, bins=e)
    hs = np.convolve(h, [1, 2, 1], mode="same") / 4.0
    thr = max(0.3 * hs.max(), 15 + 0.02 * (hi - lo) / 0.01)
    cand = [i for i in range(1, len(hs) - 1) if hs[i] >= thr and hs[i] >= hs[i - 1] and hs[i] >= hs[i + 1]]
    if not cand:
        return WallFit(orient, c0, 0.03, np.nan, int(len(x)), c0, False)
    centres = []
    for i in cand:
        c = 0.5 * (e[i] + e[i + 1])
        m = np.abs(x - c) < 0.02
        floor_reach = np.percentile(hgt[m], 5) if m.sum() > 10 else 9.0
        centres.append((c, floor_reach))
    full = [c for c, fr in centres if fr < P.wall_floor_reach]
    pool = full or [c for c, _ in centres]
    first = max(pool, key=lambda c: (c - c0) * out)     # outermost qualifying face
    best = (first, 0)
    v = x[np.abs(x - best[0]) < 0.03]
    for _ in range(3):
        med = np.median(v)
        mad = 1.4826 * np.median(np.abs(v - med)) + 1e-4
        v = v[np.abs(v - med) < 2.5 * mad]
    if len(v) < 10:
        return WallFit(orient, c0, 0.03, np.nan, int(len(v)), c0, False)
    med = float(np.median(v))
    rms = float(v.std())
    sigma = float(1.2533 * rms / np.sqrt(len(v)))
    return WallFit(orient, med, sigma, rms, int(len(v)), c0, True)
