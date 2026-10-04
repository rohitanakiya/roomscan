"""Assemble a measured room from a raster mask: walls, area, ceiling, openings."""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from matplotlib.path import Path as MplPath

from ..measure import Measure, combine
from .frame import horizontal_peaks, refine_plane_height
from .layout import LayoutParams, _corners, room_polygon, snap_edge
from .openings import detect_openings


@dataclass
class ErrorModel:
    """1-sigma error terms by tier. Calibrated in bench/calibrate.py; defaults are priors."""
    sensor_face: float = 0.004        # m, residual bias of a fitted LiDAR wall face
    scale_rel: float = 0.003          # relative scale error of the pose/depth system
    drift_per_m: float = 0.0005       # m of drift per m of camera path between observations
    ceiling_plane: float = 0.004
    unobserved_face: float = 0.05     # m, face we could not fit (raster position only)
    opening_jamb: float = 0.006
    ambiguity_k: float = 0.5          # sigma = k * (spread of competing face layers); calibrated in bench/calibrate.py
    face_floor: float = 0.0           # m, per-face residual found by calibration (inter-capture repeatability)


@dataclass
class Room:
    id: str
    label: int
    polygon: np.ndarray                        # (N,2) plan coords, CCW
    walls: list = field(default_factory=list)
    openings: list = field(default_factory=list)
    floor_area: Measure | None = None
    perimeter: Measure | None = None
    ceiling_height: Measure | None = None
    floor_h: float = 0.0
    meta: dict = field(default_factory=dict)


def _signed_area(p):
    x, y = p[:, 0], p[:, 1]
    return 0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y)


def _simplify(S, min_edge, merge_tol=0.05):
    """Post-snap cleanup: merge consecutive collinear faces, drop jogs shorter than min_edge."""
    def merge_same(S):
        changed = True
        while changed and len(S) > 4:
            changed = False
            for i in range(len(S)):
                j = (i + 1) % len(S)
                if S[i]["o"] == S[j]["o"]:
                    keep = S[i] if (S[i]["fit"].observed, S[i]["fit"].n) >= (S[j]["fit"].observed, S[j]["fit"].n) else S[j]
                    S[i] = keep
                    S.pop(j)
                    changed = True
                    break
        return S

    S = merge_same(list(S))
    for _ in range(200):
        if len(S) <= 4:
            break
        corners = _corners([[d["o"], d["coord"], 0] for d in S])
        L = [np.hypot(*(corners[(i + 1) % len(S)] - corners[i])) for i in range(len(S))]
        # a jog between two parallel faces at (almost) the same coordinate is noise
        cand = []
        for i in range(len(S)):
            prv, nxt = S[i - 1], S[(i + 1) % len(S)]
            same = prv["o"] == nxt["o"] and abs(prv["coord"] - nxt["coord"]) < merge_tol
            if L[i] < min_edge or same:
                cand.append((0 if same else 1, L[i], i))
        if not cand:
            break
        _, _, i = min(cand)
        S.pop(i)
        S = merge_same(S)
    return S


def build_room(label, mask, rasters, labels, q, nq, P: LayoutParams, E: ErrorModel, path_len_m=0.0,
               room_id=None, have_ceiling_view=True):
    _, segs = room_polygon(mask, rasters.grid, P.min_edge)
    # corners[i] = seg[i-1] ∩ seg[i]; seg i runs corner i -> corner i+1. Force CCW.
    corners = _corners(segs)
    if _signed_area(corners) < 0:
        segs = segs[::-1]
        corners = _corners(segs)
    path = MplPath(corners)
    S = []
    for i in range(len(segs)):
        o, c0, _ = segs[i]
        a, b = corners[i], corners[(i + 1) % len(segs)]
        ax_n = 1 if o == "H" else 0
        probe = 0.5 * (a + b)
        probe[ax_n] += 0.05
        inward = 1 if path.contains_point(probe) else -1
        ta, tb = (a[0], b[0]) if o == "H" else (a[1], b[1])
        fit = snap_edge(o, c0, ta, tb, inward, q, nq, P)
        S.append(dict(o=o, coord=fit.coord, fit=fit))
    S = _simplify(S, P.min_edge)
    segs = [[d["o"], d["coord"], 0.0] for d in S]
    fits = []
    corners = _corners(segs)
    path = MplPath(corners)
    for i, d in enumerate(S):
        a, b = corners[i], corners[(i + 1) % len(S)]
        ax_n = 1 if d["o"] == "H" else 0
        probe = 0.5 * (a + b)
        probe[ax_n] += 0.03
        fits.append((d["fit"], 1 if path.contains_point(probe) else -1))
    n = len(S)
    corners = _corners(segs)
    walls = []
    drift = E.drift_per_m * path_len_m
    for i in range(n):
        fit, inward = fits[i]
        a, b = corners[i], corners[(i + 1) % n]
        L = float(np.hypot(*(b - a)))
        fa, _ = fits[i - 1]
        fb, _ = fits[(i + 1) % n]
        sa = fa.sigma if fa.observed else E.unobserved_face
        sb = fb.sigma if fb.observed else E.unobserved_face
        length = combine(L, face_fit_a=sa, face_fit_b=sb, sensor=np.sqrt(2) * E.sensor_face,
                         scale=E.scale_rel * L, drift=drift,
                         face_choice_a=E.ambiguity_k * fa.ambiguity, face_choice_b=E.ambiguity_k * fb.ambiguity,
                         calibration=np.sqrt(2) * E.face_floor)
        if not (fa.observed and fb.observed):
            length.note = "one bounding wall not directly observed; raster position used"
        walls.append(dict(index=i, orient=fit.orient, coord=fit.coord, start=a, end=b, inward=inward,
                          length=length, fit=fit))
    # area & perimeter
    area = abs(_signed_area(corners))
    var_area = 0.0
    for i, w in enumerate(walls):
        s = w["fit"].sigma if w["fit"].observed else E.unobserved_face
        s = np.sqrt(s ** 2 + E.sensor_face ** 2 + (E.ambiguity_k * w["fit"].ambiguity) ** 2 + E.face_floor ** 2)
        var_area += (w["length"].value * s) ** 2
    perim = sum(w["length"].value for w in walls)
    floor_area = combine(area, unit="m2", wall_positions=np.sqrt(var_area), scale=2 * E.scale_rel * area,
                         drift=drift * perim / 2)
    perimeter = combine(perim, wall_positions=np.sqrt(sum(w["length"].sigma ** 2 for w in walls)))
    room = Room(room_id or f"room_{label}", label, corners, walls, [], floor_area, perimeter)

    # ceiling + local floor
    inside = MplPath(corners).contains_points(q[:, :2])
    up = np.abs(nq[:, 2]) > 0.9
    fl_c = refine_plane_height(q[inside & up, 2], 0.0, band=0.06)
    room.floor_h = fl_c[0]
    hs = q[inside & up & (q[:, 2] > 1.9), 2]
    if len(hs) > 300:
        pk = horizontal_peaks(hs, min_frac=0.1)
        top = max(pk, key=lambda t: t[1])[0] if pk else np.median(hs)
        cy, cs, cn = refine_plane_height(hs, top, band=0.04)
        H = cy - fl_c[0]
        room.ceiling_height = combine(H, floor_fit=fl_c[1] / np.sqrt(max(fl_c[2], 1)) * 1.25,
                                      ceiling_fit=cs / np.sqrt(max(cn, 1)) * 1.25,
                                      sensor=np.sqrt(2) * E.ceiling_plane, scale=E.scale_rel * H)
        room.meta["ceiling_points"] = int(cn)
    else:
        wall_top = q[inside & (np.abs(nq[:, 2]) < 0.3), 2]
        lb = float(np.percentile(wall_top, 99.5)) if len(wall_top) else 2.2
        # unobserved: report a prior interval anchored by the highest observed wall point
        val = max(2.75, lb + 0.05)
        m = Measure(val, max(0.2, (val - lb) / 1.96), "m", {"prior_unobserved": 0.2}, observed=False,
                    note=f"ceiling not observed in this capture; lower bound from highest wall point {lb:.2f} m")
        room.ceiling_height = m

    # openings
    for w in walls:
        o = w["orient"]
        a, b = w["start"], w["end"]
        ta, tb = (a[0], b[0]) if o == "H" else (a[1], b[1])
        edge = dict(orient=o, coord=w["coord"], t_lo=min(ta, tb), t_hi=max(ta, tb), inward=w["inward"])
        ops = detect_openings(edge, q, rasters, labels, label)
        for op in ops:
            op.wall_index = w["index"]
            room.openings.append(op)
    return room
