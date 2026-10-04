"""Opening (door / window / passage) detection on fitted wall faces.

A wall face is sampled in 2 cm bins along its length. An opening is a run of bins with no
surface points in the door band *and* positive see-through evidence: the visibility raster is
free on the far side of the wall. Requiring see-through evidence is what separates a real
opening from a wall that simply wasn't observed (which would otherwise be a phantom opening).
Widths come from the jamb points themselves (sub-bin), not from bin edges.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

BIN = 0.02


@dataclass
class Opening:
    kind: str            # door | window | passage
    t0: float            # jamb positions along the wall tangent (plan coords)
    t1: float
    width: float
    width_sigma: float
    height: float | None
    height_sigma: float | None
    sill: float | None
    through_frac: float
    neighbor_label: int  # room label on the far side (0 = exterior / unknown)

    @property
    def centre(self):
        return 0.5 * (self.t0 + self.t1)


def _runs(mask):
    out, start = [], None
    for i, v in enumerate(mask):
        if v and start is None:
            start = i
        elif not v and start is not None:
            out.append((start, i))
            start = None
    if start is not None:
        out.append((start, len(mask)))
    return out


def detect_openings(edge, q, rasters, labels, self_label, min_width=0.55, max_door=1.4,
                    min_through=0.5, face_tol=0.06):
    """edge: dict(orient, coord, t_lo, t_hi, inward) in plan coords."""
    ax_n, ax_t = (1, 0) if edge["orient"] == "H" else (0, 1)
    c, lo, hi, inward = edge["coord"], edge["t_lo"], edge["t_hi"], edge["inward"]
    L = hi - lo
    if L < min_width:
        return []
    nb = int(np.ceil(L / BIN))
    near = (np.abs(q[:, ax_n] - c) < face_tol) & (q[:, ax_t] > lo - 0.05) & (q[:, ax_t] < hi + 0.05)
    pts = q[near]
    t, h = pts[:, ax_t], pts[:, 2]
    bi = np.clip(((t - lo) / BIN).astype(int), 0, nb - 1)
    low = np.zeros(nb, bool)
    mid = np.zeros(nb, bool)
    top = np.zeros(nb, bool)
    low[bi[(h > 0.08) & (h < 0.9)]] = True
    mid[bi[(h >= 0.9) & (h < 1.7)]] = True
    top[bi[(h >= 1.7) & (h < 2.6)]] = True

    # see-through evidence on the far side of the wall
    G = rasters.grid
    tc = lo + (np.arange(nb) + 0.5) * BIN
    through = np.zeros(nb)
    nbr = np.zeros(nb, int)
    for off in (0.30, 0.45, 0.60):
        n_coord = c - inward * off
        uv = np.zeros((nb, 2))
        uv[:, ax_t] = tc
        uv[:, ax_n] = n_coord
        ij = G.to_px(uv)
        ok = (ij[:, 0] >= 0) & (ij[:, 0] < G.shape[1]) & (ij[:, 1] >= 0) & (ij[:, 1] < G.shape[0])
        f = np.zeros(nb)
        f[ok] = rasters.free[ij[ok, 1], ij[ok, 0]]
        through = np.maximum(through, f)
        lab = np.zeros(nb, int)
        lab[ok] = labels[ij[ok, 1], ij[ok, 0]]
        nbr = np.where((nbr == 0) & (lab != self_label), lab, nbr)

    out = []
    # small morphological tolerance: a single occupied bin (handle, noise) doesn't close a gap
    empty = ~(low | mid)
    for s, e in _runs(empty):
        w_bins = e - s
        if w_bins * BIN < min_width * 0.8:
            continue
        thr = through[s:e].mean()
        if thr < min_through:
            continue
        # sub-bin jambs from actual points; a run touching the edge end is bounded by the corner
        left = t[(t < tc[s]) & ((h > 0.08) & (h < 1.7))]
        right = t[(t > tc[e - 1]) & ((h > 0.08) & (h < 1.7))]
        t0 = float(left.max()) if (s > 0 and len(left)) else lo
        t1 = float(right.min()) if (e < nb and len(right)) else hi
        width = t1 - t0
        if width < min_width:
            continue
        # jamb precision: spread of the outermost points at each jamb
        def jamb_sigma(arr, side):
            if len(arr) < 5:
                return 0.02
            a = np.sort(arr)[-10:] if side == "l" else np.sort(arr)[:10]
            return float(max(np.std(a), 0.004))
        ws = float(np.hypot(jamb_sigma(left, "l") if s > 0 else 0.01, jamb_sigma(right, "r") if e < nb else 0.01))
        # header -> door height
        in_gap = (t > t0 + 0.03) & (t < t1 - 0.03)
        hdr = h[in_gap & (h > 1.7)]
        height, hs = (float(np.percentile(hdr, 2)), 0.015) if len(hdr) > 20 else (None, None)
        lab_far = np.bincount(nbr[s:e][nbr[s:e] > 0]).argmax() if (nbr[s:e] > 0).any() else 0
        kind = "door" if width <= max_door else "passage"
        out.append(Opening(kind, t0, t1, width, ws, height, hs, None, float(thr), int(lab_far)))

    # windows: wall present low, absent mid, see-through
    for s, e in _runs(low & ~mid & (through > 0)):
        if (e - s) * BIN < 0.4:
            continue
        lowpts = h[((t > tc[s]) & (t < tc[e - 1])) & (h < 1.2)]
        sill = float(np.percentile(lowpts, 98)) if len(lowpts) else None
        out.append(Opening("window", float(tc[s]), float(tc[e - 1]), float((e - s) * BIN), 0.02,
                           None, None, sill, float(through[s:e].mean()), 0))
    return out
