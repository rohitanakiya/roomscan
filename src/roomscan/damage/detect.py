"""2D damage candidate detectors (classical; no learned weights required).

Classes:
  water_stain   diffuse discoloured blob, darker/yellower than the surrounding paint
  crack         thin, dark, elongated, non-straight ridge
  mould         dark, high-frequency speckle cluster

Every detector runs only inside a `surface_mask` (pixels whose 3D point lies on a fitted wall /
ceiling / floor plane), so furniture, plants, posters standing proud of the wall are excluded
before any colour test. Candidates are deliberately permissive here; specificity comes from
multi-view agreement in 3D (pipeline.py).
"""
from __future__ import annotations

import cv2
import numpy as np


def _lab(img_bgr):
    return cv2.cvtColor(img_bgr, cv2.COLOR_BGR2LAB).astype(np.float32)


def stain_candidates(img_bgr, surface_mask, min_px=150):
    lab = _lab(img_bgr)
    L, a, b = lab[..., 0], lab[..., 1], lab[..., 2]
    m = surface_mask.astype(bool)
    if m.sum() < 500:
        return []
    # local background: large median of the surface (illumination gradients are low-frequency)
    # background = wide median computed at 1/4 resolution (window ~1/3 of the frame, so a stain
    # does not become its own background)
    hh, ww = L.shape
    def bg(ch):
        small = cv2.resize(np.clip(ch, 0, 255).astype(np.uint8), (ww // 4, hh // 4), interpolation=cv2.INTER_AREA)
        k = max(31, (min(small.shape) // 3) | 1)
        return cv2.resize(cv2.medianBlur(small, k), (ww, hh), interpolation=cv2.INTER_LINEAR).astype(np.float32)
    Lb, bb = bg(L), bg(b)
    dL = Lb - L                      # darker than surroundings
    db = b - bb                      # yellower (water tide marks are yellow-brown)
    score = np.clip(dL / 15.0, 0, None) + np.clip(db / 8.0, 0, None)
    score[~m] = 0
    # stains are smooth: suppress sharp edges (tile grout, frames, shadows of objects have hard edges)
    edges = cv2.Canny(np.clip(L, 0, 255).astype(np.uint8), 40, 100) > 0
    edge_density = cv2.blur(edges.astype(np.float32), (15, 15))
    cand = (score > 1.2) & (edge_density < 0.12)
    cand = cv2.morphologyEx(cand.astype(np.uint8), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    cand = cv2.morphologyEx(cand, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    n, lab_img, stats, _ = cv2.connectedComponentsWithStats(cand)
    out = []
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < min_px:
            continue
        mask = lab_img == i
        # reject if touching the surface-mask border (likely a shadow at a corner / object edge)
        ring = cv2.dilate(mask.astype(np.uint8), np.ones((7, 7), np.uint8)).astype(bool) & ~mask
        if (ring & ~m).mean() > 0.0 and (ring & ~m).sum() > 0.35 * ring.sum():
            continue
        ys, xs = np.nonzero(mask)
        (_, _), (rw, rh), _ = cv2.minAreaRect(np.c_[xs, ys].astype(np.float32))
        if max(rw, rh) > 5 * max(min(rw, rh), 1):
            continue  # elongated: a line/edge/crack, not a stain
        contrast = float(score[mask].mean())
        out.append(dict(cls="water_stain", mask=mask, score=min(1.0, contrast / 3.0)))
    return out


def crack_candidates(img_bgr, surface_mask, min_len_px=40):
    g = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    m = surface_mask.astype(bool)
    if m.sum() < 500:
        return []
    bh = cv2.morphologyEx(g, cv2.MORPH_BLACKHAT, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9)))
    bh[~m] = 0
    thr = max(25, np.percentile(bh[m], 99.0))
    cand = (bh > thr).astype(np.uint8)
    n, lab_img, stats, _ = cv2.connectedComponentsWithStats(cand, connectivity=8)
    out = []
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        length = np.hypot(w, h)
        if length < min_len_px or area / max(length, 1) > 7.0:
            continue  # too short, or too thick (mean width > 7 px) to be a crack
        ys, xs = np.nonzero(lab_img == i)
        pts = np.c_[xs, ys].astype(np.float32)
        # straightness: ratio of principal-axis spread; grout lines / edges are near-perfect lines
        c = np.cov(pts.T)
        ev = np.sort(np.linalg.eigvalsh(c))
        straight = 1 - ev[0] / (ev[1] + 1e-9)
        if straight > 0.995 and length > 120:
            continue
        mask = lab_img == i
        mask = cv2.dilate(mask.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
        out.append(dict(cls="crack", mask=mask, score=float(min(1.0, bh[lab_img == i].mean() / 40)),
                        length_px=float(length)))
    return out


def mould_candidates(img_bgr, surface_mask, min_px=200):
    g = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY).astype(np.float32)
    m = surface_mask.astype(bool)
    if m.sum() < 500:
        return []
    bg = cv2.GaussianBlur(g, (0, 0), 15)
    dark = (bg - g) > 18
    speck = cv2.blur(dark.astype(np.float32), (21, 21))
    hf = np.abs(g - cv2.GaussianBlur(g, (0, 0), 2))
    tex = cv2.blur(hf, (21, 21))
    cand = (speck > 0.30) & (speck < 0.8) & (tex > 6) & m
    cand = cv2.morphologyEx(cand.astype(np.uint8), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, lab_img, stats, _ = cv2.connectedComponentsWithStats(cand)
    out = []
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < min_px:
            continue
        mask = lab_img == i
        out.append(dict(cls="mould", mask=mask, score=float(min(1.0, speck[mask].mean() * 2))))
    return out


def detect_all(img_bgr, surface_mask):
    return (stain_candidates(img_bgr, surface_mask) + crack_candidates(img_bgr, surface_mask)
            + mould_candidates(img_bgr, surface_mask))


# ---------------------------------------------------------------- orthomosaic detectors
def _fill(ch, valid):
    """Nearest-valid fill so filters don't see holes as dark."""
    from scipy import ndimage as ndi

    idx = ndi.distance_transform_edt(~valid, return_distances=False, return_indices=True)
    return ch[tuple(idx)]


def detect_ortho(lab_mean, valid, res):
    """Detect damage on a surface orthomosaic (mean Lab colour per cell, metric grid).

    Working on the surface itself (instead of per image) gives metric scale for free, makes the
    background estimate surface-local (a grey tile wall is not 'darker' than itself), and averages
    away view-dependent shading.
    """
    from scipy import ndimage as ndi

    out = []
    L = _fill(lab_mean[..., 0], valid)
    b = _fill(lab_mean[..., 2], valid)
    win = max(15, int(0.6 / res) | 1)                        # 0.6 m background window
    Lb = ndi.median_filter(L, size=win)
    bb = ndi.median_filter(b, size=win)
    score = np.clip((Lb - L) / 15.0, 0, None) + np.clip((b - bb) / 8.0, 0, None)
    score[~valid] = 0
    # stains: smooth blobs
    cand = ndi.binary_opening(score > 1.0, iterations=2)
    cand = ndi.binary_closing(cand, iterations=2) & valid
    lab_img, n = ndi.label(cand)
    for i in range(1, n + 1):
        m = lab_img == i
        area = m.sum() * res * res
        if area < 0.01:
            continue
        ys, xs = np.nonzero(m)
        (_, _), (rw, rh), _ = cv2.minAreaRect(np.c_[xs, ys].astype(np.float32))
        if max(rw, rh) > 5 * max(min(rw, rh), 1):
            continue
        # tile/fixture rejection: a uniform rectangle aligned with the surface axes is an object
        fill = m.sum() / max(rw * rh, 1)
        if fill > 0.92 and area > 0.05:
            continue
        if (b - bb)[m].mean() < 1.0:
            continue  # neutral darkening = shadow / grime / grey tile; water stains shift yellow-brown
        out.append(dict(cls="water_stain", mask=m, score=float(score[m].mean())))
    # cracks: thin dark ridges, not straight axis-aligned lines (grout, panel joints)
    Lu = np.clip(L, 0, 255).astype(np.uint8)
    bh = cv2.morphologyEx(Lu, cv2.MORPH_BLACKHAT, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))).astype(np.float32)
    bh[~ndi.binary_erosion(valid, iterations=3)] = 0
    ridge = bh > max(18, np.percentile(bh[valid], 99.0) if valid.any() else 18)
    lab_img, n = ndi.label(ridge, structure=np.ones((3, 3)))
    for i in range(1, n + 1):
        m = lab_img == i
        ys, xs = np.nonzero(m)
        if len(xs) < 10:
            continue
        length = np.hypot(np.ptp(xs) + 1, np.ptp(ys) + 1) * res
        if length < 0.15 or m.sum() * res / max(length, 1e-6) > 0.012:   # mean width > 12 mm
            continue
        c = np.cov(np.c_[xs, ys].T.astype(float))
        ev, evec = np.linalg.eigh(c)
        straight = 1 - ev[0] / (ev[1] + 1e-9)
        ang = np.degrees(np.arctan2(evec[1, 1], evec[0, 1])) % 90
        axis_aligned = min(ang, 90 - ang) < 4
        if straight > 0.985 and axis_aligned:
            continue
        out.append(dict(cls="crack", mask=ndi.binary_dilation(m), score=float(bh[m].mean() / 40), length_m=length))
    # mould: dark speckle clusters (high local variance of darkness)
    dark = (Lb - L) > 12
    speck = ndi.uniform_filter(dark.astype(np.float32), size=max(5, int(0.1 / res)))
    hf = np.abs(L - ndi.gaussian_filter(L, 1.5))
    tex = ndi.uniform_filter(hf, size=max(5, int(0.1 / res)))
    cand = (speck > 0.45) & (speck < 0.9) & (tex > 8) & valid
    cand = ndi.binary_opening(cand, iterations=2)
    lab_img, n = ndi.label(cand)
    for i in range(1, n + 1):
        m = lab_img == i
        if m.sum() * res * res < 0.02:
            continue
        out.append(dict(cls="mould", mask=m, score=float(speck[m].mean())))
    return out
