"""Registration of unposed photos by top-view correlation in a gravity + Manhattan frame.

Feature matching fails on the sample flat (SIFT+PnP linked 28 photos into 19 blocks; FPFH global
registration gave >0.5 m errors on 30/42 pairs: white walls and repeated planes). What each photo
does give reliably, once it has metric depth, is (a) gravity (floor plane), (b) the Manhattan wall
directions, and (c) a top-view map of the vertical structure it saw. After (a)+(b) the unknowns
are one of four 90° yaws and a 2-D translation, which an FFT cross-correlation of top-view maps
solves exhaustively. Photos are added greedily to a room map by best normalised correlation;
rooms are joined the same way through the doorway shots that see into the next room.
"""
from __future__ import annotations

import numpy as np
from scipy import ndimage as ndi
from scipy.signal import fftconvolve

RES = 0.05


def _rotz_yaw(theta_deg):
    t = np.radians(theta_deg)
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def gravity_manhattan(points_cam, normals_cam):
    """Rotation taking camera coords to a frame with +y up and walls on x/z axes; floor y (frame)."""
    from ..geometry.frame import dominant_yaw, horizontal_peaks

    g = np.array([0, -1.0, 0])                     # upright photo: image-up is world-up
    for _ in range(3):
        c = normals_cam @ g
        sel = np.abs(c) > np.cos(np.radians(20))
        if sel.sum() < 200:
            break
        nn = normals_cam[sel] * np.sign(c[sel])[:, None]
        g = nn.mean(0)
        g /= np.linalg.norm(g)
    y = np.array([0, 1.0, 0])
    v = np.cross(g, y)
    s, cth = np.linalg.norm(v), g @ y
    if s < 1e-9:
        Rg = np.eye(3)
    else:
        vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        Rg = np.eye(3) + vx + vx @ vx * ((1 - cth) / s ** 2)
    n2 = normals_cam @ Rg.T
    yaw = dominant_yaw(n2)
    Rm = _rotz_yaw(-yaw) if False else np.eye(3)
    # rotate about y so that the dominant wall normal is along +x
    t = np.radians(yaw)
    Ry = np.array([[np.cos(t), 0, np.sin(t)], [0, 1, 0], [-np.sin(t), 0, np.cos(t)]])
    R = Ry @ Rg
    p = points_cam @ R.T
    up = np.abs((normals_cam @ R.T)[:, 1]) > 0.9
    pk = horizontal_peaks(p[up, 1], min_frac=0.05) if up.sum() > 50 else []
    floor = pk[0][0] if pk else np.percentile(p[:, 1], 2)
    return R, float(floor)


def topview(points, floor_y, lo, shape):
    h = points[:, 1] - floor_y
    m = (h > 0.15) & (h < 2.2)
    img = np.zeros(shape, np.float32)
    ij = ((points[m][:, [0, 2]] - lo) / RES).astype(int)
    ok = (ij[:, 0] >= 0) & (ij[:, 0] < shape[1]) & (ij[:, 1] >= 0) & (ij[:, 1] < shape[0])
    np.add.at(img, (ij[ok, 1], ij[ok, 0]), 1.0)
    img = np.log1p(img)
    return ndi.gaussian_filter(img, 1.0)


def _ncc_best(A, B):
    """Best translation of B onto A (same-size images). Returns (score, dy, dx)."""
    c = fftconvolve(A, B[::-1, ::-1], mode="same")
    iy, ix = np.unravel_index(np.argmax(c), c.shape)
    score = c.max() / (np.sqrt((A * A).sum() * (B * B).sum()) + 1e-9)
    dy, dx = iy - A.shape[0] // 2, ix - A.shape[1] // 2
    if A.shape[0] % 2 == 0:
        dy += 1
    if A.shape[1] % 2 == 0:
        dx += 1
    return float(score), int(dy), int(dx)


def _rot4(k):
    return _rotz_yaw(90 * k)


def register_set(clouds, min_score=0.25, extent=12.0):
    """clouds: list of (points (N,3) in a gravity+Manhattan frame with floor at y=0).

    Returns list of 4x4 transforms (None for photos that could not be placed) and scores.
    """
    n = len(clouds)
    shape = (int(2 * extent / RES),) * 2
    lo = np.array([-extent, -extent])
    rich = [len(c[(c[:, 1] > 0.15) & (c[:, 1] < 2.2)]) for c in clouds]
    order = list(np.argsort(rich)[::-1])
    T = [None] * n
    score = [0.0] * n
    seed = order[0]
    T[seed] = np.eye(4)
    placed_pts = [clouds[seed]]
    remaining = [i for i in order[1:]]
    while remaining:
        M = topview(np.concatenate(placed_pts), 0.0, lo, shape)
        best = None
        for i in remaining:
            c = clouds[i] - np.r_[np.median(clouds[i][:, 0]), 0, np.median(clouds[i][:, 2])]
            for k in range(4):
                ck = c @ _rot4(k).T
                B = topview(ck, 0.0, lo, shape)
                s, dy, dx = _ncc_best(M, B)
                if best is None or s > best[0]:
                    Tk = np.eye(4)
                    Tk[:3, :3] = _rot4(k)
                    Tk[:3, 3] = -_rot4(k) @ np.r_[np.median(clouds[i][:, 0]), 0, np.median(clouds[i][:, 2])] + \
                        np.array([dx * RES, 0, dy * RES])
                    best = (s, i, Tk)
        s, i, Tk = best
        remaining.remove(i)
        if s < min_score:
            continue
        T[i], score[i] = Tk, s
        placed_pts.append(clouds[i] @ Tk[:3, :3].T + Tk[:3, 3])
    return T, score


def register_blocks(blocks, min_score=0.15, extent=16.0):
    """Join room blocks (each an (N,3) cloud already in a common gravity+Manhattan frame) by the
    same correlation, penalising placements whose footprints overlap (rooms do not overlap)."""
    shape = (int(2 * extent / RES),) * 2
    lo = np.array([-extent, -extent])
    order = list(np.argsort([-len(b) for b in blocks]))
    T = [None] * len(blocks)
    T[order[0]] = np.eye(4)
    placed = [blocks[order[0]]]
    scores = {order[0]: 1.0}
    for i in order[1:]:
        M = topview(np.concatenate(placed), 0.0, lo, shape)
        occ = ndi.binary_dilation(topview(np.concatenate(placed), 0.0, lo, shape) > 0, iterations=1)
        c0 = np.r_[np.median(blocks[i][:, 0]), 0, np.median(blocks[i][:, 2])]
        best = None
        for k in range(4):
            ck = (blocks[i] - c0) @ _rot4(k).T
            B = topview(ck, 0.0, lo, shape)
            s, dy, dx = _ncc_best(M, B)
            if best is None or s > best[0]:
                Tk = np.eye(4)
                Tk[:3, :3] = _rot4(k)
                Tk[:3, 3] = -_rot4(k) @ c0 + np.array([dx * RES, 0, dy * RES])
                best = (s, Tk)
        s, Tk = best
        if s < min_score:
            # unlinked: park to the +x side of everything placed (flagged by caller)
            allp = np.concatenate(placed)
            ck = blocks[i] @ np.eye(3).T
            shift = allp[:, 0].max() - ck[:, 0].min() + 1.0
            Tk = np.eye(4)
            Tk[0, 3] = shift
            Tk[2, 3] = allp[:, 2].min() - ck[:, 2].min()
            s = 0.0
        T[i] = Tk
        scores[i] = s
        placed.append(blocks[i] @ Tk[:3, :3].T + Tk[:3, 3])
    return T, [scores[i] for i in range(len(blocks))]
