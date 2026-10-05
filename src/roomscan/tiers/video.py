"""Video tier: a handheld walkthrough clip, nothing else (no depth, no poses, no intrinsics).

Reconstruction = monocular-metric-depth visual odometry:
  1. sample frames (default 3 fps), drop blurred ones
  2. intrinsics: focal from a prior on iPhone video FOV, refined from Manhattan vanishing points
  3. metric depth per frame (Depth Anything V2 metric-indoor), scaled by a calibrated factor
  4. frame-to-frame pose: KLT tracks lifted to 3D with frame-t depth, PnP-RANSAC in frame t+1
     (metric scale comes from the depth, so there is no SfM scale ambiguity); fallback to
     point-to-plane ICP between depth clouds when tracks are scarce (white walls)
  5. same fragment pose-graph loop closure as the LiDAR tier (drift)
  6. gravity from the camera "up" prior refined by the dominant horizontal plane
Classical SfM (COLMAP) was tried first: on the sample walkthroughs only 39% of frames
registered and the model split in three (low-texture walls) — see DECISIONS.md D7.
"""
from __future__ import annotations

import json
import subprocess
from pathlib import Path

import cv2
import numpy as np

from ..geometry.room import ErrorModel
from ..scene import Scene

# From measurement (D24): scale = leave-one-out depth-scale residual (max 5.2 %, bench/results/depth_calibration.json);
# drift = trajectory error vs ARKit per metre walked (2.7-8 %, bench/results/video_ablation.md), taken at 1.5 %/m
# because walls are fitted from views close in time; face = per-frame mono-depth scale scatter on a 2-4 m wall.
VIDEO_ERRORS = ErrorModel(sensor_face=0.030, scale_rel=0.050, drift_per_m=0.015,
                          ceiling_plane=0.030, unobserved_face=0.15, opening_jamb=0.04)
FOCAL_PRIOR = 0.83          # f / long-side (px); iPhone main camera, video mode with stabilisation crop
WORK_W = 640


import os as _os

# odometry components (D22); environment switches exist for the ablation in bench/video_ablation.py
# Default = lowest mean ATE in bench/results/video_ablation.md: fast-turn bridging on, Manhattan rotation off (it
# cuts ATE by 40 % on single_room but nearly doubles it on floor_only, where doors and fixtures mislead the yaw)
MANHATTAN_ROTATION = _os.environ.get("ROOMSCAN_VIDEO_MW", "0") == "1"      # absolute rotation from image lines
GAP_BRIDGING = _os.environ.get("ROOMSCAN_VIDEO_GAP", "1") == "1"           # native-rate rotation over fast turns
TRUSTED_ONLY = _os.environ.get("ROOMSCAN_VIDEO_TRUST", "1") == "1"         # fuse only trusted orientations
FLOOR_VETO_DEG = float(_os.environ.get("ROOMSCAN_VIDEO_VETO", "0"))        # 0 = off


def _gray(img):
    return img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def _probe_rotation(path):
    try:
        out = subprocess.check_output(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
                                       "stream=width,height:stream_side_data=rotation", "-of", "json", str(path)])
        return json.loads(out)
    except Exception:
        return {}


def extract_frames(video, out_dir: Path, fps=3.0, width=WORK_W):
    out_dir.mkdir(parents=True, exist_ok=True)
    if not any(out_dir.glob("*.jpg")):
        subprocess.check_call(["ffmpeg", "-v", "error", "-i", str(video), "-vf",
                               f"fps={fps},scale={width}:-2", "-q:v", "2", str(out_dir / "%05d.jpg")])
    files = sorted(out_dir.glob("*.jpg"))
    return files


def sharpness(img):
    g = _gray(img)
    return float(cv2.Laplacian(g, cv2.CV_64F).var())


def focal_from_vanishing_points(images, f0):
    """Median focal over frames from pairs of orthogonal vanishing points (Manhattan world)."""
    est = []
    lsd = cv2.createLineSegmentDetector(0)
    for img in images:
        g = _gray(img)
        h, w = g.shape
        c = np.array([w / 2, h / 2])
        lines = lsd.detect(g)[0]
        if lines is None or len(lines) < 30:
            continue
        L = lines.reshape(-1, 4)
        ln = np.hypot(L[:, 2] - L[:, 0], L[:, 3] - L[:, 1])
        L = L[ln > 0.06 * w]
        if len(L) < 20:
            continue
        hom = np.cross(np.c_[L[:, :2], np.ones(len(L))], np.c_[L[:, 2:], np.ones(len(L))])
        hom /= np.linalg.norm(hom[:, :2], axis=1, keepdims=True) + 1e-9
        rng = np.random.default_rng(0)
        vps = []
        remaining = np.arange(len(L))
        for _ in range(3):
            best, best_in = None, None
            for _ in range(300):
                if len(remaining) < 2:
                    break
                i, j = rng.choice(remaining, 2, replace=False)
                v = np.cross(hom[i], hom[j])
                if abs(v[2]) < 1e-9:
                    continue
                v = v / v[2]
                d = np.abs(hom[remaining] @ v) / (np.linalg.norm(v[:2]) + 1e-9)
                # angular consistency: line passes near v
                inl = remaining[np.abs(hom[remaining] @ v) < 2.0]
                if best_in is None or len(inl) > len(best_in):
                    best, best_in = v, inl
            if best is None or len(best_in) < 8:
                break
            vps.append(best[:2])
            remaining = np.setdiff1d(remaining, best_in)
        finite = [v for v in vps if np.linalg.norm(v - c) < 8 * w]
        for a in range(len(finite)):
            for b in range(a + 1, len(finite)):
                f2 = -np.dot(finite[a] - c, finite[b] - c)
                if f2 > 0:
                    f = np.sqrt(f2)
                    if 0.6 * f0 < f < 1.6 * f0:
                        est.append(f)
    if len(est) < 5:
        return f0, None
    return float(np.median(est)), float(1.4826 * np.median(np.abs(np.array(est) - np.median(est))))


def _pnp_step(img0, img1, d0, K, max_corners=600):
    g0 = _gray(img0)
    g1 = _gray(img1)
    p0 = cv2.goodFeaturesToTrack(g0, max_corners, 0.005, 8)
    if p0 is None or len(p0) < 12:
        return None, 0
    p1, st, _ = cv2.calcOpticalFlowPyrLK(g0, g1, p0, None, winSize=(21, 21), maxLevel=3)
    pb, stb, _ = cv2.calcOpticalFlowPyrLK(g1, g0, p1, None, winSize=(21, 21), maxLevel=3)
    ok = (st[:, 0] == 1) & (stb[:, 0] == 1) & (np.linalg.norm(pb - p0, axis=2)[:, 0] < 1.0)
    p0, p1 = p0[ok, 0], p1[ok, 0]
    if len(p0) < 12:
        return None, len(p0)
    u, v = p0[:, 0].astype(int), p0[:, 1].astype(int)
    z = d0[v.clip(0, d0.shape[0] - 1), u.clip(0, d0.shape[1] - 1)]
    m = (z > 0.2) & (z < 8)
    if m.sum() < 12:
        return None, int(m.sum())
    X = np.c_[(p0[m, 0] - K[0, 2]) / K[0, 0] * z[m], (p0[m, 1] - K[1, 2]) / K[1, 1] * z[m], z[m]]
    ok, rvec, tvec, inl = cv2.solvePnPRansac(X, p1[m], K, None, reprojectionError=2.0, iterationsCount=200,
                                             flags=cv2.SOLVEPNP_EPNP)
    if not ok or inl is None or len(inl) < 10:
        return None, 0 if inl is None else len(inl)
    rvec, tvec = cv2.solvePnPRefineLM(X[inl[:, 0]], p1[m][inl[:, 0]], K, None, rvec, tvec)
    R, _ = cv2.Rodrigues(rvec)
    T10 = np.eye(4)
    T10[:3, :3], T10[:3, 3] = R, tvec[:, 0]   # maps frame-0 camera coords -> frame-1 camera coords
    return np.linalg.inv(T10), len(inl)       # pose of camera 1 in camera-0 frame


def _icp_step(c0, c1, init=np.eye(4)):
    import open3d as o3d

    a = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(c1))
    b = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(c0))
    a, b = a.voxel_down_sample(0.05), b.voxel_down_sample(0.05)
    b.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.15, max_nn=30))
    r = o3d.pipelines.registration.registration_icp(
        a, b, 0.15, init, o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return r.transformation, r.fitness


def depth_cloud(d, K, stride=4, max_depth=5.0):
    h, w = d.shape
    v, u = np.mgrid[0:h:stride, 0:w:stride]
    z = d[::stride, ::stride]
    m = (z > 0.2) & (z < max_depth)
    return np.c_[(u[m] - K[0, 2]) / K[0, 0] * z[m], (v[m] - K[1, 2]) / K[1, 1] * z[m], z[m]]


def gravity_align(poses, xyz_fn):
    """Rotate world so +y is up. Prior: mean camera 'up' (-y_cam); refine with plane normals."""
    up_cam = -poses[:, :3, 1]
    g = up_cam.mean(0)
    g /= np.linalg.norm(g)
    pts = xyz_fn()
    import open3d as o3d

    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts)).voxel_down_sample(0.05)
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=30))
    n = np.asarray(pcd.normals)
    for _ in range(3):
        cosv = n @ g
        sel = np.abs(cosv) > np.cos(np.radians(15))
        if sel.sum() < 100:
            break
        nn = n[sel] * np.sign(cosv[sel])[:, None]
        g = nn.mean(0)
        g /= np.linalg.norm(g)
    # rotation taking g -> +y
    y = np.array([0, 1.0, 0])
    v = np.cross(g, y)
    s, c = np.linalg.norm(v), g @ y
    if s < 1e-9:
        return np.eye(3)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * ((1 - c) / s**2)


def _track_chain(imgs, i0, i1, max_corners=800):
    """KLT-track corners from frame i0 through every frame up to i1 (high-rate tracking keeps
    per-step motion small). Returns (pts_i0, pts_i1) of surviving tracks."""
    g = _gray(imgs[i0])
    p0 = cv2.goodFeaturesToTrack(g, max_corners, 0.005, 8)
    if p0 is None:
        return None, None
    start = p0.copy()
    cur = p0
    alive = np.ones(len(p0), bool)
    for k in range(i0 + 1, i1 + 1):
        g1 = _gray(imgs[k])
        nxt, st, _ = cv2.calcOpticalFlowPyrLK(g, g1, cur, None, winSize=(21, 21), maxLevel=3)
        back, stb, _ = cv2.calcOpticalFlowPyrLK(g1, g, nxt, None, winSize=(21, 21), maxLevel=3)
        ok = (st[:, 0] == 1) & (stb[:, 0] == 1) & (np.linalg.norm(back - cur, axis=2)[:, 0] < 1.0)
        alive &= ok
        cur, g = nxt, g1
    return start[alive, 0], cur[alive, 0]


def _pnp_from_tracks(p0, p1, d0, K):
    if p0 is None or len(p0) < 12:
        return None, 0
    u, v = p0[:, 0].astype(int), p0[:, 1].astype(int)
    z = d0[v.clip(0, d0.shape[0] - 1), u.clip(0, d0.shape[1] - 1)]
    m = (z > 0.2) & (z < 8)
    if m.sum() < 12:
        return None, int(m.sum())
    X = np.c_[(p0[m, 0] - K[0, 2]) / K[0, 0] * z[m], (p0[m, 1] - K[1, 2]) / K[1, 1] * z[m], z[m]]
    ok, rvec, tvec, inl = cv2.solvePnPRansac(X, p1[m], K, None, reprojectionError=2.0, iterationsCount=300,
                                             flags=cv2.SOLVEPNP_EPNP)
    if not ok or inl is None or len(inl) < 12:
        return None, 0 if inl is None else len(inl)
    rvec, tvec = cv2.solvePnPRefineLM(X[inl[:, 0]], p1[m][inl[:, 0]], K, None, rvec, tvec)
    R, _ = cv2.Rodrigues(rvec)
    T10 = np.eye(4)
    T10[:3, :3], T10[:3, 3] = R, tvec[:, 0]
    _pnp_from_tracks.last = (X[inl[:, 0]], p1[m][inl[:, 0]])
    return np.linalg.inv(T10), len(inl)


# ------------------------------------------------------------------ Manhattan rotation (drift-free)
_AXES = np.vstack([np.eye(3), -np.eye(3)])


def depth_normals(d, K, stride=4, max_depth=5.0):
    """Unit surface normals (camera coords) of a depth map, from a kNN plane fit on the back-projected cloud."""
    import open3d as o3d

    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(depth_cloud(d, K, stride, max_depth)))
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamKNN(20))
    return np.asarray(pcd.normals)


def manhattan_refine(R_pred, normals_cam, max_angle=15.0, max_corr=10.0, min_frac=0.03):
    """Snap a predicted camera->world rotation to the Manhattan frame (world axes = floor/wall normals).

    Each normal is assigned to the nearest world axis within `max_angle`; the rotation correction is the
    orthogonal Procrustes solution over those assignments. Needs two axis families (one family leaves a
    rotation about that axis free). Returns (R, correction_deg, families) - R_pred unchanged if unobservable.
    """
    R = R_pred.copy()
    fam = 0
    for _ in range(4):
        nw = normals_cam @ R.T
        cos = nw @ _AXES.T
        j = cos.argmax(1)
        ok = cos[np.arange(len(nw)), j] > np.cos(np.radians(max_angle))
        fams = np.bincount(j[ok] % 3, minlength=3) / max(len(nw), 1)
        fam = int((fams > min_frac).sum())
        if fam < 2:
            return R_pred, 0.0, fam
        A, B = nw[ok], _AXES[j[ok]]
        U, _, Vt = np.linalg.svd(B.T @ A)
        D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
        C = U @ D @ Vt
        R = C @ R
    corr = np.degrees(np.arccos(np.clip((np.trace(R @ R_pred.T) - 1) / 2, -1, 1)))
    if corr > max_corr:
        return R_pred, float(corr), -fam      # implausible jump: wrong axis assignment, keep prediction
    return R, float(corr), fam


_LSD = None


def image_lines(img, min_len_frac=0.03):
    """Interpretation-plane normals (camera coords, unit) and lengths of straight image segments (LSD)."""
    global _LSD
    if _LSD is None:
        _LSD = cv2.createLineSegmentDetector(0)
    g = _gray(img)
    lines = _LSD.detect(g)[0]
    if lines is None:
        return np.zeros((0, 3)), np.zeros(0)
    L = lines.reshape(-1, 4)
    ln = np.hypot(L[:, 2] - L[:, 0], L[:, 3] - L[:, 1])
    keep = ln > min_len_frac * max(g.shape)
    return L[keep], ln[keep]


def manhattan_refine_lines(R_pred, segs, lens, K, max_corr=12.0, min_lines=4):
    """Correct a predicted camera->world rotation so straight image lines align with the world axes.

    A 3-D line with direction d (camera coords) projects to an image line whose interpretation-plane normal
    n = K^T l satisfies n.d = 0. Lines are assigned to the closest predicted axis d_a = R^T e_a and a small
    rotation correction is solved by Gauss-Newton on sum (n.d_a)^2 (length-weighted, Huber). Uses only the
    focal length - not the mono-depth, whose geometry is not square enough to define the frame (D22).
    Returns (R, correction_deg, n_families); R_pred unchanged when fewer than two axis families are seen.
    """
    if len(segs) < 2 * min_lines:
        return R_pred, 0.0, 0
    a = np.c_[segs[:, :2], np.ones(len(segs))]
    b = np.c_[segs[:, 2:], np.ones(len(segs))]
    l = np.cross(a, b)
    n = l @ K                                     # K^T l (row form)
    n /= np.linalg.norm(n, axis=1, keepdims=True)
    w0 = lens / lens.max()
    R = R_pred.copy()
    fam = 0
    for it, thr in enumerate((np.sin(np.radians(4)), np.sin(np.radians(3)), np.sin(np.radians(2)),
                              np.sin(np.radians(2)), np.sin(np.radians(2)))):
        D = R.T                                   # columns: world axes in camera coords
        res = n @ D                               # (N,3) n . d_a
        j = np.abs(res).argmin(1)
        r = res[np.arange(len(n)), j]
        ok = np.abs(r) < thr
        cnt = np.bincount(j[ok], minlength=3)
        fam = int((cnt >= min_lines).sum())
        if fam < 2:
            return R_pred, 0.0, fam
        da = D[:, j[ok]].T
        J = np.cross(da, n[ok])                   # d(n.d)/d omega for d -> d + omega x d
        w = w0[ok] * np.minimum(1.0, (0.5 * thr) / np.maximum(np.abs(r[ok]), 1e-9))
        omega = np.linalg.lstsq(J * w[:, None], -r[ok] * w, rcond=None)[0]
        th = np.linalg.norm(omega)
        if th > 1e-12:
            kx = np.array([[0, -omega[2], omega[1]], [omega[2], 0, -omega[0]], [-omega[1], omega[0], 0]]) / th
            Delta = np.eye(3) + np.sin(th) * kx + (1 - np.cos(th)) * kx @ kx
            R = R @ Delta.T                       # new axes in camera: Delta @ d_a
    corr = np.degrees(np.arccos(np.clip((np.trace(R @ R_pred.T) - 1) / 2, -1, 1)))
    if corr > max_corr:
        return R_pred, float(corr), -fam
    return R, float(corr), fam


def _rodrigues(axis, th):
    axis = axis / np.linalg.norm(axis)
    kx = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(th) * kx + (1 - np.cos(th)) * kx @ kx


def manhattan_absolute(R_pred, segs, lens, K, up_tol=20.0, yaw_tol=30.0, min_lines=4):
    """Wide-basin version of manhattan_refine_lines: vertical vanishing direction by RANSAC near the
    predicted up, then yaw by scanning +-45 deg for the most horizontal lines on the two wall axes, then
    the fine Gauss-Newton refinement. Recovers from odometry rotation errors of up to ~20-30 deg."""
    if len(segs) < 2 * min_lines:
        return R_pred, 0.0, 0
    a = np.c_[segs[:, :2], np.ones(len(segs))]
    b = np.c_[segs[:, 2:], np.ones(len(segs))]
    n = np.cross(a, b) @ K
    n /= np.linalg.norm(n, axis=1, keepdims=True)
    w = lens / lens.max()
    up = R_pred.T[:, 1]                                       # world +y in camera coords
    cand = np.nonzero(np.abs(n @ up) < np.sin(np.radians(up_tol)))[0]
    if len(cand) < min_lines:
        return R_pred, 0.0, 0
    rng = np.random.default_rng(len(segs))
    best, best_s = None, 0.0
    t2 = np.sin(np.radians(1.5))
    for _ in range(150):
        i, j = rng.choice(cand, 2, replace=False)
        v = np.cross(n[i], n[j])
        if np.linalg.norm(v) < 1e-6:
            continue
        v /= np.linalg.norm(v)
        v *= np.sign(v @ up)
        if v @ up < np.cos(np.radians(up_tol)):
            continue
        inl = np.abs(n[cand] @ v) < t2
        sc = w[cand][inl].sum()
        if inl.sum() >= min_lines and sc > best_s:
            best, best_s = v, sc
    if best is None:
        return R_pred, 0.0, 0
    inl = cand[np.abs(n[cand] @ best) < t2]
    if len(inl) < 5 or w[inl].sum() < 1.5:
        return R_pred, 0.0, 0                                 # too little vertical structure to trust
    v = np.linalg.svd(n[inl] * w[inl, None])[2][-1]
    v *= np.sign(v @ up)
    # tilt R_pred so its up axis is v (minimal rotation, camera side)
    ax = np.cross(up, v)
    R = R_pred if np.linalg.norm(ax) < 1e-9 else R_pred @ _rodrigues(ax, np.arcsin(min(1.0, np.linalg.norm(ax)))).T
    # yaw scan about the world vertical
    th = np.sin(np.radians(1.5))
    scores = []
    for yd in np.arange(-45.0, 45.0, 0.5):
        Ry = _rodrigues(np.array([0, 1.0, 0]), np.radians(yd))
        D = (Ry @ R).T
        rr = np.abs(n @ D[:, [0, 2]])
        r = rr.min(1)
        on = r < th
        both = int(min((on & (rr[:, 0] < rr[:, 1])).sum(), (on & (rr[:, 1] <= rr[:, 0])).sum()))
        scores.append((w[on].sum(), int(on.sum()), both, -abs(yd), yd))
    sc, cnt, both, _, yd = max(scores)
    # a door leaf (or any single rotated object) supplies only one horizontal direction: large yaw
    # corrections need line support on both wall axes, otherwise only small corrections are trusted
    if cnt < min_lines or abs(yd) > yaw_tol or (both < 3 and abs(yd) > 8.0):
        return R_pred, 0.0, 1
    R = _rodrigues(np.array([0, 1.0, 0]), np.radians(yd)) @ R
    R2, _, fam = manhattan_refine_lines(R, segs, lens, K, max_corr=5.0, min_lines=min_lines)
    if fam >= 2:
        R = R2
    corr = np.degrees(np.arccos(np.clip((np.trace(R @ R_pred.T) - 1) / 2, -1, 1)))
    return R, float(corr), 2

def gap_rotation(video, t0, t1, width, K, max_frames=60):
    """Rotation across a fast turn that broke 10 fps tracking: decode [t0, t1] at the native frame rate and
    chain frame-to-frame rotations (homography of a pure rotation, R = K^-1 H K). Returns R_rel (camera at
    t1 expressed in camera at t0) or None."""
    cmd = ["ffmpeg", "-v", "error", "-ss", f"{max(t0, 0):.3f}", "-i", str(video), "-t", f"{t1 - t0:.3f}",
           "-vf", f"scale={width}:-2", "-f", "rawvideo", "-pix_fmt", "gray", "pipe:1"]
    raw = subprocess.run(cmd, capture_output=True).stdout
    w = width
    h = int(round(2 * K[1, 2]))
    n = len(raw) // (w * h)
    if n < 2 or n > max_frames:
        return None
    frames = np.frombuffer(raw[: n * w * h], np.uint8).reshape(n, h, w)
    Ki = np.linalg.inv(K)
    R10 = np.eye(3)
    for a, b in zip(frames[:-1], frames[1:]):
        p0 = cv2.goodFeaturesToTrack(a, 400, 0.005, 8)
        if p0 is None or len(p0) < 15:
            return None
        p1, st, _ = cv2.calcOpticalFlowPyrLK(a, b, p0, None, winSize=(21, 21), maxLevel=4)
        pb, stb, _ = cv2.calcOpticalFlowPyrLK(b, a, p1, None, winSize=(21, 21), maxLevel=4)
        ok = (st[:, 0] == 1) & (stb[:, 0] == 1) & (np.linalg.norm(pb - p0, axis=2)[:, 0] < 1.0)
        if ok.sum() < 15:
            return None
        H, mask = cv2.findHomography(p0[ok], p1[ok], cv2.RANSAC, 2.0)
        if H is None or mask.sum() < 12:
            return None
        Rh = Ki @ H @ K
        U, S, Vt = np.linalg.svd(Rh)
        Rh = U @ Vt
        if np.linalg.det(Rh) < 0:
            return None
        R10 = Rh @ R10                                  # maps camera-t0 coords to camera-t1 coords
    return R10.T


def floor_up_agrees(R, normals_cam, tol=10.0, search=30.0, min_frac=0.05):
    """Veto for a Manhattan snap: if the depth map sees a horizontal surface, its normal must agree with the
    snapped vertical within `tol` (mono-depth normals are biased by a few degrees, never by 20+)."""
    up = R.T[:, 1]
    c = normals_cam @ up
    sel = np.abs(c) > np.cos(np.radians(search))
    if sel.mean() < min_frac:
        return True                                     # no floor/ceiling in view: nothing to check
    m = (normals_cam[sel] * np.sign(c[sel])[:, None]).mean(0)
    m /= np.linalg.norm(m)
    return bool(np.degrees(np.arccos(np.clip(m @ up, -1, 1))) < tol)

def translation_given_rotation(X, p1, K, R10, iters=5):
    """t minimising reprojection of camera-0 points X into frame 1 with rotation R10 fixed (linear, IRLS)."""
    x = np.c_[(p1[:, 0] - K[0, 2]) / K[0, 0], (p1[:, 1] - K[1, 2]) / K[1, 1], np.ones(len(p1))]
    RX = X @ R10.T
    # x × (RX + t) = 0  ->  [x]x t = -x × RX  (two independent rows per point)
    A = np.stack([np.c_[np.zeros(len(x)), -x[:, 2], x[:, 1]], np.c_[x[:, 2], np.zeros(len(x)), -x[:, 0]]], 1)
    b = -np.cross(x, RX)[:, :2]
    w = np.ones(len(x))
    t = np.zeros(3)
    for _ in range(iters):
        Aw = (A * w[:, None, None]).reshape(-1, 3)
        bw = (b * w[:, None]).reshape(-1)
        t = np.linalg.lstsq(Aw, bw, rcond=None)[0]
        r = np.linalg.norm((A @ t) - b, axis=1) / np.maximum(RX[:, 2] + t[2], 0.1)
        s = max(np.median(r) * 1.5, 1e-4)
        w = np.where(r < s, 1.0, s / np.maximum(r, 1e-12))   # Huber
    return t


def build_scene(video, work: Path, fps=10.0, depth_every=3, depth_source=None, log=print) -> Scene:
    """fps: tracking rate; depth runs on every `depth_every`-th tracked frame (keyframes)."""
    from ..geometry.drift import correct_drift
    from ..geometry.fusion import select_keyframes
    from ..models.depth import MonoDepth

    video = Path(video)
    if video.is_dir():
        vids = [f for f in video.iterdir() if f.suffix.lower() in {".mp4", ".mov", ".m4v"}]
        video = vids[0]
    work = Path(work)
    cv2.setRNGSeed(0)                       # RANSAC (PnP, homography) repeatable run to run
    files = extract_frames(video, work / "frames", fps=fps)
    # greyscale in memory (tracking, lines); colour is read from disk for the depth keyframes only - a 3.5 min
    # walkthrough at 10 fps would otherwise need ~5 GB
    imgs = [cv2.imread(str(f), cv2.IMREAD_GRAYSCALE) for f in files]
    h, w = imgs[0].shape[:2]
    # keyframes: every `depth_every`-th frame, nudged to the sharpest frame in its window
    sh = np.array([sharpness(i) for i in imgs])
    kfi = []
    for s0 in range(0, len(imgs), depth_every):
        win = range(s0, min(s0 + depth_every, len(imgs)))
        kfi.append(max(win, key=lambda i: sh[i]))
    f0 = FOCAL_PRIOR * max(h, w)
    f_vp, f_spread = focal_from_vanishing_points([imgs[i] for i in kfi[:: max(1, len(kfi) // 40)]], f0)
    f = f_vp
    K = np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1.0]])
    log(f"  video: {len(imgs)} frames tracked at {fps} fps, {len(kfi)} depth keyframes, {w}x{h}, "
        f"focal prior {f0:.0f}px, VP focal {f_vp:.0f}px")
    depth_model = depth_source or MonoDepth(cache_dir=work.parent.parent / ".cache" / "depth")
    from ..models.depth import depth_scale

    DEPTH_SCALE = depth_scale()
    log(f"  depth scale correction {DEPTH_SCALE:.3f}")
    depths = []
    for k, i in enumerate(kfi):
        if hasattr(depth_model, "set_index"):
            depth_model.set_index(i)
        rgb = cv2.cvtColor(cv2.imread(str(files[i])), cv2.COLOR_BGR2RGB)
        depths.append((depth_model.predict(rgb) * DEPTH_SCALE).astype(np.float16))
        if k % 50 == 0:
            log(f"  depth {k}/{len(kfi)}")
    poses = [np.eye(4)]
    n_pnp = n_icp = n_cv = 0
    n_mw = 0
    n_gap = 0
    n_veto = 0
    mw_corr = []
    last = np.eye(4)
    if MANHATTAN_ROTATION:
        # absolute rotation: world = gravity + Manhattan frame of the first keyframe (upright video)
        from .photo_reg import gravity_manhattan

        R0, _ = gravity_manhattan(depth_cloud(depths[0], K), depth_normals(depths[0], K))
        R0, _, _ = manhattan_absolute(R0, *image_lines(imgs[kfi[0]]), K, yaw_tol=45.0)
        poses[0][:3, :3] = R0                            # gravity_manhattan: p_world = R0 @ p_cam
    trusted = [True]
    for k in range(1, len(kfi)):
        p0, p1 = _track_chain(imgs, kfi[k - 1], kfi[k])
        _pnp_from_tracks.last = None
        step_ok = False
        T, ninl = _pnp_from_tracks(p0, p1, depths[k - 1], K)
        corr = _pnp_from_tracks.last
        if T is not None and np.linalg.norm(T[:3, 3]) < 0.6:
            n_pnp += 1
            step_ok = True
        else:
            corr = None
            # fast turn / blur broke 10 fps tracking: rotation from a native-rate decode of the gap, then
            # depth ICP seeded with it for translation; constant velocity only as the last resort
            Rg = gap_rotation(video, kfi[k - 1] / fps, kfi[k] / fps, w, K) if GAP_BRIDGING else None
            init = last.copy()
            if Rg is not None:
                init[:3, :3] = Rg
                n_gap += 1
            T, fit = _icp_step(depth_cloud(depths[k - 1], K), depth_cloud(depths[k], K), init=init)
            rot_ok = Rg is None or np.degrees(np.arccos(np.clip((np.trace(T[:3, :3] @ Rg.T) - 1) / 2, -1, 1))) < 5
            if fit > 0.3 and np.linalg.norm(T[:3, 3]) < 0.4 and rot_ok:
                n_icp += 1
            else:
                T = init
                n_cv += 1
        last = T
        P = poses[-1] @ T
        fam = 0
        if MANHATTAN_ROTATION:
            R_new, c, fam = manhattan_absolute(P[:3, :3], *image_lines(imgs[kfi[k]]), K)
            if fam >= 2 and FLOOR_VETO_DEG > 0 and not floor_up_agrees(R_new, depth_normals(depths[k], K),
                                                                       tol=FLOOR_VETO_DEG):
                fam, n_veto = -2, n_veto + 1
            if fam >= 2:
                n_mw += 1
                mw_corr.append(c)
                R_rel = poses[-1][:3, :3].T @ R_new          # camera k orientation in camera k-1
                t_rel = T[:3, 3]
                if corr is not None:                         # re-solve translation with the corrected rotation
                    R10 = R_rel.T
                    t10 = translation_given_rotation(corr[0], corr[1], K, R10)
                    t_rel = -R10.T @ t10
                    if np.linalg.norm(t_rel) > 0.6:
                        t_rel = T[:3, 3]
                P = np.eye(4)
                P[:3, :3] = R_new
                P[:3, 3] = poses[-1][:3, 3] + poses[-1][:3, :3] @ t_rel
        poses.append(P)
        # orientation is trusted when measured (Manhattan snap) or carried by accurate tracking from a trusted
        # frame; frames after a guessed step stay untrusted until the next snap and are not fused (D22)
        trusted.append(bool(fam >= 2 or (step_ok and trusted[-1])) if (MANHATTAN_ROTATION and TRUSTED_ONLY) else True)
    poses = np.array(poses)
    trusted = np.array(trusted)
    if MANHATTAN_ROTATION:
        log(f"  manhattan rotation: {n_mw}/{len(kfi) - 1} keyframes snapped, "
            f"median correction {np.median(mw_corr) if mw_corr else 0:.2f} deg, {n_veto} vetoed by the floor normal")
    odo_raw = poses.copy()
    files = [files[i] for i in kfi]
    imgs = [imgs[i] for i in kfi]
    log(f"  odometry: {n_pnp} PnP steps, {n_icp} ICP fallbacks, {n_cv} constant-velocity bridges, "
        f"{n_gap} fast turns bridged at native frame rate")

    def cloud_fn(i):
        return depth_cloud(depths[i], K, stride=6, max_depth=4.0)

    kf = [i for i in select_keyframes(poses, 0.10, 8, 4) if trusted[i]]
    log(f"  {int(trusted.sum())}/{len(trusted)} keyframes with trusted orientation; {len(kf)} fused")
    poses, drift_info = correct_drift(poses, kf, cloud_fn, frag_len=6, voxel=0.06, max_pair_dist=3.0,
                                      icp_rmse=0.04, fit_min=0.3, max_jump=0.6, log=log)

    def all_pts():
        return np.concatenate([depth_cloud(depths[i], K, 8, 4.0) @ poses[i][:3, :3].T + poses[i][:3, 3]
                               for i in kf])

    Rg = gravity_align(poses, all_pts)
    G = np.eye(4)
    G[:3, :3] = Rg
    poses = np.einsum("ij,njk->nik", G, poses)

    import open3d as o3d

    pts, views = [], []
    for i in kf:
        p = depth_cloud(depths[i], K, stride=3, max_depth=5.0)
        pw = p @ poses[i][:3, :3].T + poses[i][:3, 3]
        pts.append(pw)
        views.append((poses[i][:3, 3], pw[::2]))
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.concatenate(pts))).voxel_down_sample(0.03)
    pcd, _ = pcd.remove_statistical_outlier(20, 2.0)
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.12, max_nn=30))
    from ..geometry.fusion import orient_to_nearest

    _cams = np.array([v[0] for v in views])
    pcd.normals = o3d.utility.Vector3dVector(orient_to_nearest(np.asarray(pcd.points), np.asarray(pcd.normals).copy(), _cams))
    path_len = float(np.linalg.norm(np.diff(poses[:, :3, 3], axis=0), axis=1).sum())
    # trajectory for diagnostics (bench/video_odometry.py): keyframe times at `fps`, raw odometry and final poses
    np.savez_compressed(work / "trajectory.npz", t=np.asarray(kfi) / fps, raw=odo_raw, final=poses, K=K,
                        trusted=trusted)
    images = [(i, str(files[i]), K, poses[i]) for i in kf]
    return Scene(tier="video", xyz=np.asarray(pcd.points), normals=np.asarray(pcd.normals),
                 cam_centres=poses[kf, :3, 3], views=views, path_length_m=path_len, error_model=VIDEO_ERRORS,
                 images=images,
                 meta=dict(capture=str(video), frames_tracked=len(sh), depth_keyframes=len(imgs), keyframes=len(kf),
                           fps=fps,
                           focal_px=round(f, 1), focal_prior_px=round(f0, 1),
                           odometry=dict(pnp=n_pnp, icp=n_icp, constant_velocity=n_cv, manhattan_snapped=n_mw, fast_turns_bridged=n_gap), drift=drift_info,
                           depth_model="Depth-Anything-V2-Metric-Indoor-Small", depth_scale=DEPTH_SCALE))
