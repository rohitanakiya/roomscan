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

VIDEO_ERRORS = ErrorModel(sensor_face=0.012, scale_rel=0.020, drift_per_m=0.0020,
                          ceiling_plane=0.015, unobserved_face=0.08, opening_jamb=0.02)
FOCAL_PRIOR = 0.83          # f / long-side (px); iPhone main camera, video mode with stabilisation crop
DEPTH_SCALE = 1.0           # calibrated multiplicative correction for the mono-depth model (bench/calibrate)
WORK_W = 640


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
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(g, cv2.CV_64F).var())


def focal_from_vanishing_points(images, f0):
    """Median focal over frames from pairs of orthogonal vanishing points (Manhattan world)."""
    est = []
    lsd = cv2.createLineSegmentDetector(0)
    for img in images:
        g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
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
    g0 = cv2.cvtColor(img0, cv2.COLOR_BGR2GRAY)
    g1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
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


def build_scene(video, work: Path, fps=3.0, depth_source=None, log=print) -> Scene:
    from ..geometry.drift import correct_drift
    from ..geometry.fusion import select_keyframes
    from ..models.depth import MonoDepth

    video = Path(video)
    if video.is_dir():
        vids = [f for f in video.iterdir() if f.suffix.lower() in {".mp4", ".mov", ".m4v"}]
        video = vids[0]
    work = Path(work)
    files = extract_frames(video, work / "frames", fps=fps)
    imgs = [cv2.imread(str(f)) for f in files]
    sh = np.array([sharpness(i) for i in imgs])
    keep = sh > 0.25 * np.median(sh)
    files = [f for f, k in zip(files, keep) if k]
    imgs = [i for i, k in zip(imgs, keep) if k]
    h, w = imgs[0].shape[:2]
    f0 = FOCAL_PRIOR * max(h, w)
    f_vp, f_spread = focal_from_vanishing_points(imgs[:: max(1, len(imgs) // 40)], f0)
    f = f_vp
    K = np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1.0]])
    log(f"  video: {len(imgs)} frames ({fps} fps, {sum(~keep)} blurred dropped), {w}x{h}, focal prior {f0:.0f}px, "
        f"VP focal {f_vp:.0f}px")

    depth_model = depth_source or MonoDepth(cache_dir=work.parent.parent / ".cache" / "depth")
    depths = []
    for k, im in enumerate(imgs):
        d = depth_model.predict(cv2.cvtColor(im, cv2.COLOR_BGR2RGB)) * DEPTH_SCALE
        depths.append(d)
        if k % 50 == 0:
            log(f"  depth {k}/{len(imgs)}")

    poses = [np.eye(4)]
    n_pnp = n_icp = 0
    for k in range(1, len(imgs)):
        T, ninl = _pnp_step(imgs[k - 1], imgs[k], depths[k - 1], K)
        if T is None or np.linalg.norm(T[:3, 3]) > 0.8:
            T, fit = _icp_step(depth_cloud(depths[k - 1], K), depth_cloud(depths[k], K))
            n_icp += 1
        else:
            n_pnp += 1
        poses.append(poses[-1] @ T)
    poses = np.array(poses)
    log(f"  odometry: {n_pnp} PnP steps, {n_icp} ICP fallbacks")

    def cloud_fn(i):
        return depth_cloud(depths[i], K, stride=6, max_depth=4.0)

    kf = select_keyframes(poses, 0.15, 10, 6)
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
    path_len = float(np.linalg.norm(np.diff(poses[:, :3, 3], axis=0), axis=1).sum())
    images = [(i, str(files[i]), K, poses[i]) for i in kf]
    return Scene(tier="video", xyz=np.asarray(pcd.points), normals=np.asarray(pcd.normals),
                 cam_centres=poses[kf, :3, 3], views=views, path_length_m=path_len, error_model=VIDEO_ERRORS,
                 images=images,
                 meta=dict(capture=str(video), frames=len(imgs), keyframes=len(kf), fps=fps,
                           focal_px=round(f, 1), focal_prior_px=round(f0, 1),
                           odometry=dict(pnp=n_pnp, icp=n_icp), drift=drift_info,
                           depth_model="Depth-Anything-V2-Metric-Indoor-Small", depth_scale=DEPTH_SCALE))
