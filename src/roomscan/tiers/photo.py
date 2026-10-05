"""Photo tier: per-room folders of 2-8 stills, no depth, no poses.

  1. intrinsics: EXIF 35 mm-equivalent focal if present, else Manhattan vanishing points, else prior
  2. metric depth per photo (same model as the video tier)
  3. all-pairs SIFT matching; matches lifted to 3D with each photo's depth; relative poses from
     3D-2D PnP-RANSAC (metric, because depth is metric) -> maximum-spanning-tree pose graph
  4. photos that link two rooms (the protocol's doorway shots) join the rooms into one frame;
     that is the whole-property stitch
  5. rooms still unconnected are placed by door matching: a door of equal width (±8 cm) on a free
     wall of the stitched block, without overlap — flagged in the JSON as `placement: inferred`
  6. gravity from the photo up-vector, refined by the floor normal
Everything after the Scene (layout, openings, JSON, plan) is the shared pipeline.
"""
from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

from ..geometry.room import ErrorModel
from ..scene import Scene
from .video import FOCAL_PRIOR, depth_cloud, focal_from_vanishing_points, gravity_align

# scale from the leave-one-out depth calibration (D24); registration error is not modelled - a failed stitch is
# flagged in the JSON instead of being hidden inside a wide interval
PHOTO_ERRORS = ErrorModel(sensor_face=0.040, scale_rel=0.050, drift_per_m=0.004,
                          ceiling_plane=0.04, unobserved_face=0.20, opening_jamb=0.05)
IMG_EXT = {".jpg", ".jpeg", ".png", ".heic"}
WORK_LONG = 960


def _exif_focal(path, long_side):
    try:
        from PIL import Image

        ex = Image.open(path).getexif()
        sub = ex.get_ifd(0x8769)
        f35 = sub.get(0xA405)
        if f35:
            return float(f35) / 36.0 * long_side
    except Exception:
        pass
    return None


def load_folders(root: Path):
    rooms = []
    for d in sorted(p for p in root.iterdir() if p.is_dir()):
        files = sorted(f for f in d.iterdir() if f.suffix.lower() in IMG_EXT)
        if files:
            rooms.append((d.name, files))
    return rooms


def _sift():
    return cv2.SIFT_create(nfeatures=4000, contrastThreshold=0.02)


def _rel_pose(kp_a, des_a, kp_b, des_b, depth_a, K_a, K_b, ratio=0.8):
    """Pose of camera b in camera-a frame, from a's depth + b's 2D. Returns (T_ab, inliers)."""
    if des_a is None or des_b is None or len(kp_a) < 8 or len(kp_b) < 8:
        return None, 0
    m = cv2.BFMatcher(cv2.NORM_L2).knnMatch(des_a, des_b, k=2)
    good = [x[0] for x in m if len(x) == 2 and x[0].distance < ratio * x[1].distance]
    if len(good) < 12:
        return None, len(good)
    pa = np.float32([kp_a[g.queryIdx].pt for g in good])
    pb = np.float32([kp_b[g.trainIdx].pt for g in good])
    z = depth_a[pa[:, 1].astype(int).clip(0, depth_a.shape[0] - 1), pa[:, 0].astype(int).clip(0, depth_a.shape[1] - 1)]
    ok = (z > 0.2) & (z < 8)
    if ok.sum() < 10:
        return None, 0
    X = np.c_[(pa[ok, 0] - K_a[0, 2]) / K_a[0, 0] * z[ok], (pa[ok, 1] - K_a[1, 2]) / K_a[1, 1] * z[ok], z[ok]]
    r = cv2.solvePnPRansac(X, pb[ok], K_b, None, reprojectionError=4.0, iterationsCount=500,
                           flags=cv2.SOLVEPNP_EPNP)
    okp, rvec, tvec, inl = r
    if not okp or inl is None or len(inl) < 12:
        return None, 0 if inl is None else len(inl)
    rvec, tvec = cv2.solvePnPRefineLM(X[inl[:, 0]], pb[ok][inl[:, 0]], K_b, None, rvec, tvec)
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, tvec[:, 0]
    return np.linalg.inv(T), int(len(inl))


def build_scene(root, work: Path, depth_source=None, log=print) -> Scene:
    from ..models.depth import MonoDepth

    root, work = Path(root), Path(work)
    folders = load_folders(root)
    imgs, names, room_of, Ks = [], [], [], []
    for ri, (rname, files) in enumerate(folders):
        for f in files:
            im = cv2.imread(str(f))
            if im is None:
                continue
            s = WORK_LONG / max(im.shape[:2])
            im = cv2.resize(im, (int(im.shape[1] * s), int(im.shape[0] * s)), interpolation=cv2.INTER_AREA)
            imgs.append(im)
            names.append(str(f))
            room_of.append(ri)
            fe = _exif_focal(f, WORK_LONG)
            Ks.append(fe)
    room_of = np.array(room_of)
    h0, w0 = imgs[0].shape[:2]
    f0 = FOCAL_PRIOR * WORK_LONG
    f_vp, _ = focal_from_vanishing_points(imgs, f0)
    for i, im in enumerate(imgs):
        h, w = im.shape[:2]
        f = Ks[i] or f_vp
        Ks[i] = np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1.0]])
    log(f"  photo: {len(imgs)} photos in {len(folders)} room folders; focal {'EXIF' if any(_exif_focal(n, 1) for n in names) else 'VP'} "
        f"{f_vp:.0f}px")
    depth_model = depth_source or MonoDepth(cache_dir=work.parent.parent / ".cache" / "depth")
    depths = [depth_model.predict(cv2.cvtColor(im, cv2.COLOR_BGR2RGB)) for im in imgs]
    from ..models.depth import depth_scale

    DEPTH_SCALE = depth_scale()

    depths = [d * DEPTH_SCALE for d in depths]
    import open3d as o3d

    from .photo_reg import gravity_manhattan, register_blocks, register_set

    # per photo: metric cloud -> gravity + Manhattan frame, floor at y = 0
    n = len(imgs)
    base = []
    for i in range(n):
        p = depth_cloud(depths[i], Ks[i], 3, 5.0)
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(p))
        pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.12, max_nn=30))
        pc.orient_normals_towards_camera_location(np.zeros(3))
        Rg, fy = gravity_manhattan(p, np.asarray(pc.normals))
        B = np.eye(4)
        B[:3, :3] = Rg
        B[1, 3] = -fy
        base.append(B)
    poses = [None] * n
    room_blocks, room_scores = [], {}
    for ri, (rname, _) in enumerate(folders):
        idx = [i for i in range(n) if room_of[i] == ri]
        clouds = [(depth_cloud(depths[i], Ks[i], 4, 5.0) @ base[i][:3, :3].T + base[i][:3, 3]) for i in idx]
        Ts, sc = register_set(clouds)
        for i, T, s_ in zip(idx, Ts, sc):
            if T is not None:
                poses[i] = T @ base[i]
        room_scores[rname] = dict(placed=sum(T is not None for T in Ts), photos=len(idx),
                                  mean_ncc=round(float(np.mean([x for x, T in zip(sc, Ts) if T is not None] or [0])), 3))
        placed = [i for i in idx if poses[i] is not None]
        room_blocks.append(np.concatenate([depth_cloud(depths[i], Ks[i], 4, 5.0) @ poses[i][:3, :3].T + poses[i][:3, 3]
                                           for i in placed]) if placed else np.zeros((0, 3)))
    nonempty = [k for k, b in enumerate(room_blocks) if len(b)]
    Tb, sb = register_blocks([room_blocks[k] for k in nonempty])
    inferred = []
    for k, T, s_ in zip(nonempty, Tb, sb):
        for i in range(n):
            if room_of[i] == k and poses[i] is not None:
                poses[i] = T @ poses[i]
        if s_ == 0.0:
            inferred.append(folders[k][0])
    log(f"  photo registration: {room_scores}; rooms joined by correlation: "
        f"{len(nonempty) - len(inferred)}/{len(nonempty)}")
    order = [i for i in range(n) if poses[i] is not None]
    allp, views, cams = [], [], []
    for i in order:
        pw = depth_cloud(depths[i], Ks[i], 3, 5.0) @ poses[i][:3, :3].T + poses[i][:3, 3]
        allp.append(pw)
        views.append((poses[i][:3, 3], pw[::2]))
        cams.append(poses[i][:3, 3])
    poses = np.array([poses[i] if poses[i] is not None else np.eye(4) for i in range(n)])
    comps = [order]
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.concatenate(allp))).voxel_down_sample(0.03)
    pcd, _ = pcd.remove_statistical_outlier(20, 2.0)
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.12, max_nn=30))
    from ..geometry.fusion import orient_to_nearest

    _cams = np.array([v[0] for v in views])
    pcd.normals = o3d.utility.Vector3dVector(orient_to_nearest(np.asarray(pcd.points), np.asarray(pcd.normals).copy(), _cams))
    groups = []
    for ri, (rname, _) in enumerate(folders):
        idx = [order.index(i) for i in range(n) if room_of[i] == ri and i in order]
        if idx:
            groups.append((rname, idx))
    images = [dict(img=names[i], K=Ks[i], T=poses[i], depth=(lambda shape, d=depths[i]: cv2.resize(d, (shape[1], shape[0]))))
              for i in order]
    scene = Scene(tier="photo", xyz=np.asarray(pcd.points), normals=np.asarray(pcd.normals),
                  cam_centres=np.array(cams), views=views, path_length_m=0.0, error_model=PHOTO_ERRORS,
                  groups=groups,
                  meta=dict(capture=str(root), photos=n, photos_placed=len(order), rooms=len(folders),
                            room_registration=room_scores, rooms_placed_by_inference=inferred,
                            focal_px=round(float(f_vp), 1),
                            stitch="top-view correlation" + (" (some rooms inferred)" if inferred else ""),
                            depth_model="Depth-Anything-V2-Metric-Indoor-Small", damage_views=images))
    return scene


def segment_by_groups(scene, F, R, P):
    """Room labels from folder membership: a free cell belongs to the room folder whose photos
    saw it most (visibility fans per folder)."""
    from ..geometry.layout import visibility_raster

    G = R.grid
    votes = []
    for rname, idx in scene.groups:
        votes.append(visibility_raster([scene.views[i] for i in idx], F, G).astype(np.float32))
    V = np.stack(votes)
    lab = V.argmax(0) + 1
    lab[(V.max(0) == 0) | ~R.free] = 0
    lab[R.wall] = 0
    stats = []
    for k, (rname, idx) in enumerate(scene.groups, start=1):
        a = (lab == k).sum() * G.res ** 2
        if a > 0.5:
            stats.append(dict(label=k, raster_area_m2=float(a), wall_support=1.0, name=rname))
    return lab, stats
