"""Back-project depth frames into a gravity-aligned world point cloud.

World convention used everywhere downstream (after `to_plan_frame`):
    x, z  horizontal plan axes (metres), y up (gravity, from ARKit / IMU).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class PointCloud:
    xyz: np.ndarray          # (N,3) world
    normals: np.ndarray | None = None
    frame: np.ndarray | None = None  # (N,) source frame index (for ray tests)

    def __len__(self):
        return len(self.xyz)


def backproject(depth: np.ndarray, K: np.ndarray, stride: int = 1, max_depth: float = 5.0):
    """Return (M,3) camera-frame points (OpenCV convention) and (M,2) pixel coords."""
    h, w = depth.shape
    v, u = np.mgrid[0:h:stride, 0:w:stride]
    d = depth[::stride, ::stride]
    m = (d > 0.05) & (d < max_depth)
    u, v, d = u[m].astype(np.float32), v[m].astype(np.float32), d[m]
    x = (u - K[0, 2]) / K[0, 0] * d
    y = (v - K[1, 2]) / K[1, 1] * d
    return np.stack([x, y, d], 1), np.stack([u, v], 1)


def select_keyframes(poses: np.ndarray, min_trans=0.05, min_rot_deg=5.0, max_gap=30):
    """Greedy keyframe selection on motion; keeps redundancy low on slow scans."""
    keep = [0]
    last = poses[0]
    cos_thr = np.cos(np.radians(min_rot_deg))
    for i in range(1, len(poses)):
        dt = np.linalg.norm(poses[i, :3, 3] - last[:3, 3])
        fwd_a, fwd_b = last[:3, 2], poses[i, :3, 2]
        if dt > min_trans or fwd_a @ fwd_b < cos_thr or i - keep[-1] >= max_gap:
            keep.append(i)
            last = poses[i]
    return np.array(keep)


def fuse(capture, frame_idx, poses=None, stride=2, max_depth=4.5, min_conf=2, voxel=0.02):
    """Fuse selected frames into one voxel-downsampled cloud with normals."""
    import open3d as o3d

    poses = capture.poses if poses is None else poses
    K = capture.K_depth
    pts, src = [], []
    for i in frame_idx:
        d = capture.load_depth(int(capture.frame_ids[i]), min_conf=min_conf)
        if d is None:
            continue
        pc, _ = backproject(d, K, stride=stride, max_depth=max_depth)
        T = poses[i]
        pw = pc @ T[:3, :3].T + T[:3, 3]
        pts.append(pw)
        src.append(np.full(len(pw), i, np.int32))
    xyz = np.concatenate(pts)
    srcs = np.concatenate(src)
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    pcd, _, idx = pcd.voxel_down_sample_and_trace(voxel, pcd.get_min_bound(), pcd.get_max_bound())
    keep_src = np.array([srcs[ix[0]] for ix in idx]) if len(idx) else None
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 4, max_nn=30))
    return PointCloud(np.asarray(pcd.points), np.asarray(pcd.normals), keep_src)
