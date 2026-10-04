"""LiDAR tier: Stray Scanner depth + ARKit poses + intrinsics."""
from __future__ import annotations

import numpy as np

from ..geometry.fusion import backproject, fuse, select_keyframes
from ..geometry.room import ErrorModel
from ..io.stray import load_stray
from ..scene import Scene

LIDAR_ERRORS = ErrorModel(sensor_face=0.004, scale_rel=0.003, drift_per_m=0.0004,
                          ceiling_plane=0.004, unobserved_face=0.05, opening_jamb=0.006)


def _load_calibration():
    """Terms fitted by bench/calibrate.py on repeat captures (leave-one-pair-out validated)."""
    import json
    from pathlib import Path

    p = Path(__file__).resolve().parents[1] / "calibration.json"
    if p.exists():
        c = json.load(open(p)).get("lidar", {})
        LIDAR_ERRORS.ambiguity_k = c.get("ambiguity_k", LIDAR_ERRORS.ambiguity_k)
        LIDAR_ERRORS.face_floor = c.get("face_floor", LIDAR_ERRORS.face_floor)


_load_calibration()


def build_scene(capture_dir, drift_correction=True, log=print) -> Scene:
    cap = load_stray(capture_dir)
    kf = select_keyframes(cap.poses, min_trans=0.10, min_rot_deg=8.0, max_gap=30)
    poses = cap.poses
    drift_info = {"method": "none (poses used as-is)", "enabled": False}
    if drift_correction:
        from ..geometry.drift import correct_drift, lidar_cloud_fn

        poses, drift_info = correct_drift(cap.poses, kf, lidar_cloud_fn(cap), log=log)
    pc = fuse(cap, kf, poses=poses, stride=2, max_depth=5.0, min_conf=2, voxel=0.02)
    views = []
    for i in kf:
        d = cap.load_depth(int(cap.frame_ids[i]), min_conf=2)
        if d is None:
            continue
        p, _ = backproject(d, cap.K_depth, stride=3, max_depth=5.0)
        T = poses[i]
        views.append((T[:3, 3], p @ T[:3, :3].T + T[:3, 3]))
    E = LIDAR_ERRORS
    return Scene(
        tier="lidar", xyz=pc.xyz, normals=pc.normals, cam_centres=poses[kf, :3, 3], views=views,
        path_length_m=cap.path_length_m, error_model=E,
        meta=dict(capture=str(capture_dir), keyframes=len(kf), frames=len(cap.poses),
                  duration_s=round(cap.duration_s, 1), drift=drift_info, poses=poses, kf=kf,
                  K_rgb=cap.K_rgb, frame_ids=cap.frame_ids),
    )
