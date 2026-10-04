"""Accumulated-drift correction: fragment pose graph with ICP loop closures.

ARKit poses are good locally but drift over a long multi-room walk (they show up as doubled
walls when the walk revisits a room). We:
  1. cut the keyframe sequence into fragments (~1.5 m of path each) and fuse each fragment,
  2. add odometry edges between consecutive fragments from the ARKit relative pose,
  3. add loop-closure edges between *non-consecutive* fragments that overlap spatially, from
     point-to-plane ICP initialised at the ARKit relative pose (gravity is trusted, so the
     ICP result is projected back to yaw + translation, i.e. 4-DoF),
  4. optimise the pose graph (Open3D Levenberg–Marquardt with line-process outlier pruning),
  5. apply each fragment's correction to its frames.
Reported diagnostics feed the drift-ablation table.
"""
from __future__ import annotations

import numpy as np

from .fusion import backproject


def _yaw_only(T):
    """Project a rigid transform to rotation about +y and translation (gravity trusted)."""
    R = T[:3, :3]
    yaw = np.arctan2(R[0, 2], R[2, 2])
    c, s = np.cos(yaw), np.sin(yaw)
    out = np.eye(4)
    out[:3, :3] = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    out[:3, 3] = T[:3, 3]
    return out


def _fragment_cloud(cap, idx, poses, anchor_inv, voxel):
    import open3d as o3d

    pts = []
    for i in idx:
        d = cap.load_depth(int(cap.frame_ids[i]), min_conf=2)
        if d is None:
            continue
        p, _ = backproject(d, cap.K_depth, stride=3, max_depth=4.0)
        T = anchor_inv @ poses[i]
        pts.append(p @ T[:3, :3].T + T[:3, 3])
    if not pts:
        return None
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.concatenate(pts)))
    pcd = pcd.voxel_down_sample(voxel)
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 3, max_nn=30))
    return pcd


def correct_drift(cap, kf, frag_len=12, voxel=0.04, max_pair_dist=3.0, log=print):
    import open3d as o3d

    poses = cap.poses
    frags = [kf[i:i + frag_len] for i in range(0, len(kf), frag_len)]
    anchors = [poses[f[0]] for f in frags]
    clouds = [_fragment_cloud(cap, f, poses, np.linalg.inv(a), voxel) for f, a in zip(frags, anchors)]
    centres = np.array([poses[f, :3, 3].mean(0) for f in frags])
    n = len(frags)

    pg = o3d.pipelines.registration.PoseGraph()
    for a in anchors:
        pg.nodes.append(o3d.pipelines.registration.PoseGraphNode(a.copy()))
    info_odo = np.eye(6) * 1e4
    for k in range(n - 1):
        rel = np.linalg.inv(anchors[k + 1]) @ anchors[k]   # maps k -> k+1 frame (source k, target k+1)
        pg.edges.append(o3d.pipelines.registration.PoseGraphEdge(k, k + 1, rel, info_odo, uncertain=False))

    loops, residuals = [], []
    crit = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=40)
    for i in range(n):
        for j in range(i + 3, n):
            if clouds[i] is None or clouds[j] is None:
                continue
            if np.linalg.norm(centres[i] - centres[j]) > max_pair_dist:
                continue
            init = np.linalg.inv(anchors[j]) @ anchors[i]
            r1 = o3d.pipelines.registration.registration_icp(
                clouds[i], clouds[j], 0.10, init, o3d.pipelines.registration.TransformationEstimationPointToPlane(), crit)
            r2 = o3d.pipelines.registration.registration_icp(
                clouds[i], clouds[j], 0.03, r1.transformation,
                o3d.pipelines.registration.TransformationEstimationPointToPlane(), crit)
            if r2.fitness < 0.35 or r2.inlier_rmse > 0.02:
                continue
            # discrepancy expressed in the (gravity-aligned) world frame, projected to 4-DoF
            D = _yaw_only(anchors[j] @ r2.transformation @ np.linalg.inv(anchors[i]))
            Tij = np.linalg.inv(anchors[j]) @ D @ anchors[i]
            dt = float(np.linalg.norm(D[:3, 3] + (D[:3, :3] - np.eye(3)) @ centres[i]))
            if dt > 0.30:
                continue
            info = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
                clouds[i], clouds[j], 0.03, Tij)
            pg.edges.append(o3d.pipelines.registration.PoseGraphEdge(i, j, Tij, info, uncertain=True))
            loops.append((i, j))
            residuals.append(dt)

    if not loops:
        log("  drift: no loop closures found; poses unchanged")
        return poses.copy(), dict(method="fragment pose graph + ICP loop closure", enabled=True,
                                  fragments=n, loop_edges=0, mean_loop_residual_m=None,
                                  max_correction_m=0.0)
    opt = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=0.03, edge_prune_threshold=0.25, reference_node=0)
    o3d.pipelines.registration.global_optimization(
        pg, o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(), opt)

    new = poses.copy()
    corr_mag = []
    # piecewise correction, linearly blended in translation between fragment anchors
    corrs = []
    for k in range(n):
        C = _yaw_only(pg.nodes[k].pose @ np.linalg.inv(anchors[k]))
        corrs.append(C)
        corr_mag.append(np.linalg.norm(pg.nodes[k].pose[:3, 3] - anchors[k][:3, 3]))
    frag_of = np.zeros(len(poses), int)
    for k, f in enumerate(frags):
        lo = f[0]
        hi = frags[k + 1][0] if k + 1 < n else len(poses)
        frag_of[lo:hi] = k
    frag_of[: frags[0][0]] = 0
    for t in range(len(poses)):
        new[t] = corrs[frag_of[t]] @ poses[t]

    # residual after optimisation on the accepted loop edges
    post = []
    for (i, j), e in zip(loops, [e for e in pg.edges if e.uncertain]):
        pred = np.linalg.inv(pg.nodes[j].pose) @ pg.nodes[i].pose
        post.append(float(np.linalg.norm((np.linalg.inv(pred) @ e.transformation)[:3, 3])))
    info = dict(method="fragment pose graph + 4-DoF ICP loop closure (Open3D LM, line-process pruning)",
                enabled=True, fragments=n, loop_edges=len(loops),
                mean_loop_residual_before_m=round(float(np.mean(residuals)), 4),
                mean_loop_residual_after_m=round(float(np.mean(post)), 4),
                max_correction_m=round(float(np.max(corr_mag)), 4))
    log(f"  drift: {n} fragments, {len(loops)} loop edges, residual "
        f"{info['mean_loop_residual_before_m']*100:.1f} -> {info['mean_loop_residual_after_m']*100:.1f} cm, "
        f"max correction {info['max_correction_m']*100:.1f} cm")
    return new, info
