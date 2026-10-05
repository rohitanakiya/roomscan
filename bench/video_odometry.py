"""Video-tier odometry vs ARKit: absolute trajectory error of the RGB-only camera path.

The video tier never sees ARKit poses; this script uses them afterwards, as the reference, to separate
"the camera path is wrong" from "the layout step is wrong" when a video plan disagrees with the LiDAR plan.

    python bench/video_odometry.py out/bench/video/<capture>/work/trajectory.npz data/raw/<capture>
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from roomscan.io.stray import estimate_rgb_offset, load_stray  # noqa: E402


def umeyama(src, dst, scale=True):
    """dst ~ s R src + t (least squares)."""
    ms, md = src.mean(0), dst.mean(0)
    A, B = src - ms, dst - md
    U, S, Vt = np.linalg.svd(B.T @ A / len(src))
    D = np.eye(3)
    D[2, 2] = np.sign(np.linalg.det(U @ Vt))
    R = U @ D @ Vt
    s = (S * np.diag(D)).sum() / (A ** 2).sum(1).mean() if scale else 1.0
    return s, R, md - s * R @ ms


def evaluate(traj_npz, capture):
    z = np.load(traj_npz)
    cap = load_stray(capture)
    off = estimate_rgb_offset(cap)
    # video frame n <-> odometry row n + off; video time 0 = first video frame
    t_ref = cap.timestamps - cap.timestamps[off]
    out = {}
    for key in ("raw", "final"):
        P = z[key][:, :3, 3]
        idx = np.clip(np.searchsorted(t_ref, z["t"]), 0, len(t_ref) - 1)
        G = cap.poses[idx, :3, 3]
        ok = z["t"] <= t_ref[-1]
        P, G = P[ok], G[ok]
        s, R, t = umeyama(P, G, scale=True)
        e_sim = np.linalg.norm((s * P @ R.T + t) - G, axis=1)
        _, R1, t1 = umeyama(P, G, scale=False)
        e_se3 = np.linalg.norm((P @ R1.T + t1) - G, axis=1)
        path = float(np.linalg.norm(np.diff(G, axis=0), axis=1).sum())
        out[key] = dict(n=int(len(P)), path_m=round(path, 2), scale_factor=round(float(s), 4),
                        ate_rmse_m=round(float(np.sqrt((e_se3 ** 2).mean())), 3),
                        ate_rmse_after_scale_m=round(float(np.sqrt((e_sim ** 2).mean())), 3),
                        ate_max_m=round(float(e_se3.max()), 3),
                        ate_per_m_walked_pct=round(100 * float(np.sqrt((e_se3 ** 2).mean())) / max(path, 1e-6), 2))
    return out


if __name__ == "__main__":
    print(json.dumps(evaluate(sys.argv[1], sys.argv[2]), indent=1))
