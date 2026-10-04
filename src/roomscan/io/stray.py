"""Loader for Stray Scanner captures (iOS, LiDAR devices).

Folder layout produced by Stray Scanner "export":
    rgb.mp4            HEVC, 1920x1440 @ 60 fps
    depth/NNNNNN.png   uint16 millimetres, 256x192, one per video frame
    confidence/NNNNNN.png  uint8 {0,1,2} ARKit depth confidence
    odometry.csv       timestamp, frame, x, y, z, qx, qy, qz, qw [, fx, fy, cx, cy, ...]
    camera_matrix.csv  3x3 RGB intrinsics (pixels at 1920x1440)
    imu.csv            accelerometer + gyro

Poses are ARKit camera-to-world: camera +x right, +y up, +z backward
(the camera looks down -z). World is gravity-aligned with +y up.
We convert to an OpenCV-style camera frame (+z forward, +y down) on load
so every downstream module uses one convention.
"""
from __future__ import annotations

import csv
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

# ARKit camera axes -> OpenCV camera axes
_ARKIT_TO_CV = np.eye(3)  # Stray already stores OpenCV-convention camera axes (verified empirically, see DECISIONS.md)

RGB_W, RGB_H = 1920, 1440
DEPTH_W, DEPTH_H = 256, 192


@dataclass
class StrayCapture:
    root: Path
    timestamps: np.ndarray            # (N,)
    frame_ids: np.ndarray             # (N,) int
    poses: np.ndarray                 # (N,4,4) camera(cv)->world, metres
    K_rgb: np.ndarray                 # (3,3) at 1920x1440
    per_frame_K: np.ndarray | None = None  # (N,3,3) if present
    meta: dict = field(default_factory=dict)

    # ---- intrinsics -------------------------------------------------
    @property
    def K_depth(self) -> np.ndarray:
        s = DEPTH_W / RGB_W
        K = self.K_rgb.copy()
        K[0, :] *= s
        K[1, :] *= s
        return K

    # ---- frames ------------------------------------------------------
    def depth_path(self, fid: int) -> Path:
        return self.root / "depth" / f"{fid:06d}.png"

    def conf_path(self, fid: int) -> Path:
        return self.root / "confidence" / f"{fid:06d}.png"

    def has_depth(self) -> bool:
        return (self.root / "depth").is_dir() and any((self.root / "depth").iterdir())

    def load_depth(self, fid: int, min_conf: int = 2) -> np.ndarray | None:
        """Depth in metres (float32, H x W). Pixels below `min_conf` are set to 0."""
        import cv2

        p = self.depth_path(fid)
        if not p.exists():
            return None
        d = cv2.imread(str(p), cv2.IMREAD_UNCHANGED).astype(np.float32) / 1000.0
        cp = self.conf_path(fid)
        if min_conf > 0 and cp.exists():
            c = cv2.imread(str(cp), cv2.IMREAD_UNCHANGED)
            d[c < min_conf] = 0.0
        return d

    def gravity_up(self) -> np.ndarray:
        return np.array([0.0, 1.0, 0.0])

    @property
    def duration_s(self) -> float:
        return float(self.timestamps[-1] - self.timestamps[0])

    @property
    def path_length_m(self) -> float:
        t = self.poses[:, :3, 3]
        return float(np.linalg.norm(np.diff(t, axis=0), axis=1).sum())


def _read_matrix(path: Path) -> np.ndarray:
    rows = [r for r in csv.reader(open(path)) if r]
    return np.array([[float(v) for v in r] for r in rows], dtype=np.float64)


def load_stray(root: str | Path) -> StrayCapture:
    root = Path(root)
    K = _read_matrix(root / "camera_matrix.csv")
    rows = list(csv.reader(open(root / "odometry.csv")))
    header = [h.strip() for h in rows[0]]
    body = [r for r in rows[1:] if r]
    col = {h: i for i, h in enumerate(header)}

    def f(r, name):
        v = r[col[name]].strip()
        return float(v) if v else np.nan

    ts = np.array([f(r, "timestamp") for r in body])
    fids = np.array([int(float(r[col["frame"]])) for r in body])
    xyz = np.array([[f(r, "x"), f(r, "y"), f(r, "z")] for r in body])
    quat = np.array([[f(r, "qx"), f(r, "qy"), f(r, "qz"), f(r, "qw")] for r in body])
    R = Rotation.from_quat(quat).as_matrix() @ _ARKIT_TO_CV
    T = np.tile(np.eye(4), (len(body), 1, 1))
    T[:, :3, :3] = R
    T[:, :3, 3] = xyz

    per_K = None
    if "fx" in col:
        per_K = np.tile(np.eye(3), (len(body), 1, 1))
        per_K[:, 0, 0] = [f(r, "fx") for r in body]
        per_K[:, 1, 1] = [f(r, "fy") for r in body]
        per_K[:, 0, 2] = [f(r, "cx") for r in body]
        per_K[:, 1, 2] = [f(r, "cy") for r in body]
    return StrayCapture(root=root, timestamps=ts, frame_ids=fids, poses=T, K_rgb=K, per_frame_K=per_K)


def is_stray_capture(root: str | Path) -> bool:
    root = Path(root)
    return (root / "odometry.csv").exists() and (root / "camera_matrix.csv").exists()


def estimate_rgb_offset(cap: StrayCapture, n_samples=6, search=6) -> int:
    """Video frame n shows the scene of depth/odometry row n + offset.

    Found by maximising overlap between RGB edges and depth discontinuities. On the sample
    captures the encoder drops the first frame, so the answer is +1; a wrong offset showed up
    as 18-32 px reprojection error at 1920 px and as damage masks landing on the wrong surface.
    """
    import subprocess
    import tempfile

    import cv2

    N = len(cap.frame_ids)
    ns = np.linspace(N * 0.15, N * 0.85, n_samples).astype(int)
    expr = "+".join(f"eq(n\\,{n})" for n in ns)
    with tempfile.TemporaryDirectory() as td:
        subprocess.run(["ffmpeg", "-v", "error", "-y", "-i", str(cap.root / "rgb.mp4"), "-vf",
                        f"select='{expr}',scale={DEPTH_W}:{DEPTH_H}", "-vsync", "0", f"{td}/a_%02d.png"], check=True)
        score = {o: 0.0 for o in range(-search, search + 1)}
        for k, n in enumerate(ns):
            g = cv2.imread(f"{td}/a_{k + 1:02d}.png", 0)
            if g is None:
                continue
            ge = cv2.dilate(cv2.Canny(g, 30, 90), np.ones((3, 3), np.uint8)) > 0
            for o in score:
                if not 0 <= n + o < N:
                    continue
                d = cap.load_depth(int(cap.frame_ids[n + o]), min_conf=0)
                if d is None:
                    continue
                dl = np.abs(cv2.Laplacian(d, cv2.CV_32F)) > 0.15
                score[o] += (dl & ge).sum() / max(dl.sum(), 1)
    return max(score, key=score.get)
