"""Gravity + Manhattan frame estimation.

After `PlanFrame.fit`, every point can be expressed in plan coordinates:
    u, v  horizontal, aligned with the dominant wall directions
    h     height above the (global) floor plane
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def _circ_hist_peak(angles_deg: np.ndarray, period: float = 90.0, bin_deg: float = 0.5, weights=None):
    a = np.mod(angles_deg, period)
    nb = int(period / bin_deg)
    h, _ = np.histogram(a, bins=nb, range=(0, period), weights=weights)
    # circular smoothing
    k = np.array([1, 2, 3, 2, 1], float)
    hp = np.concatenate([h[-2:], h, h[:2]])
    hs = np.convolve(hp, k / k.sum(), mode="valid")
    pk = (np.argmax(hs) + 0.5) * bin_deg
    # refine with circular mean of samples within ±2°
    d = (a - pk + period / 2) % period - period / 2
    m = np.abs(d) < 2.0
    if m.sum() > 10:
        pk = pk + np.median(d[m])
    return pk % period


def dominant_yaw(normals: np.ndarray) -> float:
    """Yaw (deg) of the dominant wall direction, from horizontal normals (y-up world)."""
    hor = np.abs(normals[:, 1]) < 0.15
    n = normals[hor]
    ang = np.degrees(np.arctan2(n[:, 2], n[:, 0]))
    return float(_circ_hist_peak(ang))


def horizontal_peaks(heights: np.ndarray, bin_m: float = 0.01, min_frac: float = 0.02):
    """Return (centre, count) of strong peaks in a height histogram."""
    if len(heights) == 0:
        return []
    lo, hi = np.floor(heights.min()), np.ceil(heights.max())
    h, e = np.histogram(heights, bins=np.arange(lo, hi + bin_m, bin_m))
    hs = np.convolve(h, np.ones(3) / 3, mode="same")
    thr = max(min_frac * hs.max(), 50)
    peaks = []
    for i in range(1, len(hs) - 1):
        if hs[i] >= thr and hs[i] >= hs[i - 1] and hs[i] >= hs[i + 1]:
            peaks.append((0.5 * (e[i] + e[i + 1]), float(hs[i])))
    # non-max suppression within 10 cm
    peaks.sort(key=lambda t: -t[1])
    kept = []
    for c, s in peaks:
        if all(abs(c - k[0]) > 0.10 for k in kept):
            kept.append((c, s))
    return sorted(kept)


def refine_plane_height(y: np.ndarray, centre: float, band: float = 0.04):
    """Robust height of a horizontal plane near `centre`: (value, std, n)."""
    m = np.abs(y - centre) < band
    if m.sum() < 20:
        return centre, np.inf, int(m.sum())
    v = y[m]
    for _ in range(3):
        med = np.median(v)
        mad = 1.4826 * np.median(np.abs(v - med)) + 1e-4
        v = v[np.abs(v - med) < 3 * mad]
    return float(np.median(v)), float(v.std()), int(len(v))


@dataclass
class PlanFrame:
    yaw_deg: float
    floor_y: float
    floor_std: float

    @classmethod
    def fit(cls, xyz: np.ndarray, normals: np.ndarray) -> "PlanFrame":
        yaw = dominant_yaw(normals)
        up = np.abs(normals[:, 1]) > 0.9
        y = xyz[up, 1]
        # floor = lowest strong horizontal peak
        pk = horizontal_peaks(y, min_frac=0.05)
        floor_c = pk[0][0] if pk else np.percentile(xyz[:, 1], 1)
        fy, fs, _ = refine_plane_height(y, floor_c)
        return cls(yaw_deg=yaw, floor_y=fy, floor_std=fs)

    @property
    def R(self) -> np.ndarray:
        """2x2 rotation mapping world (x,z) -> plan (u,v)."""
        t = np.radians(self.yaw_deg)
        c, s = np.cos(t), np.sin(t)
        return np.array([[c, s], [-s, c]])

    def to_plan(self, xyz: np.ndarray) -> np.ndarray:
        """(N,3) world -> (N,3) [u, v, h]."""
        uv = xyz[:, [0, 2]] @ self.R.T
        return np.column_stack([uv, xyz[:, 1] - self.floor_y])

    def normals_to_plan(self, n: np.ndarray) -> np.ndarray:
        uv = n[:, [0, 2]] @ self.R.T
        return np.column_stack([uv, n[:, 1]])

    def from_plan_uv(self, uv: np.ndarray) -> np.ndarray:
        return uv @ self.R  # inverse rotation -> world (x,z)
