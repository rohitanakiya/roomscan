"""2D plan rasters (top-down evidence images) in plan coordinates."""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np
from scipy import ndimage as ndi


@dataclass
class Grid:
    origin: np.ndarray  # (2,) plan (u,v) of pixel (0,0) corner
    res: float
    shape: tuple        # (rows, cols) -> rows index v, cols index u

    @classmethod
    def around(cls, uv: np.ndarray, res=0.02, pad=0.3):
        lo = uv.min(0) - pad
        hi = uv.max(0) + pad
        sz = np.ceil((hi - lo) / res).astype(int)
        return cls(lo, res, (int(sz[1]), int(sz[0])))

    def to_px(self, uv):
        ij = np.floor((uv - self.origin) / self.res).astype(int)
        return ij  # (col=u, row=v)

    def to_uv(self, col, row):
        return self.origin + (np.stack([col, row], -1) + 0.5) * self.res

    def count(self, uv, weights=None):
        ij = self.to_px(uv)
        ok = (ij[:, 0] >= 0) & (ij[:, 0] < self.shape[1]) & (ij[:, 1] >= 0) & (ij[:, 1] < self.shape[0])
        img = np.zeros(self.shape, np.float32)
        np.add.at(img, (ij[ok, 1], ij[ok, 0]), 1.0 if weights is None else weights[ok])
        return img


def disk(r_px: int):
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r_px + 1, 2 * r_px + 1))


def fill_small_holes(mask: np.ndarray, max_area_px: int) -> np.ndarray:
    holes = ~mask
    lab, n = ndi.label(holes)
    if n == 0:
        return mask
    sizes = ndi.sum(np.ones_like(lab), lab, index=np.arange(1, n + 1))
    border = set(np.unique(np.concatenate([lab[0], lab[-1], lab[:, 0], lab[:, -1]])))
    out = mask.copy()
    for k, s in enumerate(sizes, start=1):
        if s <= max_area_px and k not in border:
            out[lab == k] = True
    return out
