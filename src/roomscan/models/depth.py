"""Monocular metric depth.

Model: Depth Anything V2, Metric-Indoor, ViT-S (Apache-2.0, 24.8M params; trained on
Hypersim for metric indoor depth). Weights are fetched by scripts/fetch_weights.py into
weights/depth_anything_v2_metric_indoor_small/ — never committed.

Outputs are cached per frame (npz keyed by image content hash + model id), so a benchmark
replay is deterministic and the live path is identical code.
"""
from __future__ import annotations

import hashlib
import os
from pathlib import Path

import numpy as np

MODEL_DIR = Path(os.environ.get("ROOMSCAN_WEIGHTS", Path(__file__).resolve().parents[3] / "weights")) \
    / "depth_anything_v2_metric_indoor_small"
MODEL_ID = "depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf"
INPUT_SIZE = 518


class MonoDepth:
    def __init__(self, cache_dir: str | Path | None = None, device="cpu", threads=None):
        self.cache_dir = Path(cache_dir) if cache_dir else None
        self._model = None
        self.device = device
        self.threads = threads

    def _load(self):
        if self._model is not None:
            return
        import torch
        from transformers import AutoModelForDepthEstimation

        if not (MODEL_DIR / "model.safetensors").exists():
            raise FileNotFoundError(
                f"depth weights missing at {MODEL_DIR}; run `python scripts/fetch_weights.py`")
        if self.threads:
            torch.set_num_threads(self.threads)
        self._model = AutoModelForDepthEstimation.from_pretrained(str(MODEL_DIR)).to(self.device).eval()

    def _key(self, img: np.ndarray):
        h = hashlib.sha1(img[::4, ::4].tobytes()).hexdigest()[:16]
        return f"{h}_{img.shape[0]}x{img.shape[1]}"

    def predict(self, img_rgb: np.ndarray) -> np.ndarray:
        """img_rgb uint8 HxWx3 -> metric depth (m) HxW float32 (resized back to input)."""
        if self.cache_dir:
            p = self.cache_dir / f"{self._key(img_rgb)}.npz"
            if p.exists():
                return np.load(p)["d"].astype(np.float32)
        d = self._infer(img_rgb)
        if self.cache_dir:
            self.cache_dir.mkdir(parents=True, exist_ok=True)
            np.savez_compressed(self.cache_dir / f"{self._key(img_rgb)}.npz", d=d.astype(np.float16))
            d = d.astype(np.float16).astype(np.float32)  # identical numbers live vs replay
        return d

    def _infer(self, img_rgb):
        import cv2
        import torch

        self._load()
        h, w = img_rgb.shape[:2]
        # keep aspect, long side ~ INPUT_SIZE*4/3, multiple of 14 (ViT patch)
        s = INPUT_SIZE / min(h, w)
        nh, nw = int(round(h * s / 14)) * 14, int(round(w * s / 14)) * 14
        x = cv2.resize(img_rgb, (nw, nh), interpolation=cv2.INTER_CUBIC).astype(np.float32) / 255.0
        x = (x - np.array([0.485, 0.456, 0.406])) / np.array([0.229, 0.224, 0.225])
        t = torch.from_numpy(x.transpose(2, 0, 1)[None].astype(np.float32)).to(self.device)
        with torch.inference_mode():
            out = self._model(pixel_values=t).predicted_depth[0].cpu().numpy()
        return cv2.resize(out, (w, h), interpolation=cv2.INTER_LINEAR).astype(np.float32)


def available() -> bool:
    return (MODEL_DIR / "model.safetensors").exists()


def depth_scale() -> float:
    """Multiplicative correction for the model's metric depth on iPhone video/photos.

    Depth Anything V2 metric-indoor over-estimates distance on these frames by ~25 % (its metric scale is
    tied to its training cameras). One number per device, fitted against LiDAR frames by
    bench/calibrate_depth.py and shipped in roomscan/depth_calibration.json. The benchmark overrides it
    per capture with a leave-one-capture-out value via ROOMSCAN_DEPTH_SCALE, so no capture is scored with a
    scale fitted on itself.
    """
    env = os.environ.get("ROOMSCAN_DEPTH_SCALE")
    if env:
        return float(env)
    f = Path(__file__).resolve().parents[1] / "depth_calibration.json"
    if f.exists():
        import json

        return float(json.loads(f.read_text())["scale"])
    return 1.0

