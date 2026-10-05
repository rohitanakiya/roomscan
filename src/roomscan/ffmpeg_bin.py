"""Locate ffmpeg: the one on PATH if present, else the static build shipped by the `imageio-ffmpeg` wheel.

A reviewer on a clean Windows machine usually has no ffmpeg; `pip install -r requirements.txt` alone must be
enough to run every tier (found by the clean-machine test, DECISIONS D26).
"""
from __future__ import annotations

import functools
import shutil


@functools.lru_cache(maxsize=1)
def ffmpeg_exe() -> str:
    exe = shutil.which("ffmpeg")
    if exe:
        return exe
    try:
        import imageio_ffmpeg

        return imageio_ffmpeg.get_ffmpeg_exe()
    except Exception as e:  # noqa: BLE001
        raise RuntimeError("ffmpeg not found: install it (e.g. `winget install Gyan.FFmpeg`, `brew install ffmpeg`, "
                           "`apt install ffmpeg`) or `pip install imageio-ffmpeg`") from e
