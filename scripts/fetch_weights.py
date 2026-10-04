"""Fetch model weights (never committed to the repo).

    python scripts/fetch_weights.py            # downloads into ./weights
    ROOMSCAN_WEIGHTS=/some/dir python ...      # alternative location

Model: Depth Anything V2, Metric-Indoor, Small (Apache-2.0).
https://huggingface.co/depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf
Only the video and photo tiers need it; the LiDAR tier runs without any weights.
"""
from __future__ import annotations

import hashlib
import os
import shutil
import sys
import urllib.request
from pathlib import Path

REPO = "depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf"
FILES = ["config.json", "preprocessor_config.json", "model.safetensors"]
DEST = Path(os.environ.get("ROOMSCAN_WEIGHTS", Path(__file__).resolve().parents[1] / "weights")) \
    / "depth_anything_v2_metric_indoor_small"
# sha256 of the files used for every reported number (filled from the benchmark machine)
SHA256 = {}


def sha256(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def main():
    DEST.mkdir(parents=True, exist_ok=True)
    for name in FILES:
        out = DEST / name
        if out.exists() and (name not in SHA256 or sha256(out) == SHA256[name]):
            print(f"ok   {out}")
            continue
        url = f"https://huggingface.co/{REPO}/resolve/main/{name}"
        print(f"get  {url}")
        tmp = out.with_suffix(out.suffix + ".part")
        with urllib.request.urlopen(url) as r, open(tmp, "wb") as f:
            shutil.copyfileobj(r, f)
        tmp.rename(out)
        if name in SHA256 and sha256(out) != SHA256[name]:
            sys.exit(f"checksum mismatch for {name}")
    print(f"weights ready in {DEST}")


if __name__ == "__main__":
    main()
