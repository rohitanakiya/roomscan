"""Calibrate the mono-depth model's metric scale against LiDAR (one number per device).

For each capture, 30 evenly spaced frames of the *derived* upright video (exactly what the video tier sees)
are passed through the depth model and compared with the LiDAR depth of the same instant (median ratio
over confident pixels in 0.3-6 m). Output:

  bench/results/depth_calibration.json   per-capture ratios, pooled scale, leave-one-capture-out scales
  src/roomscan/depth_calibration.json    pooled scale shipped with the package (used by default)

The benchmark scores each capture with the scale fitted on the *other* captures (leave-one-out).

    python bench/calibrate_depth.py
"""
from __future__ import annotations

import json
import subprocess
import sys
import tempfile
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))
from make_tier_inputs import rotate_img  # noqa: E402
from roomscan.io.stray import load_stray  # noqa: E402
from roomscan.models.depth import MonoDepth, default_cache_dir  # noqa: E402
from roomscan.ffmpeg_bin import ffmpeg_exe  # noqa: E402

CAPTURES = ["single_room", "floor_only", "with_ceiling"]
N_FRAMES = 30


def capture_ratio(name, model):
    man = json.loads((ROOT / "data" / "tiers" / f"{name}_tier_manifest.json").read_text())
    cap = load_stray(ROOT / "data" / "raw" / name)
    deg, off = man["rotation_deg"], man["rgb_offset"]
    vid = ROOT / "data" / "tiers" / "video" / f"{name}.mp4"
    n_video = len(cap.frame_ids) - off
    ns = np.linspace(5, n_video - 5, N_FRAMES).astype(int)
    with tempfile.TemporaryDirectory() as td:
        expr = "+".join(f"eq(n\\,{n})" for n in ns)
        subprocess.check_call([ffmpeg_exe(), "-v", "error", "-i", str(vid), "-vf", f"select='{expr}'", "-vsync", "0",
                               "-q:v", "2", f"{td}/%03d.jpg"])
        files = sorted(Path(td).glob("*.jpg"))
        assert len(files) == len(ns), "frame extraction count mismatch"
        ratios = []
        for n, f in zip(ns, files):
            L = cap.load_depth(int(cap.frame_ids[n + off]), min_conf=2)
            if L is None:
                continue
            L = rotate_img(L, deg)
            d = model.predict(cv2.cvtColor(cv2.imread(str(f)), cv2.COLOR_BGR2RGB))
            d = cv2.resize(d, (L.shape[1], L.shape[0]), interpolation=cv2.INTER_AREA)
            ok = (L > 0.3) & (L < 6.0)
            if ok.sum() > 500:
                ratios.append(float(np.median(d[ok] / L[ok])))
    r = np.array(ratios)
    return dict(frames=len(r), ratio_median=round(float(np.median(r)), 4),
                ratio_p10=round(float(np.percentile(r, 10)), 4), ratio_p90=round(float(np.percentile(r, 90)), 4))


def main():
    model = MonoDepth(cache_dir=default_cache_dir())
    per = {c: capture_ratio(c, model) for c in CAPTURES}
    for c, v in per.items():
        print(f"{c}: model/LiDAR depth = {v['ratio_median']} (p10 {v['ratio_p10']}, p90 {v['ratio_p90']}, n={v['frames']})")
    allr = np.array([v["ratio_median"] for v in per.values()])
    pooled = 1.0 / float(allr.mean())
    loo = {c: round(1.0 / float(np.mean([per[o]["ratio_median"] for o in CAPTURES if o != c])), 4) for c in CAPTURES}
    # how far each held-out capture's true correction is from its leave-one-out value (scale error it will carry)
    loo_err = {c: round(100 * (loo[c] * per[c]["ratio_median"] - 1), 2) for c in CAPTURES}
    out = dict(model="Depth-Anything-V2-Metric-Indoor-Small", device="iPhone (Stray Scanner RGB, 1920x1440)",
               per_capture=per, scale=round(pooled, 4), leave_one_out_scale=loo, leave_one_out_scale_error_pct=loo_err)
    (ROOT / "bench" / "results").mkdir(parents=True, exist_ok=True)
    (ROOT / "bench" / "results" / "depth_calibration.json").write_text(json.dumps(out, indent=1))
    (ROOT / "src" / "roomscan" / "depth_calibration.json").write_text(
        json.dumps(dict(scale=out["scale"], model=out["model"], device=out["device"],
                        source="bench/calibrate_depth.py (bench/results/depth_calibration.json)"), indent=1))
    print(f"pooled scale {out['scale']}; leave-one-out {loo}; residual scale error {loo_err} %")


if __name__ == "__main__":
    main()
