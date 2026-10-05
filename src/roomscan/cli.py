"""One command per capture.

    python -m roomscan <capture_path> [--tier auto|lidar|video|photo] [--out out/<name>]

Tier auto-detection:
  * folder with odometry.csv + depth/      -> lidar
  * a video file (.mp4/.mov) or folder holding exactly one video and no depth -> video
  * folder of sub-folders containing images (one sub-folder per room)          -> photo
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

VIDEO_EXT = {".mp4", ".mov", ".m4v"}
IMG_EXT = {".jpg", ".jpeg", ".png", ".heic", ".heif"}


def detect_tier(p: Path) -> str:
    if p.is_file() and p.suffix.lower() in VIDEO_EXT:
        return "video"
    if (p / "odometry.csv").exists() and (p / "depth").is_dir():
        return "lidar"
    vids = [f for f in p.iterdir() if f.suffix.lower() in VIDEO_EXT] if p.is_dir() else []
    if vids:
        return "video"
    subs = [d for d in p.iterdir() if d.is_dir() and any(f.suffix.lower() in IMG_EXT for f in d.iterdir())]
    if subs:
        return "photo"
    raise SystemExit(f"cannot detect tier for {p}")


def run(capture: str, tier: str = "auto", out: str | None = None, drift: bool = True, damage: bool = True,
        log=print) -> dict:
    t0 = time.time()
    p = Path(capture)
    tier = detect_tier(p) if tier == "auto" else tier
    out = Path(out or Path("out") / (p.stem if p.is_file() else p.name))
    out.mkdir(parents=True, exist_ok=True)
    log(f"[roomscan] {p} -> tier={tier} -> {out}")
    timing = {}
    t = time.time()
    if tier == "lidar":
        from .tiers.lidar import build_scene
        scene = build_scene(p, drift_correction=drift, log=log)
    elif tier == "video":
        from .tiers.video import build_scene
        scene = build_scene(p, work=out / "work", log=log)
    elif tier == "photo":
        from .tiers.photo import build_scene
        scene = build_scene(p, work=out / "work", log=log)
    else:
        raise SystemExit(f"unknown tier {tier}")
    timing["reconstruct_s"] = round(time.time() - t, 1)
    from .analyze import analyze
    t = time.time()
    prop = analyze(scene, log=log)
    timing["layout_s"] = round(time.time() - t, 1)
    if damage:
        t = time.time()
        try:
            from .damage.pipeline import run_damage
            run_damage(scene, prop, out / "damage", log=log)
        except Exception as e:  # damage must never block the geometry contract
            log(f"  damage: skipped ({type(e).__name__}: {e})")
            prop.meta.setdefault("warnings", []).append(f"damage stage failed: {e}")
        timing["damage_s"] = round(time.time() - t, 1)
    from .export import property_to_json, write_json
    from .render import render_plan

    res = property_to_json(prop, scene)
    timing["total_s"] = round(time.time() - t0, 1)
    res["pipeline"]["timing"] = timing
    write_json(res, out / "result.json")
    render_plan(res, str(out / "plan.png"), str(out / "plan.svg"))
    log(f"[roomscan] done in {timing['total_s']} s: {len(res['rooms'])} rooms, footprint "
        f"{res['property']['footprint_area']['value']:.2f} m2 -> {out/'result.json'}")
    return res


def main(argv=None):
    ap = argparse.ArgumentParser(prog="roomscan", description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("capture")
    ap.add_argument("--tier", default="auto", choices=["auto", "lidar", "video", "photo"])
    ap.add_argument("--out")
    ap.add_argument("--no-drift", action="store_true", help="ablation: use raw ARKit poses (LiDAR tier)")
    ap.add_argument("--no-damage", action="store_true")
    a = ap.parse_args(argv)
    run(a.capture, a.tier, a.out, drift=not a.no_drift, damage=not a.no_damage)


if __name__ == "__main__":
    main()
