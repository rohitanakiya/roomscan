"""Derive the video-tier and photo-tier benchmark inputs from a Stray LiDAR capture.

The sample data contains only Stray Scanner captures, so the thinner tiers are produced from
them, keeping only what that tier would really have:

  video/<name>.mp4         rgb.mp4 re-encoded upright (as the iPhone Camera app would store it),
                           no depth, no poses, no intrinsics, no metadata
  photos/<name>/room_k/    2-8 stills per room (the rooms a person would photograph), upright,
                           EXIF-free JPEGs; the protocol's "doorway shot" (standing in a room,
                           looking through an opening into the next room) is included per room

The LiDAR result is used only to decide which frames a person standing in each room would
have taken; nothing geometric from it is passed to the thin tiers.

    python scripts/make_tier_inputs.py data/raw/floor_only out/floor_only/result.json data/tiers
"""
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
from matplotlib.path import Path as MplPath

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from roomscan.io.stray import estimate_rgb_offset, load_stray  # noqa: E402

ROT = {0: None, 90: "transpose=2", 180: "hflip,vflip", 270: "transpose=1"}


def upright_rotation(cap) -> int:
    """Degrees CCW to rotate the raw sensor image so world-up points to image-up."""
    up_cam = np.einsum("nji,j->ni", cap.poses[:, :3, :3], np.array([0, 1.0, 0])).mean(0)  # world up in cam (cv)
    # image-up direction in cv camera coords is -y
    cands = {0: np.array([0, -1, 0]), 90: np.array([1, 0, 0]), 180: np.array([0, 1, 0]), 270: np.array([-1, 0, 0])}
    return max(cands, key=lambda k: up_cam @ cands[k])


def rotate_img(img, deg):
    if deg == 90:
        return cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    if deg == 180:
        return cv2.rotate(img, cv2.ROTATE_180)
    if deg == 270:
        return cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    return img


def main(capture, lidar_result, out_root, max_per_room=8):
    capture, out_root = Path(capture), Path(out_root)
    name = capture.name
    cap = load_stray(capture)
    deg = upright_rotation(cap)
    off = estimate_rgb_offset(cap)
    # --- video tier
    vdir = out_root / "video"
    vdir.mkdir(parents=True, exist_ok=True)
    vout = vdir / f"{name}.mp4"
    if not vout.exists():
        vf = ROT[deg]
        # passthrough keeps the original (variable) frame timing; a CFR re-encode duplicated 30% of frames
        cmd = ["ffmpeg", "-v", "error", "-y", "-i", str(capture / "rgb.mp4"), "-map_metadata", "-1",
               "-fps_mode", "passthrough"]
        if vf:
            cmd += ["-vf", vf]
        cmd += ["-c:v", "libx264", "-crf", "20", "-preset", "veryfast", "-pix_fmt", "yuv420p", "-an", str(vout)]
        subprocess.check_call(cmd)
    # --- photo tier
    res = json.load(open(lidar_result))
    fr = res["plan_frame"]
    t = np.radians(fr["yaw_deg"])
    R = np.array([[np.cos(t), np.sin(t)], [-np.sin(t), np.cos(t)]])
    cam = cap.poses[:, :3, 3]
    fwd = cap.poses[:, :3, 2]
    cam_uv = cam[:, [0, 2]] @ R.T
    fwd_uv = fwd[:, [0, 2]] @ R.T
    yaw = np.degrees(np.arctan2(fwd_uv[:, 1], fwd_uv[:, 0]))
    pitch = np.degrees(np.arcsin(np.clip(fwd[:, 1], -1, 1)))
    # sharpness proxy: low angular speed
    dang = np.r_[0, np.degrees(np.arccos(np.clip(np.einsum("ni,ni->n", fwd[1:], fwd[:-1]), -1, 1)))]
    rooms = [r for r in res["rooms"] if r["floor_area"]["value"] >= 2.0]
    picks = {}
    for k, r in enumerate(rooms, start=1):
        P = MplPath(np.array(r["polygon"]))
        inside = np.nonzero(P.contains_points(cam_uv) & (dang < 1.5) & (np.abs(pitch) < 45))[0]
        if len(inside) < 2:
            continue
        # greedy max-spread in yaw (a person turning around the room), keep 6
        chosen = [inside[np.argmin(dang[inside])]]
        while len(chosen) < min(6, len(inside)):
            d = np.min(np.abs(((yaw[inside][:, None] - yaw[chosen][None, :]) + 180) % 360 - 180), axis=1)
            chosen.append(inside[np.argmax(d)])
        # doorway shots: frames inside this room whose view direction points at an opening
        door_c = []
        walls = {w["id"]: w for w in r["walls"]}
        for o in r["openings"]:
            if o["type"] == "window":
                continue
            w = walls[o["wall_id"]]
            a, b = np.array(w["start"]), np.array(w["end"])
            dvec = (b - a) / max(np.linalg.norm(b - a), 1e-9)
            door_c.append(a + dvec * (o["offset_from_wall_start_m"] + o["width"]["value"] / 2))
        for c in door_c[:2]:
            to = c - cam_uv[inside]
            dist = np.linalg.norm(to, axis=1)
            cosang = np.einsum("ni,ni->n", to, fwd_uv[inside]) / np.maximum(dist * np.linalg.norm(fwd_uv[inside], axis=1), 1e-9)
            score = cosang - 0.2 * np.abs(dist - 1.5)
            chosen.append(inside[np.argmax(score)])
        picks[f"room_{k}"] = sorted(set(int(i) for i in chosen))[:max_per_room]
    pdir = out_root / "photos" / name
    manifest = {}
    for folder, idxs in picks.items():
        d = pdir / folder
        d.mkdir(parents=True, exist_ok=True)
        expr = "+".join(f"eq(n\\,{int(cap.frame_ids[i]) - off})" for i in idxs)
        tmp = d / "_tmp"
        tmp.mkdir(exist_ok=True)
        subprocess.check_call(["ffmpeg", "-v", "error", "-y", "-i", str(capture / "rgb.mp4"), "-vf", f"select='{expr}'",
                               "-vsync", "0", "-q:v", "2", str(tmp / "%03d.jpg")])
        for j, (i, f) in enumerate(zip(idxs, sorted(tmp.glob("*.jpg")))):
            img = rotate_img(cv2.imread(str(f)), deg)
            cv2.imwrite(str(d / f"IMG_{j + 1:04d}.jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 92])
            f.unlink()
        tmp.rmdir()
        manifest[folder] = dict(source_frames=[int(cap.frame_ids[i]) for i in idxs],
                                lidar_room=rooms[int(folder.split("_")[1]) - 1]["id"])
    json.dump(dict(capture=str(capture), rotation_deg=deg, rgb_offset=off, video=str(vout), photo_folders=manifest,
                   note="derived inputs; geometry from the LiDAR result is NOT passed to these tiers"),
              open(out_root / f"{name}_tier_manifest.json", "w"), indent=1)
    print(f"{name}: rotation {deg} deg, video -> {vout}, photo folders {list(manifest)}")


if __name__ == "__main__":
    main(*sys.argv[1:4])
