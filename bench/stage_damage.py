"""Digitally stage damage into a real capture (sample-data substitute for physically staged damage).

We cannot stage physical damage in a property we have no access to, so we paint it *onto the
3D wall surface* and re-render it into every video frame through the capture's own poses,
intrinsics and LiDAR depth (occlusion-tested). The damage therefore behaves like real paint on a
wall: it is geometrically consistent across views, foreshortens, and is hidden by furniture.
Ground truth (class, area, length, surface) is known exactly and written to staged_truth.json.

    python bench/stage_damage.py data/raw/single_room data/staged/single_room_staged
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from roomscan.analyze import analyze  # noqa: E402
from roomscan.io.stray import estimate_rgb_offset, load_stray  # noqa: E402
from roomscan.tiers.lidar import build_scene  # noqa: E402
from roomscan.ffmpeg_bin import ffmpeg_exe  # noqa: E402

PX = 0.004  # texture resolution on the wall (m / px)


def stain_texture(w_m, h_m, seed=0):
    rng = np.random.default_rng(seed)
    W, H = int(w_m / PX), int(h_m / PX)
    yy, xx = np.mgrid[0:H, 0:W]
    cx, cy = W / 2, H / 2
    r = np.hypot((xx - cx) / (W / 2), (yy - cy) / (H / 2))
    noise = cv2.GaussianBlur(rng.standard_normal((H, W)).astype(np.float32), (0, 0), W / 12)
    noise /= np.abs(noise).max() + 1e-9
    shape = r + 0.25 * noise
    alpha = np.clip((1.0 - shape) / 0.15, 0, 1)            # soft edge
    tide = np.exp(-((shape - 0.92) / 0.05) ** 2) * 0.6       # darker tide-mark ring
    a = np.clip(alpha * 0.55 + tide * (shape < 1.05), 0, 0.85)
    color = np.array([60, 120, 165], np.float32)            # BGR yellow-brown
    return a.astype(np.float32), color


def crack_texture(length_m, seed=1):
    rng = np.random.default_rng(seed)
    W, H = int(0.10 / PX), int(length_m / PX)
    img = np.zeros((H, W), np.uint8)
    x = W // 2
    pts = []
    for y in range(0, H, 6):
        x = int(np.clip(x + rng.integers(-3, 4), 4, W - 5))
        pts.append((x, y))
    cv2.polylines(img, [np.array(pts, np.int32)], False, 255, 2, cv2.LINE_AA)
    a = img.astype(np.float32) / 255.0 * 0.9
    return a, np.array([35, 35, 40], np.float32)


def wall_frame(wall, plan_frame, floor_y):
    """Return origin (world), u-axis (along wall, start->end), v-axis (up), for a wall face."""
    a, b = np.array(wall["start"]), np.array(wall["end"])
    R = plan_frame.R
    aw, bw = a @ R, b @ R          # plan (u,v) -> world (x,z)
    o = np.array([aw[0], floor_y, aw[1]])
    d = np.array([bw[0] - aw[0], 0, bw[1] - aw[1]])
    d /= np.linalg.norm(d)
    return o, d, np.array([0, 1.0, 0])


def main(src, dst, room_id=None, truth_only=False):
    src, dst = Path(src), Path(dst)
    cap = load_stray(src)
    scene = build_scene(src, drift_correction=False, log=lambda *a: None)
    prop = analyze(scene, log=lambda *a: None)
    from roomscan.export import property_to_json

    res = property_to_json(prop, scene)
    if room_id == "largest":
        # the furnished living room (sofa, wardrobe) is the largest room of this capture; the case study asks for
        # damage staged in a furnished room
        room = max(res["rooms"], key=lambda r: r["floor_area"]["value"])
    else:
        room = next(r for r in res["rooms"] if (r["id"] == room_id if room_id else r["primary"]))
    walls = sorted([w for w in room["walls"] if w["face_observed"] and w["length"]["value"] > 1.2],
                   key=lambda w: -w["face_points"])
    W1, W2 = walls[0], walls[1]
    F = prop.frame
    patches = [
        dict(id="GT1", cls="water_stain", wall=W1, along=min(1.0, W1["length"]["value"] / 2), h=0.40,
             size=(0.42, 0.32), tex=stain_texture(0.42, 0.32)),
        dict(id="GT2", cls="crack", wall=W2, along=min(0.9, W2["length"]["value"] / 2), h=1.35,
             size=(0.10, 0.60), tex=crack_texture(0.60)),
    ]
    truth = []
    for p in patches:
        a, col = p["tex"]
        area = float((a > 0.25).sum() * PX * PX) if p["cls"] != "crack" else None
        o, du, dv = wall_frame(p["wall"], F, F.floor_y)
        p.update(o=o, du=du, dv=dv)
        centre_w = o + du * p["along"] + dv * p["h"]
        truth.append(dict(id=p["id"], cls=p["cls"], room=room["id"], wall=p["wall"]["id"],
                          centre_world=[round(float(x), 4) for x in centre_w],
                          centre_along_m=p["along"], centre_h_m=p["h"], width_m=p["size"][0], height_m=p["size"][1],
                          area_m2=area, length_m=0.60 if p["cls"] == "crack" else None))
    off = estimate_rgb_offset(cap)
    dst.mkdir(parents=True, exist_ok=True)
    if truth_only:
        old = json.load(open(dst / "staged_truth.json"))
        old["truth"] = truth
        json.dump(old, open(dst / "staged_truth.json", "w"), indent=1)
        print("truth rewritten")
        return
    for name in ["depth", "confidence", "odometry.csv", "camera_matrix.csv", "imu.csv"]:
        t = dst / name
        if not t.exists():
            os.symlink((src / name).resolve(), t)
    K = cap.K_rgb
    w, h = 1920, 1440
    reader = subprocess.Popen([ffmpeg_exe(), "-v", "error", "-i", str(src / "rgb.mp4"), "-vsync", "0", "-f", "rawvideo", "-pix_fmt",
                               "bgr24", "-"], stdout=subprocess.PIPE)
    writer = subprocess.Popen([ffmpeg_exe(), "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{w}x{h}",
                               "-r", "60", "-i", "-", "-c:v", "libx264", "-crf", "18", "-preset", "veryfast",
                               "-pix_fmt", "yuv420p", str(dst / "rgb.mp4")], stdin=subprocess.PIPE)
    n = 0
    N = len(cap.poses)
    painted = 0
    while True:
        buf = reader.stdout.read(w * h * 3)
        if len(buf) < w * h * 3:
            break
        img = np.frombuffer(buf, np.uint8).reshape(h, w, 3).copy()
        i = min(max(n + off, 0), N - 1)
        T = cap.poses[i]
        Tcw = np.linalg.inv(T)
        dep = None
        for p in patches:
            a, col = p["tex"]
            th, tw = a.shape
            # texture pixel (x,y) -> world: o + du*(along - W/2 + x*PX) + dv*(h + H/2 - y*PX)
            x0 = p["along"] - tw * PX / 2
            y0 = p["h"] + th * PX / 2
            corners_t = np.array([[0, 0], [tw, 0], [tw, th], [0, th]], np.float32)
            Xw = np.array([p["o"] + p["du"] * (x0 + cx * PX) + p["dv"] * (y0 - cy * PX) for cx, cy in corners_t])
            Xc = Xw @ Tcw[:3, :3].T + Tcw[:3, 3]
            if (Xc[:, 2] < 0.2).any():
                continue
            uv = (Xc[:, :2] / Xc[:, 2:]) * [K[0, 0], K[1, 1]] + K[:2, 2]
            if (uv[:, 0].max() < 0) or (uv[:, 0].min() > w) or (uv[:, 1].max() < 0) or (uv[:, 1].min() > h):
                continue
            Hm = cv2.getPerspectiveTransform(corners_t, uv.astype(np.float32))
            aw = cv2.warpPerspective(a, Hm, (w, h), flags=cv2.INTER_LINEAR)
            if aw.max() <= 0:
                continue
            # occlusion: expected plane depth vs LiDAR depth
            if dep is None:
                dep = cv2.resize(cap.load_depth(int(cap.frame_ids[i]), min_conf=0), (w, h),
                                 interpolation=cv2.INTER_LINEAR)
            nrm = np.cross(p["du"], p["dv"])
            nc = Tcw[:3, :3] @ nrm
            pc = Tcw[:3, :3] @ p["o"] + Tcw[:3, 3]
            ys, xs = np.nonzero(aw > 0.01)
            rays = np.c_[(xs - K[0, 2]) / K[0, 0], (ys - K[1, 2]) / K[1, 1], np.ones(len(xs))]
            zp = (nc @ pc) / (rays @ nc)
            vis = np.abs(dep[ys, xs] - zp) < 0.06
            al = np.zeros((h, w), np.float32)
            al[ys[vis], xs[vis]] = aw[ys[vis], xs[vis]]
            if p["cls"] == "water_stain":
                tint = img.astype(np.float32) * (col / 255.0) * 1.15
            else:
                tint = np.broadcast_to(col, img.shape).astype(np.float32)
            img = (img * (1 - al[..., None]) + tint * al[..., None]).clip(0, 255).astype(np.uint8)
            painted += 1
        writer.stdin.write(img.tobytes())
        n += 1
    writer.stdin.close()
    writer.wait()
    reader.wait()
    json.dump(dict(source=str(src), rgb_offset=off, frames=n, frames_with_damage=painted, truth=truth,
                   note="digitally staged damage, rendered on the 3D wall plane through capture poses"),
              open(dst / "staged_truth.json", "w"), indent=1)
    print(f"staged {len(truth)} damage items into {n} frames ({painted} patch renders) -> {dst}")


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    main(*args, truth_only='--truth-only' in sys.argv)
