# Design decisions log

Running log of non-obvious choices, newest last. Each entry: decision, why, evidence.

## D1. Capture route = Route 2 (stock app protocol)
Stray Scanner (free, App Store) for the LiDAR tier; built-in Camera app for video/photo tiers.
Why: the sample data is already Stray Scanner format; no iOS build/TestFlight overhead; a
non-engineer can install it in < 2 min.

## D2. Pose convention: Stray odometry is already OpenCV camera axes
ARKit's raw camera frame is (+x right, +y up, +z back). Applying the ARKit->OpenCV flip
produced a smeared cloud (axis-aligned-normal fraction 3%, y-extent 3.7 m). Using the
quaternion as-is with OpenCV back-projection gives a clean cloud (floor peak at a single
height, Manhattan wall histogram concentrated, 48% of horizontal normals within ±3° of one
direction). Stray's own visualiser does the same. World frame stays ARKit's: +y = gravity up.

## D3. Sample-data constraint
All three tiers are derived from the three provided Stray captures:
LiDAR = everything; video = rgb.mp4 only (depth, poses, intrinsics dropped);
photo = 2–8 stills per room extracted from the video, no metadata. Disclosed in report.
