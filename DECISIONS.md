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

## D4. Room segmentation = visibility free-space + wall evidence + watershed
Floor-point occupancy was too sparse (beds/furniture hide floor; camera aimed low).
Wall-enclosure flood fill leaked through unobserved wall stretches. What works: per keyframe,
a 2D visibility fan (camera -> farthest return per 1° azimuth) marks free space; walls are
cells whose vertical points span > 0.9 m of height (furniture rarely does). Interior = free −
walls; rooms = watershed on the distance transform seeded by cores that survive erosion by
0.42 m (so passages narrower than ~0.84 m — doors — split rooms; corridors don't vanish).
Regions with < 35% of their boundary on walls are ray leaks through doors/windows → dropped.

## D5. Wall face = first strong face outward from the raster edge
The raster boundary sits at or inside the wall. Taking the strongest histogram peak made edges
jump across thin walls to the neighbour's face; taking the first face met going outward fixes
that. Faces are fitted from raw points (robust median, MAD clipping), so dimensions do not
inherit the 2 cm raster quantisation.
