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

## D6. The three sample captures are one property
Correlating Manhattan-aligned wall rasters: floor_only vs with_ceiling peak NCC 0.53 at a 90°
rotation (next best 0.31); with_ceiling vs single_room 0.43. So the benchmark can measure
LiDAR repeatability across captures of the same rooms, and the LiDAR consensus is the
reference the video and photo tiers are scored against (no laser ground truth exists for the
sample data — stated plainly in the report).

## D7. Video tier: classical SfM rejected, depth-driven odometry instead
COLMAP (SIFT, sequential matching) on the single-room walkthrough at 3 fps registered 44/112
frames split across 3 models: white walls. Metric mono-depth + KLT/PnP odometry with ICP
fallback avoids both the texture problem and SfM's scale ambiguity. Focal length is not
known for a plain video: a Manhattan vanishing-point estimate gave 536 px vs the true 533 px
(0.6%) on the sample clip.

## D8. Visibility uses ceiling returns too
The ceiling-aimed capture sees little below 2.4 m; excluding ceiling points from the visibility
fans starved free space and dropped the corridor. A ray to a ceiling point still crossed free
plan space, so all returns up to 4 m now count.

## D9. RGB/depth frame offset is estimated, not assumed
Stray's rgb.mp4 decodes to N-1 frames; video frame n shows the scene of depth row n+1.
Detected by RGB-edge / depth-discontinuity overlap (score 0.80 at +1 vs 0.48 at 0). Before the
fix, reprojection error between frames was 18–32 px and damage masks landed on wrong surfaces.

## D10. Damage detection on surface orthomosaics, not on images
Per-image detection + multi-view voting: 89 false positives on a clean apartment (skirting,
tile grout, shadows). Detecting on a per-surface orthomosaic (median of the 15 most frontal
views, 1 cm grid) makes the background surface-local and metric, and excludes skirting / cornice
/ corner bands and floors. Clean capture: 89 -> 3 detections. Staged-damage recall is the open
problem: the staged stain sits behind a glass shower screen, where the orthomosaic is dominated
by reflections. Known failure mode, reported.

## D11. Digitally staged damage (sample-data substitute)
No physical access → damage is painted on the 3D wall plane and re-rendered into every frame
through the capture's own poses/intrinsics with LiDAR occlusion testing; truth is exact.

## D12. Photo tier registration = top-view correlation in gravity+Manhattan frame
SIFT+PnP linked 28 sample photos into 19 blocks; FPFH+RANSAC gave >0.5 m errors on 30/42
pairs (white walls, repeated planes). With metric depth each photo yields gravity, Manhattan
directions and a top-view structure map; the residual unknown (4 yaws × 2-D shift) is solved
exhaustively by FFT correlation. Status: runs end-to-end and places every room, but with
oracle (LiDAR) depth the single-room set still under-estimates room area by ~55% — single
photos see too little of each room. Intervals for this tier are calibrated to that error.

## D13. Hybrid room geometry
Room identity from the watershed (door-width splitting); room geometry from the wall-line
arrangement (cells snapped to fitted faces). Pure cell decomposition merged rooms through
unobserved wall stretches (floor-only capture became one 61 m² room).

## D14. Video odometry: track at 10 fps, depth at keyframes, KLT chains
3 fps tracking lost tracks on turns (per-step error ≈ step size). Chained KLT at ~10 fps with
forward-backward checks between depth keyframes (every 3rd frame) gives cm-level PnP steps
(oracle depth: 0.1–1.5 cm error per 0.3 s step). Steps with no surviving tracks fall back to
depth ICP seeded with constant velocity, else constant velocity. Oracle-depth test (LiDAR
depth on the video frames — a dev-only test of the odometry, not a reported number): path
13.9 m vs ARKit 14.5 m; main room area −3.9%.
Also: the raw Stray video is VFR (avg 46 fps); a CFR re-encode duplicated 30% of frames and
broke any index-based alignment — tier inputs now keep `-fps_mode passthrough`.

## D15. Intervals calibrated to demonstrated repeatability (not to the sensor spec)
With the face-fit + sensor + scale + drift budget alone, only 45–57% of repeat-capture wall differences fell
inside the combined 95% intervals — confidently wrong. `bench/calibrate.py` fits a per-face residual term on
wall pairs from repeat captures (gross mismatches > 30 cm — a different wall, not noise — excluded and reported),
validated leave-one-pair-out: fitted face_floor = 6 cm, held-out coverage 100% / 80% / 89%. LiDAR wall intervals
are therefore ±~17 cm (95%) — the repeatability we can demonstrate on this data, not the ±1–2 cm the sensor could
deliver with better pose consistency.

## D16. Drift correction re-tuned on a wall-sharpness metric
Loop residual alone was misleading: v0.1 cut it from ~10 to ~3 cm yet made walls *less* sharp than raw ARKit
(0.517 vs 0.530; 0.368 vs 0.381), because piecewise-constant fragment corrections step at boundaries. Adopted:
8-keyframe fragments, loop pairs up to 4 m apart, corrections blended between anchors — sharper than raw poses on
both captures (0.564 vs 0.530, 0.428 vs 0.381). Table: bench/drift_tuning.md. Wall sharpness is now reported per
run so the drift ablation shows it directly.

## D17. Door vs window by what lies beyond, not by what is below
A gap with see-through was a door only if nothing was on the wall plane below 0.9 m; the ceiling-aimed capture
never sees that band, so every door became a "window". Now: another room of the plan beyond the gap -> door;
else floor continuing beyond the gap -> door; wall below the gap or no floor beyond -> window. floor_only went
from 1 to 4 room-to-room doors (all 5 rooms connected).

## D18. A wall face may not sit inside another room
The contract test caught rooms overlapping by up to 1.75 m² after the "outermost face" rule (iteration 2) pushed
edges past thin walls into the neighbour. Candidates whose room-side neighbourhood lies in another room's labelled
cells are now rejected. Residual overlap: 0.09 / 0.34 / 0.84 m² (single / floor / ceiling capture), reported per
run as `room_overlap_m2`.

## D19. Staged damage moved to the furnished room; damage detector fixes
The case study asks for damage staged in a *furnished* room; the first staging used the "primary" room, which
was the bathroom (glass screen + mirror), and recall was 0/2. Re-staged on the two best-observed walls of the
largest room (sofa, wardrobe). Three detector bugs found on the way: (1) crack width was computed in the wrong
units (every crack rejected); (2) the grout-line filter rejected any straight-ish vertical crack — replaced by
max deviation from the best-fit line (cracks wander ≥ 1.2 cm); (3) stain area used only the dark core —
hysteresis growth into the diffuse halo (area error 54% → 7%). Result: recall 2/2 (stain area +7%, inside its
interval, CD-02 fires; crack length −17%), 1 false positive on the staged capture. The bathroom staging is kept
in data/staged/single_room_staged_bathroom as a documented failure case (glass/mirror).
