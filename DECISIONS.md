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

## D20. Never pair an image with the wrong pose
Damage views are extracted with one ffmpeg `select` call and paired with poses by order. Keyframe 0 maps to video
frame −1 (the encoder drops the first frame), so ffmpeg returned one image fewer than requested and every image
after it was paired with the previous keyframe's pose — a one-frame shift that blurred thin features (the staged
crack disappeared from the orthomosaic). Frames without a video frame are now dropped up front and the
extraction count is asserted. Same guard added to tier-input derivation. Staged damage: recall 2/2.

## D21. A cache must know what produced it
The first benchmark after D20 still scored the staged crack as missed, while fresh standalone runs found it.
Cause: `--force` re-ran the pipeline but each run folder kept its `frames_640/` images, extracted before the
D20 fix, so the images were still paired with the wrong poses. Two guards: `run_benchmark.py --force` now deletes a run's outputs
(including damage caches) before running, and the frame cache stores a manifest (video path, size, mtime, frame
offset, width, `CACHE_VERSION`) and rebuilds itself on any mismatch. Clean re-run: staged damage recall 2/2
(stain area +6.7 %, inside its 95 % interval; crack length −17.5 %), 1 false positive; geometry numbers unchanged.

## D22. Video tier: measure the camera path, then fix what the measurement shows
With the depth weights available, the first video runs gave plausible totals but wrong rooms. ARKit poses are
never given to the video tier, but they can score its RGB-only path afterwards (`bench/video_odometry.py`,
absolute trajectory error after rigid alignment). Findings, each one measured:
1. **Depth scale.** Depth Anything V2 metric-indoor reads 1.26–1.35× too far on these iPhone frames (stable per
   capture, p10–p90 1.06–2.05 per frame). One number per device fixes the bias: `bench/calibrate_depth.py`
   fits it on LiDAR frames and ships it (`roomscan/depth_calibration.json`, 0.766). The benchmark scores every
   capture with the scale fitted on the *other two* (leave-one-out residual ±5 %), so no capture is scored with
   a number fitted on itself.
2. **Fast turns.** Tracked steps are accurate (rotation error 0.9° median), but 17 % of 0.3 s steps lose every
   KLT track when the camera turns 17–56°. Those gaps were bridged by constant velocity. They are now
   re-decoded at the native frame rate and the rotation chained frame to frame (`gap_rotation`): floor_only
   ATE 5.4 → 1.5 m.
3. **Absolute rotation** from straight image lines (vertical vanishing direction + yaw scan + Gauss-Newton;
   the mono-depth normals are not square enough to define the frame — even ARKit's true rotation is "corrected"
   by 8–18° against them). Open doors and close-up fixtures mislead it; acceptance rules (vertical support,
   both wall axes for large yaw corrections) help, but on floor_only it still nearly doubles ATE (1.47 → 2.68 m). Kept, **off by
   default** (`ROOMSCAN_VIDEO_MW=1`), as are trusted-only fusion and a floor-normal veto, which made things worse.
The default was chosen from `bench/results/video_ablation.md` by mean ATE over the captures every configuration
ran on — the same captures the benchmark reports (there is no held-out video). Honest status: the video tier's
camera path is still metres off on a multi-room walk; its plans do not meet the ±3 % gate.

## D23. Robustness fixes found by the thin tiers
- A photo-tier room outline self-intersected after face snapping (shapely refused it in comparison). Outlines are
  now simplified with growing minimum edge until valid; `compare.py` also repairs geometry defensively.
- The video tier held every 10 fps frame in colour plus float32 depth: the 3.5 min walkthrough was killed
  for memory (8 GB machine). Frames are now greyscale in memory, colour is read from disk for depth keyframes
  only, and depth is float16.
- Photo inputs were regenerated from the final LiDAR plans and the post-D20 extractor before scoring.

## D24. Thin-tier intervals from measurement, not from hope
The video/photo error models were priors (2 % / 3.5 % scale). The measurements say otherwise: the depth-scale
residual after leave-one-out calibration reaches 5.2 %, and the RGB-only camera path is off by 2.7–8 % of the
distance walked. Video: scale 5 %, drift 1.5 %/m of path, face 3 cm. Photo: scale 5 %, face 4 cm. Registration
failure in the photo tier (rooms stacked on each other) is *not* absorbed into an interval — the benchmark scores
it and the report states it. Whether the LiDAR value falls inside the thin tier's 95 % interval is reported
per capture (`footprint_lidar_in_ci`, `wall_ref_in_ci_pct`): that is the calibration score at every tier.

## D25. Same input, same plan
Two video runs of the same clip with the same code gave 1 room vs 2 rooms (trajectories split at keyframe 8):
OpenCV's RANSAC draws from a random generator whose state was not fixed, and the video tier amplifies one different
inlier set into a different room split. `cv2.setRNGSeed(0)` at the start of each thin-tier run (and one OpenCV
thread in the photo tier, whose parallel RANSAC draws from per-thread generators) makes runs bit-identical —
checked with four simultaneous video runs and three photo runs under load. The video ablation was re-run after
this fix; the earlier, unseeded table is not used anywhere. Lesson recorded in the report: the video tier's
output is *sensitive* to small perturbations, which is itself a finding about its reliability.
Second source, found the same way: each run kept its own depth cache, and one cache had been filled on a different
host before a session restart. CPU inference is bit-identical only for the same thread count and CPU kernels;
on different ones ~0.02 % of float16 depths differ — and that alone moved a photo-tier footprint from 7.73 to 7.11 m².
Now there is one cache per checkout (`out/.cache/depth`, the cached outputs replay bit-identically) and torch runs
on a pinned thread count (`ROOMSCAN_TORCH_THREADS`, default 2). All thin-tier numbers were recomputed from an empty
cache on one host. The photo stitch's sensitivity to perturbations this small is reported as a limitation.


## D26. The clean-machine test found a missing dependency
Running the README on the Windows laptop: `ffmpeg: command not found`. The README said "ffmpeg on PATH", which a
reviewer would have had to go and install. Every ffmpeg call now goes through `roomscan.ffmpeg_bin.ffmpeg_exe()`:
the system ffmpeg if there is one, else the static build from the `imageio-ffmpeg` wheel (added to
requirements). A dead `ffprobe` helper was removed (ffmpeg auto-rotates on decode), so no ffprobe is needed.
Checked: the LiDAR single_room run with only the bundled ffmpeg reproduces the benchmark (23.118 m², 2 damage regions).
