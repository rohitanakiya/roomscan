# Device matrix

Which tier runs on which hardware, and what accuracy each tier honestly delivers on our benchmark.
Accuracy columns are filled from `bench/results/benchmark.json` (see BENCHMARK.md for the run that produced them).
"Reference" = what the number is measured against; the sample data has no laser ground truth.

| tier | capture hardware | capture app | inputs used | processing hardware | wall length (median abs. error) | reference | 95% interval half-width (typical) |
|---|---|---|---|---|---|---|---|
| LiDAR | iPhone 12 Pro / 13 Pro / 14 Pro / 15 Pro / 16 Pro (incl. Max), iPad Pro 2020+ | Stray Scanner (free) | depth 256×192, confidence, ARKit poses, intrinsics, RGB | any x86/ARM laptop, CPU only, 4 GB RAM | see BENCHMARK.md (repeatability rows) | another capture of the same rooms | ±1.5–3 cm walls, ±0.4 m² rooms |
| video | any iPhone 15 or newer (Pro not required) | built-in Camera | RGB video only | CPU (≈1 s per depth keyframe; 4–13 min per capture on 2 cores), 6 GB RAM | footprint +38 / −26 / −6.5 % vs LiDAR; camera path 1.1–4.8 m RMS off ARKit (2.7–7.8 % of path) | LiDAR tier of the same capture | footprint ±30 % (1 room) to ±100 % (whole flat, lower bound at 0): scale 5 %, drift 1.5 %/m walked — measured, D24 |
| photo | any iPhone 15 or newer | built-in Camera | 2–8 stills per room, EXIF focal if present | CPU, 6 GB RAM | footprint −69 / −69 / −81 % vs LiDAR (rooms stacked by the stitch) | LiDAR tier of the same capture | ±25 % footprint; does **not** cover stitch failure (LiDAR inside interval 0/3) |

Notes
- iPhone 15 / 15 Plus / 16 / 16 Plus (non-Pro) have **no LiDAR**: only the video and photo tiers run on them.
- Low light: LiDAR is unaffected (active sensor); video/photo mono-depth degrades, intervals widen (error model per tier in `tiers/*.py`).
- Mirrors / glass: LiDAR returns the mirrored room behind the glass (phantom free space); openings whose far side is a
  mirror image of the room are flagged in `warnings`. Mono-depth treats mirrors as windows.
- Video/photo depth scale: the mono-depth model reads 26–35 % too far on iPhone frames; one correction per device
  model is fitted against a LiDAR iPhone once (`bench/calibrate_depth.py`, shipped value 0.766).
