# Compliance matrix

Status legend: **Done** = implemented and exercised by the benchmark; **Partial** = implemented, with a stated gap;
**N/A-data** = cannot be produced from the provided sample data (reason given); **Fail** = implemented, gate not met
(number in BENCHMARK.md).

| # | Requirement (case study) | File path(s) | Artifact | Status |
|---|---|---|---|---|
| 1 | Capture route: TestFlight build **or** one-page stock-capture protocol | `docs/CAPTURE_PROTOCOL.md` | Route 2 protocol (Stray Scanner + Camera app) | Done |
| 2 | Device matrix: tier → hardware → honest accuracy | `docs/DEVICE_MATRIX.md`, `bench/results/BENCHMARK.md` | table | Done |
| 3 | Photo tier: 2–8 stills/room, any iPhone 15+, no depth/poses | `src/roomscan/tiers/photo.py`, `tiers/photo_reg.py` | `python -m roomscan <photo_root>` | Partial (runs end-to-end on all 3 captures, JPEG or HEIC; footprints 69–81 % short — stitch stacks rooms, which the tier now detects and flags itself, D27) |
| 4 | Photo tier produces the **stitched whole-property plan** | `tiers/photo_reg.py::register_blocks`, `render.py` | `plan.png` per photo run | Partial (a stitched plan is produced and self-checked; on our captures it fails the check and says so — see row 27) |
| 5 | Video tier: handheld walkthrough clip | `src/roomscan/tiers/video.py`, `bench/video_odometry.py`, `bench/video_ablation.py` | `python -m roomscan <video>` | Done (runs on all 3 captures; camera path scored vs ARKit; ablation of odometry components) |
| 6 | LiDAR tier: depth, poses, intrinsics | `src/roomscan/tiers/lidar.py`, `io/stray.py` | `python -m roomscan <stray_export>` | Done |
| 7 | Same output contract from every tier | `src/roomscan/scene.py`, `analyze.py`, `export.py`, `schema/output.schema.json` | JSON validated by `tests/test_contract.py` | Done |
| 8 | Intervals widen honestly as sensor data thins | `tiers/*.py` (`*_ERRORS`), `measure.py`, `bench/calibrate_depth.py` | `error_budget_1sigma` per number | Done (LiDAR face 4 mm + 6.5 cm calibrated; video/photo set from measured depth-scale and odometry error, D24) |
| 9 | Dimensioned per-room plan: walls, ceiling height, floor area, openings | `geometry/room.py`, `geometry/openings.py` | `rooms[].walls/ceiling_height/floor_area/openings` | Done (ceiling *observed* only when the capture looks at it; else flagged prior) |
| 10 | Stitched multi-room plan with correct adjacency | `analyze.py` (adjacency), `geometry/openings.py`, `render.py` | `property.adjacency`, `plan.png/svg` | Partial (floor-aimed capture: all rooms linked by doors; ceiling-aimed capture: doors often unconfirmed) |
| 11 | Per-surface damage regions with class and metric extent | `damage/pipeline.py`, `damage/detect.py` | `rooms[].damage[]` (surface_id, class, area Measure, bbox) | Done (staged: 2/2 found, stain area +7% in-interval; fails behind glass/mirror — documented) |
| 12 | Concealed-damage flags with the rule that fired | `damage/rules.py` | `rooms[].concealed_damage_flags[]` (rule_id + text + evidence) | Done |
| 13 | Scope line items keyed to surfaces | `damage/rules.py` | `rooms[].scope_items[]` | Done |
| 14 | Confidence interval on every measurement | `measure.py`, `export.py` | `ci95` on every Measure | Done |
| 15 | One command per capture | `src/roomscan/cli.py` | `python -m roomscan <path>` | Done |
| 16 | JSON to the published schema | `schema/output.schema.json` | schema + contract test | Done (own schema: the case study's "published schema" was not provided) |
| 17 | Rendered plan | `src/roomscan/render.py` | `plan.png`, `plan.svg` | Done |
| 18 | Benchmark: multi-room capture, ≥3 rooms + connector | `data/raw/floor_only`, `data/raw/with_ceiling` | sample captures (5 rooms + corridor) | Done (sample data) |
| 19 | Benchmark: furnished room with staged damage, two classes | `bench/stage_damage.py`, `data/staged/single_room_staged/staged_truth.json` | digitally staged water stain + crack in the furnished room | Partial (digital staging, not physical) |
| 20 | Same rooms at all three tiers, multi-room set included | `scripts/make_tier_inputs.py`, `data/tiers/` | derived video + per-room photo folders | Done (derived from the LiDAR captures; disclosed; all 3 tiers benchmarked) |
| 21 | ≥1 room captured twice at the same tier | three LiDAR captures of the same flat | `bench/compare.py`, repeat:* rows | Done |
| 22 | Laser/tape ground truth on everything | — | — | **N/A-data** (no access to the property) |
| 23 | Gate: opening widths ≤2 cm on ≥85%, misses/phantoms scored | `bench/run_benchmark.py` | `opening_within_2cm_pct` (cross-capture) | see BENCHMARK.md |
| 24 | Gate: ceiling ≤1.5 cm; repeat spread ≤1 cm; biased vs unrepeatable stated | `bench/run_benchmark.py`, report §5 | ceilings table | Partial (only one capture observes ceilings) |
| 25 | Gate: repeatability ≤1 cm or 0.5% per wall | `bench/compare.py` | `wall_repeatable_pct` | see BENCHMARK.md |
| 26 | Gate: drift accountability + on/off ablation of stitched footprint | `geometry/drift.py`, `--no-drift`, report §3, `bench/drift_tuning.md` | ablation rows `*/no_drift` (footprint, wall sharpness, loop residual) | Done |
| 27 | Gate: photo-tier whole-property stitch, no overlaps, footprint ±8% | `tiers/photo_reg.py` | `photo_vs_lidar:*` rows | **Fail** (footprint −69 / −69 / −81 %; overlaps 0–0.37 m²); the failure is detected by the tier itself and the intervals cover the LiDAR footprint 3/3 (D27) |
| 28 | Photo ±8% / video ±3% wall lengths, calibration scored at every tier | `bench/compare.py` (`a_in_b_ci`), `run_benchmark.py` (`footprint_lidar_in_ci`) | `wall_ref_in_ci_pct`, `footprint_lidar_in_ci` | **Fail** on accuracy (video footprint +38 / −26 / −6.5 %); calibration scored at every tier: LiDAR footprint inside the video interval 3/3, photo 3/3 after the stitch self-check (0/3 before, D27) |
| 29 | Head-to-head vs consumer app on 2 rooms | `docs/TECHNICAL_REPORT.md` §8 | — | **N/A-data** (needs a consumer-app scan of the same rooms) |
| 30 | Fix loop: declaration, shipped fix, regenerable before/after, diff | `docs/FIX_LOOP.md`, `bench/fix_loop/` | before/after runs + diff | see FIX_LOOP.md |
| 31 | Process evidence: commit history | `git log` | incremental commits since hour 0 | Done |
| 32 | README to running on a fresh capture < 15 min, clean machine | `README.md`, `src/roomscan/ffmpeg_bin.py` | install + run | Done (verified on a Windows 11 laptop: fresh Python 3.12 venv, `pip install -r requirements.txt`, no system ffmpeg; single_room LiDAR in 98 s, output identical to the benchmark — 4 rooms, 23.12 m², 2 damage regions; D26) |
| 33 | Reproduction bundle: regenerate every number from raw inputs | `bench/run_benchmark.py`, `scripts/` | `BENCHMARK.md` | Done |
| 34 | Cached model outputs replay deterministically; live path also runs | `models/depth.py` (hash-keyed float16 cache, pinned threads), seeded RANSAC | `out/.cache/depth` | Done (bit-identical reruns verified, D25) |
| 35 | Technical report ≤ 6 pages | `docs/TECHNICAL_REPORT.md` | report | Done |
| 36 | Raw benchmark data: sensor logs, ground truth, app exports | `data/` manifest `docs/DATA.md` | Stray captures + derived inputs + staged truth | Partial (no GT/app exports exist) |
| 37 | Mirrors, glass, wet-look surfaces, low light covered | report §7, `docs/DEVICE_MATRIX.md` | failure-mode analysis | Done (analysis); detection partial |
| 38 | Weights / large binaries fetched by script | `scripts/fetch_weights.py`, `.gitignore` | — | Done |
| 39 | Runs without calling our infrastructure; models disclosed | README "Pretrained model disclosure" | — | Done |
| 40 | Walk-in readiness | `docs/WALK_IN.md` | runbook | Done |
