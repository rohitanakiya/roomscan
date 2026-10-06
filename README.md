# roomscan — phone capture → dimensioned, stitched floor plan with damage scope

One command per capture. Three input tiers, one output contract:

| tier | input | how geometry is recovered |
|---|---|---|
| **LiDAR** | Stray Scanner export (depth + ARKit poses + intrinsics) | depth fusion → Manhattan wall faces → rooms; fragment pose-graph loop closure for drift |
| **video** | one walkthrough clip, nothing else | metric mono-depth + KLT/PnP odometry; focal from vanishing points; same loop closure |
| **photo** | 2–8 stills per room, one folder per room | metric mono-depth per photo; gravity+Manhattan frame; top-view correlation joins photos and rooms |

Output per capture (`out/<name>/`): `result.json` (schema: `schema/output.schema.json`), `plan.png` / `plan.svg`
(stitched whole-property plan). Every number carries a 95% interval and its error budget.

**Raw benchmark data** (three Stray Scanner captures, 0.9 GB, not in git): **[roomscan_raw_data on OneDrive](https://1drv.ms/f/c/ce3127bdc58443c9/IgDQs2EzfcLhTo4DRMCZvou1AWGadrdYRKJlT-uDvs5hwUg?e=EAO600)**
— contents and checksums in [docs/DATA.md](docs/DATA.md).

## Quick start (clean machine, < 15 min)

Requirements: Python 3.10–3.12 (open3d has no 3.13+ wheels on Windows). ffmpeg is used from PATH if present, otherwise
the copy bundled by the `imageio-ffmpeg` wheel in `requirements.txt` — nothing else to install. ~1.5 GB disk for dependencies.

```bash
git clone <this repo> roomscan && cd roomscan
python -m venv .venv && source .venv/bin/activate      # Windows: .venv\Scripts\activate
pip install -r requirements.txt                          # ~2–4 min (LiDAR tier needs only this)
# video/photo tiers: CPU PyTorch (~200 MB; avoids the 2.5 GB CUDA wheels on Linux/Windows)
pip install torch --index-url https://download.pytorch.org/whl/cpu && pip install "transformers>=4.45"
python scripts/fetch_weights.py                          # ~100 MB depth model
```

Run a capture — the tier is auto-detected:

```bash
python -m roomscan path/to/stray_export/            # LiDAR (folder with rgb.mp4, depth/, odometry.csv)
python -m roomscan path/to/walkthrough.mov          # video
python -m roomscan path/to/photos/                  # photo (sub-folder per room)
```

Verified on a clean Windows 11 laptop (Git Bash, Python 3.12, no system ffmpeg): the single-room LiDAR capture runs in
98 s and reproduces the benchmark output exactly.

Useful flags: `--out DIR`, `--tier lidar|video|photo`, `--no-drift` (ablation: raw ARKit poses),
`--no-damage`. Typical runtime on a 2-core laptop CPU: LiDAR 10 s (1 room) – 3 min (whole flat).

How to capture: **[docs/CAPTURE_PROTOCOL.md](docs/CAPTURE_PROTOCOL.md)** (one page, stock apps only).

## Reproduce every reported number

```bash
# download the three Stray exports from the shared folder linked in docs/DATA.md and unzip them to
# data/raw/single_room, data/raw/floor_only, data/raw/with_ceiling; checksums are in docs/DATA.md
python -m roomscan data/raw/<capture> --out out/bench/lidar/<capture>                                  # LiDAR plans
python scripts/make_tier_inputs.py data/raw/<capture> out/bench/lidar/<capture>/result.json data/tiers   # video/photo inputs
python bench/stage_damage.py data/raw/single_room data/staged/single_room_staged largest               # staged damage (furnished room)
python bench/calibrate_depth.py        # mono-depth scale vs LiDAR (per capture + leave-one-out), video/photo only
python bench/run_benchmark.py --force  # all tiers, all captures, ablations, comparisons
python bench/video_ablation.py         # video odometry components vs ARKit (optional, ~1 h on 2 cores)
python bench/make_report.py && bash scripts/build_report_pdf.sh   # report tables + PDF from the results
```

`bench/results/BENCHMARK.md` and `benchmark.json` are regenerated; the technical report's tables are copied from them.
Mono-depth outputs are cached by image hash in `out/.cache/depth/` (float16, replayed bit-identically); deleting the
cache re-runs the live model path.

## Repository map

```
src/roomscan/
  cli.py                one command per capture
  io/stray.py           Stray Scanner loader, RGB/depth frame-offset estimation
  tiers/lidar.py        LiDAR scene      tiers/video.py  video scene      tiers/photo*.py  photo scene
  geometry/             fusion, gravity/Manhattan frame, drift pose graph, rasters, rooms, walls, openings, cells
  analyze.py            scene -> rooms, adjacency, footprint (shared by all tiers)
  damage/               surface orthomosaics, detectors, concealed-damage rules, scope items
  measure.py            Measure: value + 95% interval + error budget
  export.py render.py   JSON contract, stitched plan
bench/                  benchmark runner, cross-capture registration/compare, damage staging + eval, fix loop
docs/                   capture protocol, device matrix, technical report, compliance matrix, fix loop
DECISIONS.md            design-decision log (each with the evidence that drove it)
```

## Honest limits (details in docs/TECHNICAL_REPORT.md)
- The sample data has **no laser ground truth**: LiDAR numbers are reported as repeatability between three
  captures of the same flat; video/photo numbers are reported against the LiDAR tier of the same capture.
- No consumer-app export exists for the sample rooms, so the head-to-head table cannot be filled from sample data.
- Video tier: the RGB-only camera path is still 1–3 m off ARKit over a multi-room walk (fast turns, doors);
  footprints land within −6 % to +40 % of LiDAR, not the ±3 % target (`bench/results/video_ablation.md`).
- Photo tier: the top-view correlation stitch stacks rooms onto each other; whole-property footprints are
  70–80 % short. Reported, not hidden: see the thin-tier table in `bench/results/BENCHMARK.md`.
- Mirrors / glass shower screens corrupt surface colour maps (damage detection there is unreliable).

Pretrained model disclosure: Depth Anything V2 Metric-Indoor Small (Apache-2.0), used by video/photo tiers only.
No call is made to any service of ours; the only network access is the one-off weight download.
