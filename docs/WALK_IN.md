# Walk-in runbook (defense day)

Goal: an unseen space, their iPhone, their choice of tier, our protocol followed literally, our pipeline cold.

## Before the session (laptop)
```bash
cd roomscan && source .venv/bin/activate      # Windows Git Bash: source .venv/Scripts/activate
python -m roomscan data/raw/single_room --no-damage --out out/warmup    # ~20 s smoke test (LiDAR)
python scripts/fetch_weights.py                                          # video/photo weights present?
```
Have `docs/CAPTURE_PROTOCOL.md` printed (one page) — hand it over, do not explain beyond it.

## Hand-off from the phone
| tier | what arrives | where to put it |
|---|---|---|
| LiDAR | Stray Scanner → recording → Export → "Export all" → AirDrop: a folder | `walkin/lidar_<n>/` |
| video | Camera clip via AirDrop ("All Photos Data" on) | `walkin/video_<n>.mov` |
| photo | photos AirDropped, sorted into one folder per room | `walkin/photos_<n>/<room>/` |

## Run (one command, tier auto-detected)
```bash
python -m roomscan walkin/<capture> --out out/walkin_<n>
open out/walkin_<n>/plan.png        # Windows: start out/walkin_<n>/plan.png — stitched plan, every wall labelled
```
Measured times: LiDAR 1 room 98 s with damage on the Windows laptop (≈ 20 s with `--no-damage`), whole flat 4–13 min
on 2 cloud cores; video ≈ 1 s per depth keyframe for the model plus tracking (37 s clip ≈ 4 min, 3.5 min walkthrough
≈ 30 min first run); photos ≈ 2 s per photo. `--no-damage` if time is short.

## Reading the result against their laser
- Wall lengths: `rooms[].walls[].length` (interior face to interior face), 95% interval in `ci95`.
- Ceiling: `rooms[].ceiling_height` — if `observed: false` the capture never looked up; say so, don't defend the prior.
- Openings: `rooms[].openings[]`, width jamb-to-jamb.
- Floor area: `rooms[].floor_area`.

## If something goes wrong
| symptom | cause | action |
|---|---|---|
| a room missing | walked past it without entering | it is reported as not captured — don't re-run with tweaks |
| two rooms merged | opening wider than 1.25 m (open plan) | expected behaviour, stated in report §9 |
| "depth weights missing" | weights not fetched | `python scripts/fetch_weights.py` |
| very wide intervals on video/photo | scale from mono-depth, drift over the walk | expected; set from measured error (D24) |
| video plan: rooms merged / misplaced | camera path drifts on fast turns (1–5 m over a flat) | known limit (report §5); offer the LiDAR or a slower walk |
| photo plan far too small | stitch stacked rooms on each other | known limit (report §5, §9); say so |
| `ffmpeg not found` | no system ffmpeg and imageio-ffmpeg missing | `pip install -r requirements.txt` |
