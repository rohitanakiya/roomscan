# Benchmark data

All benchmark inputs. Raw captures are the three provided Stray Scanner exports (one flat). Everything else is derived from them by scripts in this repo. Checksums: first 16 hex of SHA-256.

## Raw captures (`data/raw/`)

| name | provided folder | frames (odometry rows) | depth frames | duration s | path m | rgb.mp4 sha | odometry sha |
|---|---|---|---|---|---|---|---|
| single_room | c00a170fe1 | 1715 | 1715 | 37.2 | 14.5 | 778648a6b2c749a1 | 4ff57818e7353af9 |
| floor_only | 1a8384c3f6 | 5251 | 5251 | 114.8 | 54.2 | 0f11d2b56e50d785 | 2ee32c3f351c5806 |
| with_ceiling | c7d28f72c6 | 9745 | 9745 | 214.9 | 99.8 | dad93c96d6255e7d | 27fff9796d5bb0a7 |

Capture style (from camera pitch): `single_room` and `floor_only` aim low (floor + lower walls); `with_ceiling` aims higher and is the only capture that observes ceilings.

## Derived tier inputs (`data/tiers/`, by `scripts/make_tier_inputs.py`)

| capture | video file | sha | photo folders (photos) |
|---|---|---|---|
| single_room | `data/tiers/video/single_room.mp4` | a612628609fa2342 | room_1 (7), room_2 (7), room_3 (7), room_4 (7) |
| floor_only | `data/tiers/video/floor_only.mp4` | 14fd06a59b91ba19 | room_1 (6), room_2 (8), room_3 (6), room_4 (6), room_5 (7) |
| with_ceiling | `data/tiers/video/with_ceiling.mp4` | ee1cb86c5ca34bc7 | room_1 (5), room_2 (6), room_3 (6), room_4 (6), room_5 (6) |

## Staged damage (`data/staged/single_room_staged/`, by `bench/stage_damage.py`)

| id | class | room/wall (at staging) | size | truth |
|---|---|---|---|---|
| GT1 | water_stain | R2/R2-W1 | 0.42×0.32 m | area 0.1038 m² |
| GT2 | crack | R2/R2-W2 | 0.1×0.6 m | length 0.60 m |

## Not available (and why)

- Laser / tape ground truth: no access to the property.
- Consumer-app exports for head-to-head: would require scanning the same rooms with that app.
- Physically staged damage: replaced by digital staging on the 3-D wall planes (above).
