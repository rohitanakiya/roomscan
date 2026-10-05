# Benchmark results

Generated 2026-10-05 07:09:59 by `python bench/run_benchmark.py`. No laser ground truth exists for the sample data; 'reference' below is another capture or the LiDAR tier of the same capture, as stated per table.

> NOTE: video/photo: each capture uses the depth scale fitted on the other captures (bench/calibrate_depth.py, leave-one-out)

## Runs (drift ablation: `*/no_drift` = ARKit poses as-is)

| run | rooms | footprint m² [95% CI] | wall sharpness (±2 cm / ±15 cm) | loop residual before→after cm | loop edges kept | total s |
|---|---|---|---|---|---|---|
| lidar/single_room | 4 | 23.12 [21.68, 24.55] | 0.6998 | 8.4 → 2.7 | 4 | 184.5 |
| lidar/single_room/no_drift | 4 | 23.86 [22.42, 25.30] | 0.6065 | — | — | 6.8 |
| lidar/floor_only | 5 | 63.80 [61.65, 65.95] | 0.489 | 14.5 → 6.1 | 14 | 350.8 |
| lidar/floor_only/no_drift | 5 | 64.25 [62.11, 66.39] | 0.5145 | — | — | 27.1 |
| lidar/with_ceiling | 5 | 68.10 [65.46, 70.74] | 0.4625 | 10.2 → 3.3 | 68 | 763.3 |
| lidar/with_ceiling/no_drift | 4 | 70.77 [68.05, 73.50] | 0.3541 | — | — | 50.6 |
| video/single_room | 1 | 32.02 [22.06, 41.98] | 0.2459 | — | — | 46.0 |
| photo/single_room | 3 | 7.11 [0.00, 50.70] | 0.1933 | — | — | 32.6 |
| video/floor_only | 3 | 47.31 [19.11, 75.51] | 0.1653 | — | — | 204.5 |
| photo/floor_only | 3 | 19.75 [0.00, 106.39] | 0.5466 | — | — | 37.8 |
| video/with_ceiling | 2 | 63.68 [0.00, 139.99] | 0.2148 | — | — | 403.4 |
| photo/with_ceiling | 3 | 12.64 [0.00, 86.70] | 0.308 | — | — | 40.5 |

## Comparisons

| comparison | reg. score | room pairs | walls | median |Δ| cm | median |Δ| % | repeatable (≤1 cm/0.5%) % | ≤3% % | ≤8% % | ref in 95% CI % | openings matched/missed/phantom | opening ≤2 cm % |
|---|---|---|---|---|---|---|---|---|---|---|---|
| repeat:floor_only~with_ceiling | 0.331 | 5 | 24 | 31.15 | 16.16 | 4.2 | 29.2 | 41.7 | 45.8 | 3/5/7 | 6.7 |
| repeat:floor_only~with_ceiling/no_drift | 0.297 | 4 | 18 | 31.75 | 26.38 | 0.0 | 16.7 | 33.3 | 27.8 | 2/7/3 | 0.0 |
| repeat:with_ceiling~single_room | 0.312 | 2 | 7 | 25.3 | 11.02 | 28.6 | 42.9 | 42.9 | 42.9 | 1/4/2 | 14.3 |
| repeat:with_ceiling~single_room/no_drift | 0.297 | 1 | 2 | 154.35 | 25.36 | 0.0 | 50.0 | 50.0 | 50.0 | 0/0/1 | 0.0 |
| repeat:floor_only~single_room | 0.294 | 2 | 6 | 11.95 | 4.22 | 16.7 | 50.0 | 66.7 | 66.7 | 0/1/3 | 0.0 |
| repeat:floor_only~single_room/no_drift | 0.258 | 2 | 8 | 24.85 | 10.03 | 0.0 | 0.0 | 37.5 | 37.5 | 1/1/2 | 0.0 |
| video_vs_lidar:single_room | 0.271 | 0 | 0 | None | None | None | None | None | None | 0/0/0 | None |
| photo_vs_lidar:single_room | 0.264 | 0 | 0 | None | None | None | None | None | None | 0/0/0 | None |
| video_vs_lidar:floor_only | 0.157 | 2 | 5 | 140.0 | 60.87 | 0.0 | 0.0 | 20.0 | 60.0 | 0/5/6 | 0.0 |
| photo_vs_lidar:floor_only | 0.179 | 0 | 0 | None | None | None | None | None | None | 0/0/0 | None |
| video_vs_lidar:with_ceiling | 0.176 | 0 | 0 | None | None | None | None | None | None | 0/0/0 | None |
| photo_vs_lidar:with_ceiling | 0.2 | 0 | 0 | None | None | None | None | None | None | 0/0/0 | None |

## Thin tiers vs LiDAR of the same capture (gates: video footprint/walls ±3 %, photo ±8 % and no overlapping rooms)

| comparison | rooms LiDAR/tier | footprint err % | LiDAR footprint in tier 95% CI | room overlap m² | walls compared | walls within 3% | walls within 8% | camera path ATE vs ARKit m (% of path) |
|---|---|---|---|---|---|---|---|---|
| video_vs_lidar:single_room | 4/1 | 38.5 | True | 0.0 | 0 | — | — | 1.099 (7.75%) |
| photo_vs_lidar:single_room | 4/3 | -69.3 | True | 0.3735 | 0 | — | — | — |
| video_vs_lidar:floor_only | 5/3 | -25.8 | True | 0.1505 | 5 | 0.0 | 20.0 | 1.466 (2.73%) |
| photo_vs_lidar:floor_only | 5/3 | -69.0 | True | 0.2733 | 0 | — | — | — |
| video_vs_lidar:with_ceiling | 5/2 | -6.5 | True | 0.5219 | 0 | — | — | 4.816 (4.87%) |
| photo_vs_lidar:with_ceiling | 5/3 | -81.4 | True | 0.0 | 0 | — | — | — |

Walls are compared only inside room pairs that overlap with IoU ≥ 0.4; where a thin tier merges or misplaces rooms there is nothing to compare, which is itself the result.

## Ceiling heights (with_ceiling capture)

| room | value m | 95% CI | σ mm |
|---|---|---|---|
| R1 | 2.455 | [2.437, 2.473] | 9.0 |
| R2 | 3.060 | [3.039, 3.081] | 11.0 |
| R3 | 3.028 | [3.007, 3.049] | 11.0 |
| R4 | 2.361 | [2.343, 2.379] | 9.0 |
| R5 | 2.397 | [2.379, 2.415] | 9.0 |

## Staged damage

recall 1.00, false positives 1, clean-capture false positives 2

- GT1 water_stain: detected=True rel_err=0.067 rules=['CD-02']
- GT2 crack: detected=True rel_err=0.175 rules=[]

## Timing (s)

| run | reconstruct | layout | damage | total |
|---|---|---|---|---|
| lidar/single_room | 32.8 | 1.6 | 149.8 | 184.5 |
| lidar/single_room/no_drift | 5.3 | 1.2 |  | 6.8 |
| lidar/floor_only | 120.8 | 7.9 | 221.7 | 350.8 |
| lidar/floor_only/no_drift | 18.4 | 8.4 |  | 27.1 |
| lidar/with_ceiling | 403.3 | 13.8 | 345.9 | 763.3 |
| lidar/with_ceiling/no_drift | 36.6 | 13.6 |  | 50.6 |
| video/single_room | 44.4 | 1.3 |  | 46.0 |
| photo/single_room | 31.6 | 0.6 |  | 32.6 |
| video/floor_only | 195.9 | 8.3 |  | 204.5 |
| photo/floor_only | 36.6 | 0.8 |  | 37.8 |
| video/with_ceiling | 384.0 | 19.0 |  | 403.4 |
| photo/with_ceiling | 39.4 | 0.7 |  | 40.5 |
