# Benchmark results

Generated 2026-10-04 09:36:55 by `python bench/run_benchmark.py`. No laser ground truth exists for the sample data; 'reference' below is another capture or the LiDAR tier of the same capture, as stated per table.

> NOTE: depth weights not present: video and photo tiers not run

## Runs

| run | rooms | footprint m² [95% CI] | total s |
|---|---|---|---|
| lidar/single_room | 4 | 22.66 [22.20, 23.12] | 169.6 |
| lidar/single_room/no_drift | 4 | 21.70 [21.22, 22.19] | 6.5 |
| lidar/floor_only | 5 | 61.36 [60.31, 62.41] | 438.0 |
| lidar/floor_only/no_drift | 5 | 60.59 [59.47, 61.72] | 23.8 |
| lidar/with_ceiling | 5 | 65.44 [63.87, 67.01] | 782.4 |
| lidar/with_ceiling/no_drift | 4 | 67.78 [66.01, 69.55] | 48.2 |

## Comparisons

| comparison | reg. score | room pairs | walls | median |Δ| cm | median |Δ| % | repeatable (≤1 cm/0.5%) % | ≤3% % | ≤8% % | ref in 95% CI % | openings matched/missed/phantom | opening ≤2 cm % |
|---|---|---|---|---|---|---|---|---|---|---|---|
| repeat:floor_only~with_ceiling | 0.378 | 4 | 14 | 17.85 | 9.67 | 0.0 | 28.6 | 42.9 | 28.6 | 0/6/8 | 0.0 |
| repeat:floor_only~with_ceiling/no_drift | 0.286 | 3 | 15 | 22.1 | 10.91 | 0.0 | 6.7 | 40.0 | 13.3 | 1/4/5 | 0.0 |
| repeat:with_ceiling~single_room | 0.276 | 1 | 3 | 89.0 | 41.17 | 0.0 | 0.0 | 0.0 | 0.0 | 0/3/2 | 0.0 |
| repeat:with_ceiling~single_room/no_drift | 0.299 | 0 | 0 | None | None | None | None | None | None | 0/0/0 | None |
| repeat:floor_only~single_room | 0.36 | 2 | 8 | 20.6 | 8.71 | 0.0 | 25.0 | 50.0 | 37.5 | 0/2/4 | 0.0 |
| repeat:floor_only~single_room/no_drift | 0.207 | 2 | 3 | 142.1 | 35.84 | 0.0 | 0.0 | 0.0 | 0.0 | 0/3/4 | 0.0 |

## Ceiling heights (with_ceiling capture)

| room | value m | 95% CI | σ mm |
|---|---|---|---|
| R1 | 3.136 | [3.115, 3.158] | 11.0 |
| R2 | 3.191 | [3.170, 3.213] | 11.0 |
| R3 | 2.635 | [2.616, 2.654] | 10.0 |
| R4 | 2.581 | [2.562, 2.599] | 10.0 |
| R5 | 2.467 | [2.448, 2.485] | 9.0 |

## Staged damage

recall 0.00, false positives 2, clean-capture false positives 3

- GT1 water_stain: detected=False 
- GT2 crack: detected=False 

## Timing (s)

| run | reconstruct | layout | damage | total |
|---|---|---|---|---|
| lidar/single_room | 13.1 | 0.7 | 155.4 | 169.6 |
| lidar/single_room/no_drift | 5.4 | 0.8 |  | 6.5 |
| lidar/floor_only | 63.9 | 4.4 | 369.4 | 438.0 |
| lidar/floor_only/no_drift | 18.6 | 4.9 |  | 23.8 |
| lidar/with_ceiling | 253.4 | 8.3 | 520.4 | 782.4 |
| lidar/with_ceiling/no_drift | 38.6 | 9.2 |  | 48.2 |
