# Benchmark results

Generated 2026-10-04 10:24:20 by `python bench/run_benchmark.py`. No laser ground truth exists for the sample data; 'reference' below is another capture or the LiDAR tier of the same capture, as stated per table.

> NOTE: depth weights not present: video and photo tiers not run

## Runs

| run | rooms | footprint m² [95% CI] | total s |
|---|---|---|---|
| lidar/single_room | 4 | 22.66 [22.20, 23.12] | 14.4 |
| lidar/single_room/no_drift | 4 | 21.70 [21.22, 22.19] | 7.8 |
| lidar/floor_only | 5 | 61.36 [60.31, 62.41] | 105.0 |
| lidar/floor_only/no_drift | 5 | 60.59 [59.47, 61.72] | 26.4 |
| lidar/with_ceiling | 5 | 65.44 [63.87, 67.01] | 255.9 |
| lidar/with_ceiling/no_drift | 4 | 67.78 [66.01, 69.55] | 45.0 |

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

## Timing (s)

| run | reconstruct | layout | damage | total |
|---|---|---|---|---|
| lidar/single_room | 13.4 | 0.7 |  | 14.4 |
| lidar/single_room/no_drift | 6.7 | 0.9 |  | 7.8 |
| lidar/floor_only | 97.7 | 7.0 |  | 105.0 |
| lidar/floor_only/no_drift | 21.3 | 4.9 |  | 26.4 |
| lidar/with_ceiling | 248.0 | 7.6 |  | 255.9 |
| lidar/with_ceiling/no_drift | 36.1 | 8.7 |  | 45.0 |
