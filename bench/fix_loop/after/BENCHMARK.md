# Benchmark results

Generated 2026-10-04 10:32:04 by `python bench/run_benchmark.py`. No laser ground truth exists for the sample data; 'reference' below is another capture or the LiDAR tier of the same capture, as stated per table.

> NOTE: depth weights not present: video and photo tiers not run

## Runs

| run | rooms | footprint m² [95% CI] | total s |
|---|---|---|---|
| lidar/single_room | 4 | 22.66 [22.20, 23.12] | 13.6 |
| lidar/single_room/no_drift | 4 | 21.70 [21.22, 22.19] | 6.1 |
| lidar/floor_only | 5 | 61.08 [60.03, 62.13] | 67.3 |
| lidar/floor_only/no_drift | 5 | 60.15 [59.03, 61.27] | 23.9 |
| lidar/with_ceiling | 8 | 63.39 [61.90, 64.88] | 257.0 |
| lidar/with_ceiling/no_drift | 4 | 67.78 [66.01, 69.55] | 45.1 |

## Comparisons

| comparison | reg. score | room pairs | walls | median |Δ| cm | median |Δ| % | repeatable (≤1 cm/0.5%) % | ≤3% % | ≤8% % | ref in 95% CI % | openings matched/missed/phantom | opening ≤2 cm % |
|---|---|---|---|---|---|---|---|---|---|---|---|
| repeat:floor_only~with_ceiling | 0.41 | 4 | 18 | 24.85 | 7.98 | 16.7 | 27.8 | 50.0 | 27.8 | 1/3/6 | 10.0 |
| repeat:floor_only~with_ceiling/no_drift | 0.301 | 2 | 14 | 23.0 | 16.35 | 0.0 | 14.3 | 35.7 | 21.4 | 1/1/1 | 0.0 |
| repeat:with_ceiling~single_room | 0.378 | 4 | 13 | 17.8 | 11.36 | 23.1 | 23.1 | 38.5 | 30.8 | 0/8/8 | 0.0 |
| repeat:with_ceiling~single_room/no_drift | 0.299 | 0 | 0 | None | None | None | None | None | None | 0/0/0 | None |
| repeat:floor_only~single_room | 0.37 | 2 | 8 | 20.6 | 8.71 | 0.0 | 25.0 | 50.0 | 37.5 | 0/2/4 | 0.0 |
| repeat:floor_only~single_room/no_drift | 0.23 | 2 | 3 | 142.1 | 35.84 | 0.0 | 0.0 | 0.0 | 0.0 | 0/3/4 | 0.0 |

## Ceiling heights (with_ceiling capture)

| room | value m | 95% CI | σ mm |
|---|---|---|---|
| R1 | 3.138 | [3.116, 3.159] | 11.0 |
| R2 | 2.635 | [2.616, 2.654] | 10.0 |
| R3 | 2.581 | [2.562, 2.599] | 10.0 |
| R4 | 3.174 | [3.152, 3.195] | 11.0 |
| R5 | 2.431 | [2.413, 2.449] | 9.0 |
| R6 | 2.467 | [2.448, 2.485] | 9.0 |
| R7 | 3.181 | [3.159, 3.203] | 11.0 |
| R8 | 2.599 | [2.580, 2.618] | 10.0 |

## Timing (s)

| run | reconstruct | layout | damage | total |
|---|---|---|---|---|
| lidar/single_room | 12.5 | 0.9 |  | 13.6 |
| lidar/single_room/no_drift | 5.0 | 0.8 |  | 6.1 |
| lidar/floor_only | 62.0 | 5.0 |  | 67.3 |
| lidar/floor_only/no_drift | 18.0 | 5.6 |  | 23.9 |
| lidar/with_ceiling | 246.5 | 10.1 |  | 257.0 |
| lidar/with_ceiling/no_drift | 35.2 | 9.6 |  | 45.1 |
