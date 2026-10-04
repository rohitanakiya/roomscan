# Benchmark results

Generated 2026-10-04 10:43:17 by `python bench/run_benchmark.py`. No laser ground truth exists for the sample data; 'reference' below is another capture or the LiDAR tier of the same capture, as stated per table.

> NOTE: depth weights not present: video and photo tiers not run

## Runs

| run | rooms | footprint m² [95% CI] | total s |
|---|---|---|---|
| lidar/single_room | 4 | 23.21 [22.71, 23.70] | 9.2 |
| lidar/single_room/no_drift | 4 | 23.86 [23.47, 24.25] | 5.3 |
| lidar/floor_only | 5 | 63.28 [62.19, 64.37] | 46.7 |
| lidar/floor_only/no_drift | 5 | 64.69 [63.52, 65.87] | 17.5 |
| lidar/with_ceiling | 8 | 69.07 [67.52, 70.63] | 165.2 |
| lidar/with_ceiling/no_drift | 4 | 71.08 [69.25, 72.90] | 34.5 |

## Comparisons

| comparison | reg. score | room pairs | walls | median |Δ| cm | median |Δ| % | repeatable (≤1 cm/0.5%) % | ≤3% % | ≤8% % | ref in 95% CI % | openings matched/missed/phantom | opening ≤2 cm % |
|---|---|---|---|---|---|---|---|---|---|---|---|
| repeat:floor_only~with_ceiling | 0.434 | 4 | 19 | 9.2 | 4.34 | 5.3 | 31.6 | 63.2 | 36.8 | 1/4/4 | 0.0 |
| repeat:floor_only~with_ceiling/no_drift | 0.301 | 4 | 19 | 25.2 | 20.61 | 0.0 | 10.5 | 36.8 | 15.8 | 2/7/1 | 0.0 |
| repeat:with_ceiling~single_room | 0.325 | 4 | 12 | 18.4 | 9.44 | 0.0 | 16.7 | 41.7 | 16.7 | 0/6/7 | 0.0 |
| repeat:with_ceiling~single_room/no_drift | 0.282 | 1 | 2 | 154.35 | 25.36 | 0.0 | 50.0 | 50.0 | 50.0 | 0/0/1 | 0.0 |
| repeat:floor_only~single_room | 0.331 | 2 | 8 | 20.9 | 17.4 | 0.0 | 25.0 | 37.5 | 12.5 | 0/3/3 | 0.0 |
| repeat:floor_only~single_room/no_drift | 0.236 | 2 | 8 | 33.85 | 12.01 | 12.5 | 12.5 | 37.5 | 12.5 | 0/1/2 | 0.0 |

## Ceiling heights (with_ceiling capture)

| room | value m | 95% CI | σ mm |
|---|---|---|---|
| R1 | 3.138 | [3.116, 3.159] | 11.0 |
| R2 | 2.632 | [2.613, 2.651] | 10.0 |
| R3 | 2.581 | [2.562, 2.600] | 10.0 |
| R4 | 3.173 | [3.151, 3.195] | 11.0 |
| R5 | 2.434 | [2.416, 2.452] | 9.0 |
| R6 | 2.467 | [2.449, 2.486] | 9.0 |
| R7 | 3.181 | [3.159, 3.203] | 11.0 |
| R8 | 2.599 | [2.580, 2.618] | 10.0 |

## Timing (s)

| run | reconstruct | layout | damage | total |
|---|---|---|---|---|
| lidar/single_room | 8.3 | 0.7 |  | 9.2 |
| lidar/single_room/no_drift | 4.3 | 0.7 |  | 5.3 |
| lidar/floor_only | 42.4 | 4.1 |  | 46.7 |
| lidar/floor_only/no_drift | 12.5 | 4.8 |  | 17.5 |
| lidar/with_ceiling | 155.6 | 9.3 |  | 165.2 |
| lidar/with_ceiling/no_drift | 25.8 | 8.4 |  | 34.5 |
