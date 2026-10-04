# Benchmark results

Generated 2026-10-04 12:20:54 by `python bench/run_benchmark.py`. No laser ground truth exists for the sample data; 'reference' below is another capture or the LiDAR tier of the same capture, as stated per table.

> NOTE: depth weights not present: video and photo tiers not run

## Runs (drift ablation: `*/no_drift` = ARKit poses as-is)

| run | rooms | footprint m² [95% CI] | wall sharpness (±2 cm / ±15 cm) | loop residual before→after cm | loop edges kept | total s |
|---|---|---|---|---|---|---|
| lidar/single_room | 4 | 23.12 [21.68, 24.55] | 0.6998 | 8.4 → 2.7 | 4 | 120.3 |
| lidar/single_room/no_drift | 4 | 23.86 [22.42, 25.30] | 0.6065 | — | — | 5.8 |
| lidar/floor_only | 5 | 63.89 [61.74, 66.04] | 0.4847 | 14.5 → 6.1 | 14 | 166.6 |
| lidar/floor_only/no_drift | 5 | 64.69 [62.53, 66.85] | 0.4981 | — | — | 21.2 |
| lidar/with_ceiling | 5 | 68.03 [65.39, 70.67] | 0.4622 | 10.2 → 3.3 | 68 | 435.4 |
| lidar/with_ceiling/no_drift | 4 | 71.08 [68.35, 73.80] | 0.3528 | — | — | 39.0 |

## Comparisons

| comparison | reg. score | room pairs | walls | median |Δ| cm | median |Δ| % | repeatable (≤1 cm/0.5%) % | ≤3% % | ≤8% % | ref in 95% CI % | openings matched/missed/phantom | opening ≤2 cm % |
|---|---|---|---|---|---|---|---|---|---|---|---|
| repeat:floor_only~with_ceiling | 0.329 | 5 | 23 | 24.0 | 12.06 | 4.3 | 30.4 | 47.8 | 47.8 | 3/6/7 | 6.2 |
| repeat:floor_only~with_ceiling/no_drift | 0.301 | 4 | 19 | 25.2 | 20.61 | 0.0 | 10.5 | 36.8 | 36.8 | 2/8/3 | 0.0 |
| repeat:with_ceiling~single_room | 0.314 | 2 | 7 | 25.3 | 11.02 | 28.6 | 42.9 | 42.9 | 42.9 | 1/4/2 | 14.3 |
| repeat:with_ceiling~single_room/no_drift | 0.282 | 1 | 2 | 154.35 | 25.36 | 0.0 | 50.0 | 50.0 | 50.0 | 0/0/1 | 0.0 |
| repeat:floor_only~single_room | 0.289 | 2 | 6 | 17.55 | 7.06 | 16.7 | 50.0 | 50.0 | 50.0 | 0/1/3 | 0.0 |
| repeat:floor_only~single_room/no_drift | 0.236 | 2 | 8 | 33.85 | 12.01 | 12.5 | 12.5 | 37.5 | 37.5 | 1/1/2 | 0.0 |

## Ceiling heights (with_ceiling capture)

| room | value m | 95% CI | σ mm |
|---|---|---|---|
| R1 | 2.455 | [2.437, 2.473] | 9.0 |
| R2 | 3.060 | [3.039, 3.081] | 11.0 |
| R3 | 3.028 | [3.007, 3.049] | 11.0 |
| R4 | 2.361 | [2.343, 2.379] | 9.0 |
| R5 | 2.397 | [2.379, 2.415] | 9.0 |

## Staged damage

recall 0.00, false positives 0, clean-capture false positives 1

- GT1 water_stain: detected=False 
- GT2 crack: detected=False 

## Timing (s)

| run | reconstruct | layout | damage | total |
|---|---|---|---|---|
| lidar/single_room | 33.8 | 1.5 | 84.5 | 120.3 |
| lidar/single_room/no_drift | 4.6 | 1.0 |  | 5.8 |
| lidar/floor_only | 87.5 | 6.7 | 72.1 | 166.6 |
| lidar/floor_only/no_drift | 14.2 | 6.8 |  | 21.2 |
| lidar/with_ceiling | 298.9 | 11.9 | 124.4 | 435.4 |
| lidar/with_ceiling/no_drift | 26.8 | 11.9 |  | 39.0 |
