# Benchmark results

Generated 2026-10-04 18:45:23 by `python bench/run_benchmark.py`. No laser ground truth exists for the sample data; 'reference' below is another capture or the LiDAR tier of the same capture, as stated per table.

> NOTE: depth weights not present: video and photo tiers not run

## Runs (drift ablation: `*/no_drift` = ARKit poses as-is)

| run | rooms | footprint m² [95% CI] | wall sharpness (±2 cm / ±15 cm) | loop residual before→after cm | loop edges kept | total s |
|---|---|---|---|---|---|---|
| lidar/single_room | 4 | 23.12 [21.68, 24.55] | 0.6998 | 8.4 → 2.7 | 4 | 87.5 |
| lidar/single_room/no_drift | 4 | 23.86 [22.42, 25.30] | 0.6065 | — | — | 5.1 |
| lidar/floor_only | 5 | 63.80 [61.65, 65.95] | 0.489 | 14.5 → 6.1 | 14 | 227.3 |
| lidar/floor_only/no_drift | 5 | 64.25 [62.11, 66.39] | 0.5145 | — | — | 19.4 |
| lidar/with_ceiling | 5 | 68.10 [65.46, 70.74] | 0.4625 | 10.2 → 3.3 | 68 | 518.1 |
| lidar/with_ceiling/no_drift | 4 | 70.77 [68.05, 73.50] | 0.3541 | — | — | 38.6 |

## Comparisons

| comparison | reg. score | room pairs | walls | median |Δ| cm | median |Δ| % | repeatable (≤1 cm/0.5%) % | ≤3% % | ≤8% % | ref in 95% CI % | openings matched/missed/phantom | opening ≤2 cm % |
|---|---|---|---|---|---|---|---|---|---|---|---|
| repeat:floor_only~with_ceiling | 0.331 | 5 | 24 | 31.15 | 16.16 | 4.2 | 29.2 | 41.7 | 45.8 | 3/5/7 | 6.7 |
| repeat:floor_only~with_ceiling/no_drift | 0.297 | 4 | 18 | 31.75 | 26.38 | 0.0 | 16.7 | 33.3 | 27.8 | 2/7/3 | 0.0 |
| repeat:with_ceiling~single_room | 0.312 | 2 | 7 | 25.3 | 11.02 | 28.6 | 42.9 | 42.9 | 42.9 | 1/4/2 | 14.3 |
| repeat:with_ceiling~single_room/no_drift | 0.297 | 1 | 2 | 154.35 | 25.36 | 0.0 | 50.0 | 50.0 | 50.0 | 0/0/1 | 0.0 |
| repeat:floor_only~single_room | 0.294 | 2 | 6 | 11.95 | 4.22 | 16.7 | 50.0 | 66.7 | 66.7 | 0/1/3 | 0.0 |
| repeat:floor_only~single_room/no_drift | 0.258 | 2 | 8 | 24.85 | 10.03 | 0.0 | 0.0 | 37.5 | 37.5 | 1/1/2 | 0.0 |

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
| lidar/single_room | 12.4 | 0.9 | 74.0 | 87.5 |
| lidar/single_room/no_drift | 3.9 | 1.0 |  | 5.1 |
| lidar/floor_only | 78.0 | 5.3 | 143.8 | 227.3 |
| lidar/floor_only/no_drift | 13.9 | 5.3 |  | 19.4 |
| lidar/with_ceiling | 283.6 | 10.7 | 223.6 | 518.1 |
| lidar/with_ceiling/no_drift | 28.6 | 9.8 |  | 38.6 |
