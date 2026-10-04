# Drift-correction tuning (evidence for DECISIONS D16)

Metric: wall sharpness = fraction of room-facing vertical points within ±2 cm of their wall-face segment,
among those within ±15 cm (segments > 1 m). Doubled walls from drift lower it. `bench/drift_tuning.py <capture>`.

| variant | floor_only | with_ceiling |
|---|---|---|
| none (ARKit poses as-is) | 0.530 | 0.381 |
| fragments of 12 KF, pairs ≤ 3 m, piecewise-constant correction (v0.1) | 0.517 | 0.368 |
| same, linearly blended correction | 0.523 | 0.375 |
| fragments of 8 KF, pairs ≤ 4 m, piecewise-constant | 0.508 | 0.433 |
| **fragments of 8 KF, pairs ≤ 4 m, blended (adopted)** | **0.564** | **0.428** |
| fragments of 8 KF, pairs ≤ 4 m, voxel 3 cm, ICP rmse ≤ 1.5 cm | 0.550 | 0.370 |

The first version lowered sharpness on both captures even though its loop residual fell 10 → 2–3 cm:
piecewise-constant corrections put a step at every fragment boundary, which itself doubles walls. Smaller fragments
(more loop edges: 88 vs 41 on with_ceiling) plus blended corrections is the only variant better than raw poses on
both captures (+6% and +12% relative).
