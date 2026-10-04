# Fix loop

## Part A — Declaration (written and committed *before* the fix; tag `fixloop-before`)

**1. Worst-performing gate.** LiDAR **repeatability**: "two captures of the same room at the same tier agree within
1 cm or 0.5% per wall". Baseline (`bench/fix_loop/before/BENCHMARK.md`, captures `floor_only` vs `with_ceiling`,
both full walks of the same flat): **0 of 14 matched walls (0.0%)** agree; median |ΔL| = **17.9 cm** (9.7%).
The two other capture pairs are also at 0%. (Opening widths are also at 0% but are downstream of the same problem:
an opening can only be matched if the rooms on both sides are the same rooms.)
Gate pass criterion used here: ≥ 85% of matched walls within max(1 cm, 0.5% L) (the case study gives no fraction for
this row; 85% mirrors the opening gate).

**2. Root-cause hypothesis.** The two captures measure the *same walls* but segment them into *different rooms*, so
the "same wall" in each plan has different end points. Room splits come from the watershed seeded at 0.42 m
(passages narrower than ~0.84 m split rooms). Whether a ~0.8–1.2 m doorway splits therefore depends on how much
free space each capture happened to observe around it (door leaf, furniture, ray reach) — not on the building.

Evidence (`bench/fix_loop/evidence.txt`, reproduced by `bench/fix_loop/evidence.py`):
- footprint overlap is high while room overlap is low: floor_only~with_ceiling footprint overlap 0.87 of the smaller
  plan, but matched-room IoU only 0.42 / 0.46 / 0.48 / 0.30 / 0.66;
- the overlaid plans (`bench/fix_loop/before_overlay.png`) show identical outer walls with different internal
  splits: floor_only merges corridor + living room (32.7 m²) where with_ceiling splits them (14.1 + 12.3 m²);
  with_ceiling merges the two south rooms (17.5 m²) where floor_only splits them (8.6 + 6.5 m²);
- the wall *faces* themselves agree to a median 3.8 cm after a single global registration, i.e. geometry is roughly
  consistent and the 18 cm length differences are dominated by end points.

**3. Fix to ship.** Make room splitting a property of the walls, not of the free space: find doorways as gaps of
0.55–1.25 m between collinear wall-face segments (jambs on both sides), close each gap with a virtual wall for
segmentation only (the opening is still detected and measured), then run the same watershed. Splits at real
doorways become capture-invariant; wider openings (> 1.25 m) consistently merge (open plan).

**Predicted numbers after the fix** (floor_only ~ with_ceiling):
- matched-room IoU: from 0.30–0.66 to **≥ 0.75** for the rooms present in both captures;
- median |ΔL| on matched walls: from 17.9 cm to **≤ 5 cm**;
- walls within the repeatability tolerance: from 0% to **20–35%** — the gate will still **fail**, because the
  per-capture face positions differ by ~2–4 cm (evidence above) and 1 cm / 0.5% is tighter than that.
  What remains after this fix is a geometric (drift/scale) problem, not a segmentation problem.

## Part B — Result of iteration 1 (tag `fixloop-after`; regenerate: `bash bench/fix_loop/run.sh after`)

| metric (LiDAR, same flat) | before | predicted | after |
|---|---|---|---|
| floor_only~with_ceiling: walls within 1 cm / 0.5% | 0.0% (0/14) | 20–35% | **16.7%** (3/18) |
| with_ceiling~single_room: walls within 1 cm / 0.5% | 0.0% (0/3) | — | **23.1%** (3/13) |
| floor_only~single_room: walls within 1 cm / 0.5% | 0.0% (0/8) | — | 0.0% (0/8) |
| floor_only~with_ceiling: matched-room IoU | 0.42/0.46/0.48/0.30/0.66 | ≥ 0.75 | 0.42/0.35/**0.87**/0.66/0.69 |
| floor_only~with_ceiling: median \|ΔL\| | 17.9 cm | ≤ 5 cm | 24.9 cm |
| gate (≥ 85% of walls) | FAIL | FAIL | **FAIL** |

**Post-mortem.** The root cause was right but incomplete, and the median-error prediction was badly wrong.
- Right: door-width gaps now split rooms the same way in both captures — the two south rooms that one capture
  merged now match at IoU 0.87, and repeatable walls rose from 0 to 3 in each of two pairs.
- Incomplete: the corridor ↔ living-room junction is an opening wider than 1.25 m, so it is (by design) merged in
  both captures in principle — but the ceiling capture still splits it through the watershed's narrowing test.
  Wide openings remain capture-dependent.
- Wrong prediction (median |ΔL|): more walls now match (18 vs 14), including walls of rooms that are still
  segmented differently, so the median got worse. The high-IoU pair exposed a **second, systematic cause**: in the
  room matched at IoU 0.87, three of five walls differ by +15.3/+26.2/+15.6 cm, all in the same direction (the
  ceiling-aimed capture is larger). `bench/fix_loop/evidence_faces.txt` shows why: behind the fitted face of
  floor_only R3-W2 (1.7k points, lowest point 0.54 m — a desk/shelf front) there is a 7.0k-point surface 42 cm
  further out that reaches the floor (0.08 m) — the wall. "First face outward" picks furniture when the capture
  aims low; the ceiling capture sees the wall above the furniture.

## Part C — Declaration, iteration 2 (written before the change; tag `fixloop2-declared`)

**Gate:** same (LiDAR repeatability). **Failing number:** 16.7% / 23.1% / 0.0% of matched walls (iteration-1 after).

**Root cause:** wall snapping takes the first face met outward from the free-space edge; with furniture against a
wall that is the furniture front, and whether the wall behind it is seen depends on where the capture aimed.
Evidence: `bench/fix_loop/evidence_faces.txt` (above); sign of the error is consistent (+15 to +26 cm in the
ceiling-aimed capture).

**Fix:** orient every normal toward the camera that observed the point (room-side faces then point into the room,
the neighbour room's face of the same wall points away), and choose the **outermost** strong, room-facing layer
within 0.5 m whose points reach the floor (5th-percentile height < 0.45 m); fall back to the outermost strong layer.

**Prediction (floor_only~with_ceiling):** the three +15–26 cm walls of the IoU-0.87 pair agree within 3 cm; matched
walls within tolerance rise from 16.7% to **25–40%**; the gate still **fails** (wide-opening segmentation and
~2–3 cm inter-capture face offsets remain).

## Part D — Result of iteration 2 (tag `fixloop2-after`; regenerate: `bash bench/fix_loop/run.sh 2-after`)

| metric (floor_only ~ with_ceiling unless noted) | iter-1 after | predicted | iter-2 after |
|---|---|---|---|
| walls within 1 cm / 0.5% (declared gate metric) | 16.7% (3/18) | 25–40% | **5.3%** (1/19) |
| with_ceiling~single_room, same metric | 23.1% (3/13) | — | 0.0% (0/12) |
| median \|ΔL\| on matched walls | 24.9 cm | — | **9.2 cm** |
| walls within 3% / within 8% | 27.8% / 50.0% | — | **31.6% / 63.2%** |
| matched-room IoU | 0.42/0.35/0.87/0.66/0.69 | — | 0.43/0.36/**0.92/0.79**/0.68 |
| the three +15–26 cm walls of the IoU-0.87 room | +15.6 / +26.2 / +15.3 cm | each < 3 cm | +20.7 / **+3.7** / +9.2 cm |
| gate (≥ 85% of walls) | FAIL | FAIL | **FAIL** |

**Post-mortem.** The furniture-front cause was real — R3-W1 went from +26.2 cm to +3.7 cm, the median wall
difference dropped from 24.9 to 9.2 cm and the ≤3%/≤8% bands rose — but the prediction on the declared metric was
wrong in direction: the 1 cm/0.5% count fell from 3 to 1 wall. Two reasons, both visible in
`bench/fix_loop/evidence_faces.txt` for the matched rooms after the change:
1. **Drift duplicates inside one capture.** Several walls of the ceiling-aimed capture show two strong room-facing
   layers 6–10 cm apart with *different height ranges* (e.g. R4-W2: 0.25–1.89 m at 0 cm, 1.89–3.09 m at +6/+8 cm):
   the same wall seen on two passes whose poses still disagree after loop closure (loop residual 10.5 → 3.2 cm on
   average, larger locally). "Outermost layer" then systematically picks the outer duplicate.
2. **The tolerance is below our inter-capture noise floor.** Walls that were already within 1 cm in iteration 1
   moved by the few centimetres the new face rule introduces; with ~20 walls per pair, 1–3 walls inside a 1 cm band
   is noise, not signal. The ≤3% and median metrics, which are not, both improved.

**What fixing it fully needs:** pose consistency *within* each capture at the 1 cm level (denser loop closures and
plane-to-plane constraints on revisited walls), so that each wall is one layer, before any face rule. That is the
next fix, not shipped here. Meanwhile the intervals are widened to the repeatability we can actually demonstrate
(technical report §6), so the plan is not confidently wrong.
