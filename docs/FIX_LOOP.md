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
