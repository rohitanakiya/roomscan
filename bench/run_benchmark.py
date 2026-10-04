"""Regenerate every reported number from raw inputs.

    python bench/run_benchmark.py [--force] [--out bench/results]

Runs all captures through all tiers (one CLI command each), the drift ablation, staged-damage
evaluation and cross-capture/cross-tier comparisons, then writes bench/results/benchmark.json
and bench/results/BENCHMARK.md. Every table in the technical report is copied from that file.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "bench"))
from compare import compare  # noqa: E402
from eval_damage import evaluate as eval_damage  # noqa: E402

CAPTURES = ["single_room", "floor_only", "with_ceiling"]
RUNS = ROOT / "out" / "bench"
DATA = ROOT / "data"


def run(cmd_args, out, force=False, log=print):
    res = Path(out) / "result.json"
    if res.exists() and not force:
        return json.load(open(res)), None
    if force:   # regenerate from raw inputs: no cached frames/orthos from an older code version survive
        import shutil
        shutil.rmtree(Path(out) / "damage", ignore_errors=True)
        for f in ("result.json", "plan.png", "plan.svg", "log.txt"):
            (Path(out) / f).unlink(missing_ok=True)
    t = time.time()
    cmd = [sys.executable, "-m", "roomscan"] + cmd_args + ["--out", str(out)]
    log("  $ " + " ".join(cmd))
    src = __import__("os").environ.get("ROOMSCAN_SRC", str(ROOT / "src"))   # fix loop: run a tagged checkout
    p = subprocess.run(cmd, cwd=ROOT, env={**__import__("os").environ, "PYTHONPATH": src},
                       capture_output=True, text=True)
    (Path(out)).mkdir(parents=True, exist_ok=True)
    (Path(out) / "log.txt").write_text(p.stdout + p.stderr)
    if p.returncode != 0 or not res.exists():
        log(f"    FAILED ({p.returncode}): {p.stderr[-400:]}")
        return None, time.time() - t
    return json.load(open(res)), time.time() - t


def gate_rows(cmp, kind):
    return [r for r in cmp["rows"] if r["kind"] == kind and "diff" in r]


def summarise_pair(cmp, wall_tol_abs=0.01, wall_tol_rel=0.005):
    walls = [r for r in gate_rows(cmp, "wall_length") if r.get("observed", True)]
    ow = gate_rows(cmp, "opening_width")
    missed = [r for r in cmp["rows"] if r["kind"] == "opening_missed_in_b"]
    extra = [r for r in cmp["rows"] if r["kind"] == "opening_extra_in_b"]
    areas = gate_rows(cmp, "floor_area")
    ch = gate_rows(cmp, "ceiling_height")
    def pct(x):
        return None if not x else round(100.0 * float(np.mean(x)), 1)
    return dict(
        registration=cmp["registration"], room_pairs=cmp["room_pairs"],
        walls_compared=len(walls),
        wall_abs_err_cm_median=None if not walls else round(100 * float(np.median([abs(r["diff"]) for r in walls])), 2),
        wall_rel_err_pct_median=None if not walls else round(100 * float(np.median([abs(r["rel"]) for r in walls])), 2),
        wall_repeatable_pct=pct([abs(r["diff"]) <= max(wall_tol_abs, wall_tol_rel * r["a"]["value"]) for r in walls]),
        wall_within_3pct=pct([abs(r["rel"]) <= 0.03 for r in walls]),
        wall_within_8pct=pct([abs(r["rel"]) <= 0.08 for r in walls]),
        wall_ref_in_ci_pct=pct([r["a_in_b_ci"] for r in walls]),
        openings_matched=len(ow), openings_missed=len(missed), openings_phantom=len(extra),
        opening_within_2cm_pct=None if not (ow or missed or extra) else
        round(100.0 * sum(abs(r["diff"]) <= 0.02 for r in ow) / max(len(ow) + len(missed) + len(extra), 1), 1),
        floor_area_rel_err_pct=[round(100 * r["rel"], 1) for r in areas],
        ceiling_diff_cm=[round(100 * r["diff"], 2) for r in ch],
        rows=[{k: (v if not isinstance(v, dict) else v.get("value")) for k, v in r.items()} for r in cmp["rows"]],
    )


def main(force=False, outdir=ROOT / "bench" / "results", tiers=("lidar", "video", "photo"), runs_dir=None,
         damage=True):
    global RUNS
    RUNS = Path(runs_dir) if runs_dir else ROOT / "out" / "bench"
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    B = dict(generated=time.strftime("%Y-%m-%d %H:%M:%S"), runs={}, timing={}, comparisons={}, notes=[])
    R = {}
    # 1. LiDAR tier, drift on (default) and off (ablation)
    for c in CAPTURES:
        for drift in (True, False):
            key = f"lidar/{c}" + ("" if drift else "/no_drift")
            args = [str(DATA / "raw" / c), "--tier", "lidar"] + ([] if drift else ["--no-drift", "--no-damage"])
            if not damage and "--no-damage" not in args:
                args.append("--no-damage")
            res, dt = run(args, RUNS / key, force)
            R[key] = res
            if res:
                B["timing"][key] = res["pipeline"].get("timing")
    # 2. video + photo tiers
    from roomscan.models.depth import available

    for c in CAPTURES:
        v = DATA / "tiers" / "video" / f"{c}.mp4"
        p = DATA / "tiers" / "photos" / c
        if "video" in tiers and v.exists() and available():
            R[f"video/{c}"], _ = run([str(v), "--tier", "video", "--no-damage"], RUNS / f"video/{c}", force)
        if "photo" in tiers and p.exists() and available():
            R[f"photo/{c}"], _ = run([str(p), "--tier", "photo", "--no-damage"], RUNS / f"photo/{c}", force)
    if not available():
        B["notes"].append("depth weights not present: video and photo tiers not run")
    for k, v in R.items():
        if v:
            B["runs"][k] = dict(rooms=len(v["rooms"]), footprint=v["property"]["footprint_area"],
                                drift=v["property"].get("drift"), timing=v["pipeline"].get("timing"),
                                wall_sharpness=v["property"].get("wall_sharpness"),
                                room_overlap_m2=v["property"].get("room_overlap_m2"))
            if k in R and v:
                B["timing"][k] = v["pipeline"].get("timing")
    # 3. LiDAR repeatability: every pair of captures of the same flat
    pairs = [("floor_only", "with_ceiling"), ("with_ceiling", "single_room"), ("floor_only", "single_room")]
    for a, b in pairs:
        for suffix in ("", "/no_drift"):
            ka, kb = f"lidar/{a}{suffix}", f"lidar/{b}{suffix}"
            if R.get(ka) and R.get(kb):
                B["comparisons"][f"repeat:{a}~{b}{suffix}"] = summarise_pair(compare(R[ka], R[kb]))
    # 4. thin tiers vs LiDAR reference of the same capture
    for c in CAPTURES:
        for t in ("video", "photo"):
            if R.get(f"{t}/{c}") and R.get(f"lidar/{c}"):
                B["comparisons"][f"{t}_vs_lidar:{c}"] = summarise_pair(compare(R[f"lidar/{c}"], R[f"{t}/{c}"]))
    # 5. staged damage
    st = DATA / "staged" / "single_room_staged"
    if st.exists() and damage:
        res, _ = run([str(st), "--tier", "lidar"], RUNS / "lidar/single_room_staged", force)
        if res:
            B["damage_staged"] = eval_damage(res, json.load(open(st / "staged_truth.json")))
    if R.get("lidar/single_room"):
        B["damage_clean_false_positives"] = sum(len(r["damage"]) for r in R["lidar/single_room"]["rooms"])
    # 6. ceiling heights observed (with_ceiling)
    wc = R.get("lidar/with_ceiling")
    if wc:
        B["ceilings"] = [dict(room=r["id"], value=r["ceiling_height"]["value"], ci95=r["ceiling_height"]["ci95"],
                              sigma=r["ceiling_height"]["sigma"]) for r in wc["rooms"] if r["ceiling_height"]["observed"]]
    json.dump(B, open(outdir / "benchmark.json", "w"), indent=1, default=float)
    write_md(B, outdir / "BENCHMARK.md")
    print(f"wrote {outdir/'benchmark.json'} and BENCHMARK.md")
    return B


def write_md(B, path):
    L = [f"# Benchmark results\n\nGenerated {B['generated']} by `python bench/run_benchmark.py`. "
         "No laser ground truth exists for the sample data; 'reference' below is another capture or the LiDAR tier "
         "of the same capture, as stated per table.\n"]
    for n in B["notes"]:
        L.append(f"> NOTE: {n}\n")
    L.append("## Runs (drift ablation: `*/no_drift` = ARKit poses as-is)\n\n| run | rooms | footprint m² [95% CI] | "
             "wall sharpness (±2 cm / ±15 cm) | loop residual before→after cm | loop edges kept | total s |\n"
             "|---|---|---|---|---|---|---|")
    for k, v in B["runs"].items():
        fp = v["footprint"]
        d = v.get("drift") or {}
        lr = (f"{100 * d['mean_loop_residual_before_m']:.1f} → {100 * (d.get('mean_loop_residual_after_m') or 0):.1f}"
              if d.get("mean_loop_residual_before_m") is not None else "—")
        L.append(f"| {k} | {v['rooms']} | {fp['value']:.2f} [{fp['ci95'][0]:.2f}, {fp['ci95'][1]:.2f}] | "
                 f"{v.get('wall_sharpness')} | {lr} | {d.get('loop_edges_kept', '—')} | "
                 f"{(v.get('timing') or {}).get('total_s', '')} |")
    L.append("\n## Comparisons\n\n| comparison | reg. score | room pairs | walls | median |Δ| cm | median |Δ| % | "
             "repeatable (≤1 cm/0.5%) % | ≤3% % | ≤8% % | ref in 95% CI % | openings matched/missed/phantom | opening ≤2 cm % |\n"
             "|---|---|---|---|---|---|---|---|---|---|---|---|")
    for k, s in B["comparisons"].items():
        L.append(f"| {k} | {s['registration']['score']} | {len(s['room_pairs'])} | {s['walls_compared']} | "
                 f"{s['wall_abs_err_cm_median']} | {s['wall_rel_err_pct_median']} | {s['wall_repeatable_pct']} | "
                 f"{s['wall_within_3pct']} | {s['wall_within_8pct']} | {s['wall_ref_in_ci_pct']} | "
                 f"{s['openings_matched']}/{s['openings_missed']}/{s['openings_phantom']} | {s['opening_within_2cm_pct']} |")
    if B.get("ceilings"):
        L.append("\n## Ceiling heights (with_ceiling capture)\n\n| room | value m | 95% CI | σ mm |\n|---|---|---|---|")
        for c in B["ceilings"]:
            L.append(f"| {c['room']} | {c['value']:.3f} | [{c['ci95'][0]:.3f}, {c['ci95'][1]:.3f}] | {1000 * c['sigma']:.1f} |")
    if B.get("damage_staged"):
        d = B["damage_staged"]
        L.append(f"\n## Staged damage\n\nrecall {d['recall']:.2f}, false positives {d['false_positives']}, "
                 f"clean-capture false positives {B.get('damage_clean_false_positives')}\n")
        for it in d["items"]:
            L.append(f"- {it['id']} {it['cls']}: detected={it['detected']} " +
                     (f"rel_err={it.get('rel_err')} rules={it.get('concealed_rules')}" if it["detected"] else ""))
    L.append("\n## Timing (s)\n\n| run | reconstruct | layout | damage | total |\n|---|---|---|---|---|")
    for k, t in B["timing"].items():
        if t:
            L.append(f"| {k} | {t.get('reconstruct_s')} | {t.get('layout_s')} | {t.get('damage_s', '')} | {t.get('total_s')} |")
    Path(path).write_text("\n".join(L) + "\n")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--force", action="store_true")
    ap.add_argument("--out", default=str(ROOT / "bench" / "results"))
    ap.add_argument("--runs", default=None, help="where per-capture outputs go (default out/bench)")
    ap.add_argument("--no-damage", action="store_true", help="geometry only (fix-loop runs)")
    ap.add_argument("--tiers", default="lidar,video,photo")
    a = ap.parse_args()
    main(a.force, a.out, tuple(a.tiers.split(",")), a.runs, not a.no_damage)
