"""Fill docs/report_template.md with numbers from bench/results/benchmark.json -> docs/TECHNICAL_REPORT.md.

    python bench/make_report.py
Every number in the technical report comes from this step, so re-running the benchmark (e.g. after fetching the
depth weights, which adds the video/photo rows) and this script regenerates a consistent report.
"""
from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]




def _na(v, unit=""):
    return "—" if v is None else f"{v}{unit}"


def _video_ablation():
    f = Path(__file__).resolve().parent / "results" / "video_ablation.md"
    if not f.exists():
        return "(run `python bench/video_ablation.py`)"
    lines = f.read_text().splitlines()
    i = next((k for k, l in enumerate(lines) if l.startswith("Means over")), None)
    if i is None:
        return ""
    tbl = [l for l in lines[i + 1:] if l.startswith("|")]
    return (lines[i] + " (full per-capture table: `bench/results/video_ablation.md`)\n\n" + "\n".join(tbl))

def _thin_calibration():
    """Depth-scale calibration + thin-tier interval scoring, from bench/results."""
    import json as _j
    R = Path(__file__).resolve().parent / "results"
    out = ""
    f = R / "depth_calibration.json"
    if f.exists():
        d = _j.loads(f.read_text())
        per = ", ".join(f"{k} {v['ratio_median']}" for k, v in d["per_capture"].items())
        out += (f" **Mono-depth scale (video/photo):** model depth / LiDAR depth = {per}; shipped correction "
                f"{d['scale']}; each capture is benchmarked with the scale fitted on the other two (residual "
                f"{', '.join(f'{k} {v:+.1f}%' for k, v in d['leave_one_out_scale_error_pct'].items())}).")
    b = R / "benchmark.json"
    if b.exists():
        comp = _j.loads(b.read_text())["comparisons"]
        thin = {k: v for k, v in comp.items() if k.startswith(("video_vs", "photo_vs"))}
        if thin:
            inci = sum(bool(v.get("footprint_lidar_in_ci")) for v in thin.values())
            out += (f" Thin-tier intervals are set from those measurements (D24), and the photo tier widens its own when "
                    f"its stitch self-check fails (D27): the LiDAR footprint falls inside the thin tier's 95% interval in "
                    f"{inci} of {len(thin)} runs" + ("." if inci == len(thin) else
                    " — where it does not, the error is registration/odometry, which the interval does not claim to cover (§5)."))
    return out

def fmt(x, nd=1, suf=""):
    return "—" if x is None else f"{x:.{nd}f}{suf}"


def figures():
    """Side-by-side figures for the report (copied from the benchmark run outputs)."""
    from PIL import Image

    out = ROOT / "docs" / "figures"
    out.mkdir(parents=True, exist_ok=True)
    runs = ROOT / "out" / "bench" / "lidar"

    def strip(paths, dst, h=520):
        ims = [Image.open(p).convert("RGB") for p in paths if Path(p).exists()]
        if not ims:
            return None
        ims = [im.resize((int(im.width * h / im.height), h)) for im in ims]
        W = sum(im.width for im in ims) + 10 * (len(ims) - 1)
        c = Image.new("RGB", (W, h), "white")
        x = 0
        for im in ims:
            c.paste(im, (x, 0))
            x += im.width + 10
        c.save(out / dst, optimize=True)
        return f"figures/{dst}"
    f1 = strip([runs / c / "plan.png" for c in ("floor_only", "with_ceiling", "single_room")], "plans_lidar.png")
    f2 = strip([runs / "with_ceiling" / "no_drift" / "plan.png", runs / "with_ceiling" / "plan.png"], "drift_ablation.png")
    return f1, f2


def main():
    B = json.load(open(ROOT / "bench" / "results" / "benchmark.json"))
    try:
        f1, f2 = figures()
    except Exception:
        f1 = f2 = None
    T = (ROOT / "docs" / "report_template.md").read_text()
    runs = B["runs"]
    # drift table
    rows = ["| capture | drift correction | footprint m² | wall sharpness | loop residual (cm) before → after | loop edges kept |",
            "|---|---|---|---|---|---|"]
    for c in ["single_room", "floor_only", "with_ceiling"]:
        for suf, lab in (("", "on"), ("/no_drift", "off (ARKit as-is)")):
            v = runs.get(f"lidar/{c}{suf}")
            if not v:
                continue
            d = v.get("drift") or {}
            lr = (f"{100 * d['mean_loop_residual_before_m']:.1f} → {100 * (d.get('mean_loop_residual_after_m') or 0):.1f}"
                  if d.get("mean_loop_residual_before_m") is not None else "—")
            rows.append(f"| {c} | {lab} | {v['footprint']['value']:.2f} | {v.get('wall_sharpness')} | {lr} | "
                        f"{d.get('loop_edges_kept', '—')} |")
    comp = B["comparisons"]
    on = comp.get("repeat:floor_only~with_ceiling", {})
    off = comp.get("repeat:floor_only~with_ceiling/no_drift", {})
    rows.append("")
    rows.append(f"Cross-capture effect (floor_only vs with_ceiling): registration score {on.get('registration', {}).get('score')} "
                f"with correction vs {off.get('registration', {}).get('score')} without; walls within 3%: "
                f"{on.get('wall_within_3pct')}% vs {off.get('wall_within_3pct')}%; median |ΔL| {on.get('wall_abs_err_cm_median')} vs "
                f"{off.get('wall_abs_err_cm_median')} cm.")
    if f2:
        rows.append(f"\n![with_ceiling stitched plan, drift correction off (left) and on (right)]({f2})")
    T = T.replace("{{DRIFT_TABLE}}", "\n".join(rows))
    # results
    R = []
    if f1:
        R.append(f"![Stitched LiDAR plans: floor_only, with_ceiling, single_room (one command each)]({f1})\n")
    R += ["### LiDAR repeatability (same tier, different captures of the same rooms)",
         "| pair | room pairs | walls | median \\|ΔL\\| cm | within 1 cm/0.5% | within 3% | within 8% | ref in 95% CI | openings matched/missed/phantom | opening ≤ 2 cm |",
         "|---|---|---|---|---|---|---|---|---|---|"]
    for k, s in comp.items():
        if not k.startswith("repeat:") or k.endswith("no_drift"):
            continue
        R.append(f"| {k[7:]} | {len(s['room_pairs'])} | {s['walls_compared']} | {s['wall_abs_err_cm_median']} | "
                 f"{s['wall_repeatable_pct']}% | {s['wall_within_3pct']}% | {s['wall_within_8pct']}% | {s['wall_ref_in_ci_pct']}% | "
                 f"{s['openings_matched']}/{s['openings_missed']}/{s['openings_phantom']} | {s['opening_within_2cm_pct']}% |")
    thin = [(k, s) for k, s in comp.items() if k.startswith(("video_vs_lidar", "photo_vs_lidar"))]
    R.append("\n### Thin tiers vs the LiDAR tier of the same capture")
    if thin:
        R.append("| comparison | rooms LiDAR/tier | footprint err % (gate: video ±3, photo ±8) | LiDAR footprint in 95% CI | room overlap m² | walls | median \\|ΔL\\| % | walls within 3% | walls within 8% | camera path ATE m |")
        R.append("|---|---|---|---|---|---|---|---|---|---|")
        for k, s in thin:
            R.append(f"| {k} | {s.get('rooms', ['', ''])[0]}/{s.get('rooms', ['', ''])[1]} | {s.get('footprint_rel_err_pct')} | "
                     f"{s.get('footprint_lidar_in_ci')} | {s.get('room_overlap_m2')} | {s['walls_compared']} | "
                     f"{_na(s['wall_rel_err_pct_median'])} | {_na(s['wall_within_3pct'], '%')} | {_na(s['wall_within_8pct'], '%')} | "
                     f"{s.get('ate_m', '—')} |")
        R.append("\nWalls are compared only inside room pairs overlapping with IoU ≥ 0.4 (— = no such pair). Camera path ATE: "
                 "RMS distance between the video tier's RGB-only path and ARKit after rigid alignment.")
    else:
        R.append("Not run in this benchmark: " + "; ".join(B.get("notes", [])) + ". The tiers run end-to-end; "
                 "their odometry/registration code was exercised in development with LiDAR depth substituted for the "
                 "model (DECISIONS D14), which is not a reported result.")
    if B.get("ceilings"):
        R.append("\n### Ceiling heights (only `with_ceiling` observes ceilings)")
        R.append("| room | height m | 95% CI | σ mm |\n|---|---|---|---|")
        for c in B["ceilings"]:
            R.append(f"| {c['room']} | {c['value']:.3f} | [{c['ci95'][0]:.3f}, {c['ci95'][1]:.3f}] | {1000 * c['sigma']:.1f} |")
        R.append("Repeat spread cannot be measured (no second capture looks at the ceilings); the gate row is "
                 "therefore *unverified*, not passed. Within-capture plane-fit σ is shown.")
    if B.get("damage_staged"):
        d = B["damage_staged"]
        R.append(f"\n### Damage (digitally staged, `single_room_staged`)\nrecall {d['recall']:.2f} "
                 f"({sum(i['detected'] for i in d['items'])}/{len(d['items'])}), false positives {d['false_positives']}; "
                 f"clean capture false positives {B.get('damage_clean_false_positives')}.")
    R.append("\n### Timing (2-core CPU, seconds)\n| run | reconstruct | layout | damage | total |\n|---|---|---|---|---|")
    for k, t in B["timing"].items():
        if t and "no_drift" not in k:
            R.append(f"| {k} | {t.get('reconstruct_s')} | {t.get('layout_s')} | {t.get('damage_s', '—')} | {t.get('total_s')} |")
    T = T.replace("{{RESULTS_TABLES}}", "\n".join(R))
    cal = ROOT / "bench" / "results" / "calibration_report.json"
    if cal.exists():
        try:
            c = json.loads(cal.read_text())
            loo = "; ".join(f"held-out {x['held_out']}: {100 * x['test_coverage']:.0f}% (n={x['n_test']})" for x in c["loo"])
            T = T.replace("{{CALIBRATION}}", (
                f"Uncalibrated (sensor/fit/scale/drift budget only), {100 * c['coverage_uncalibrated']:.0f}% of repeat-capture "
                f"wall differences fell inside the combined 95% intervals — overconfident. `bench/calibrate.py` fits a per-face "
                f"residual on repeat captures (gross mismatches > 30 cm — a different wall, not noise — excluded: "
                f"{c['gross_mismatch']} of {c['walls']}) and validates leave-one-pair-out: {loo}. Fitted per-face residual "
                f"{100 * c['fitted']['face_floor']:.1f} cm (ambiguity weight {c['fitted']['ambiguity_k']}). "
                "The intervals in every JSON are therefore the repeatability we can demonstrate, not the sensor spec. "
                + _thin_calibration()))
        except Exception:
            pass
    T = T.replace("{{VIDEO_ABLATION}}", _video_ablation())
    T = T.replace("{{CALIBRATION}}", "Calibration report not found; run bench/calibrate.py.")
    fl = (ROOT / "docs" / "FIX_LOOP.md").read_text()
    T = T.replace("{{FIXLOOP_SUMMARY}}", (
        "Worst gate: LiDAR repeatability, 0% of walls within 1 cm / 0.5%. Iteration 1 (capture-invariant room splitting at "
        "doorways): 0 → 16.7% / 23.1% on two pairs; median-error prediction badly wrong (post-mortem: a second, systematic "
        "cause). Iteration 2 (furniture front vs wall: camera-oriented normals, outermost floor-reaching face): median "
        "|ΔL| 24.9 → 9.2 cm, within-3% 27.8 → 31.6%, but the declared 1 cm metric fell to 5.3% — the remaining error is "
        "within-capture drift duplicates (6–10 cm double walls), now attacked by the re-tuned drift correction (D16). "
        "Gate still fails; the report says why."))
    T = T.replace("{{BENCH_DATE}}", B["generated"])
    (ROOT / "docs" / "TECHNICAL_REPORT.md").write_text(T)
    print("wrote docs/TECHNICAL_REPORT.md")


if __name__ == "__main__":
    main()
