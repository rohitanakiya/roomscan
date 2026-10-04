"""Calibrate the LiDAR wall-length intervals against demonstrated repeatability.

Two captures of the same wall should differ by no more than their combined 95% intervals, 95% of the time:
    |L_a - L_b| <= 1.96 * sqrt(sigma_a^2 + sigma_b^2)
We fit two error-model terms (ErrorModel.ambiguity_k: weight of the per-wall competing-layer spread, and
ErrorModel.face_floor: a per-face residual) on wall pairs from repeat captures, leave-one-pair-out, and write
the fitted terms to src/roomscan/calibration.json, which the tiers load. Gross mismatches (|Δ| > 0.30 m:
different room segmentation, not measurement noise) are excluded from the fit and reported separately — an
interval cannot honestly absorb "this is a different wall".

    python bench/calibrate.py out/bench/lidar
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from compare import compare  # noqa: E402

PAIRS = [("floor_only", "with_ceiling"), ("floor_only", "single_room"), ("with_ceiling", "single_room")]
GROSS = 0.30


def wall_rows(root, a, b):
    A = json.load(open(Path(root) / a / "result.json"))
    B = json.load(open(Path(root) / b / "result.json"))
    c = compare(A, B)
    walls = {}
    for res in (A, B):
        for r in res["rooms"]:
            for w in r["walls"]:
                walls[(id(res), w["id"])] = w
    out = []
    for r in c["rows"]:
        if r["kind"] != "wall_length" or not r.get("observed", True):
            continue
        wa = next(w for rr in A["rooms"] for w in rr["walls"] if w["id"] == r["id_a"])
        wb = next(w for rr in B["rooms"] for w in rr["walls"] if w["id"] == r["id_b"])
        out.append(dict(pair=f"{a}~{b}", d=abs(r["diff"]), L=r["a"]["value"],
                        base_a=_base_sigma(wa), base_b=_base_sigma(wb),
                        amb_a=_amb(A, wa), amb_b=_amb(B, wb)))
    return out


def _base_sigma(w):
    b = w["length"].get("error_budget_1sigma", {})
    keep = {k: v for k, v in b.items() if not k.startswith("face_choice") and k != "calibration" and v is not None}
    return float(np.sqrt(sum(v * v for v in keep.values())))


def _amb(res, w):
    # length depends on the two bounding faces; their ambiguities are in the budget as face_choice_a/b at k
    b = w["length"].get("error_budget_1sigma", {})
    return float(np.hypot(b.get("face_choice_a") or 0.0, b.get("face_choice_b") or 0.0))


def coverage(rows, k, floor, k0):
    ok = []
    for r in rows:
        sa = np.sqrt(r["base_a"] ** 2 + (k / k0 * r["amb_a"]) ** 2 + 2 * floor ** 2)
        sb = np.sqrt(r["base_b"] ** 2 + (k / k0 * r["amb_b"]) ** 2 + 2 * floor ** 2)
        ok.append(r["d"] <= 1.96 * np.hypot(sa, sb))
    return float(np.mean(ok)) if ok else float("nan")


def fit(rows, k0, target=0.95):
    best = None
    for k in np.arange(0.0, 2.01, 0.25):
        for floor in np.arange(0.0, 0.151, 0.005):
            cov = coverage(rows, k, floor, k0)
            width = k + 20 * floor        # prefer the narrowest intervals that reach the target
            if cov >= target and (best is None or width < best[2]):
                best = (float(k), float(floor), width, cov)
    return best


def main(root):
    from roomscan.geometry.room import ErrorModel  # noqa: F401

    k0 = 0.5  # ambiguity_k used when the runs were produced
    allrows = {p: wall_rows(root, *p) for p in PAIRS}
    rows = [r for p in allrows.values() for r in p]
    inl = [r for r in rows if r["d"] <= GROSS]
    report = dict(walls=len(rows), gross_mismatch=len(rows) - len(inl),
                  coverage_uncalibrated=coverage(inl, k0, 0.0, k0), loo=[])
    for p in PAIRS:
        train = [r for q in PAIRS if q != p for r in allrows[q] if r["d"] <= GROSS]
        test = [r for r in allrows[p] if r["d"] <= GROSS]
        f = fit(train, k0)
        if f and test:
            report["loo"].append(dict(held_out=f"{p[0]}~{p[1]}", k=f[0], face_floor=f[1],
                                      test_coverage=coverage(test, f[0], f[1], k0), n_test=len(test)))
    f = fit(inl, k0)
    report["fitted"] = dict(ambiguity_k=f[0], face_floor=f[1], train_coverage=f[3]) if f else None
    out = Path(__file__).resolve().parents[1] / "src" / "roomscan" / "calibration.json"
    if f:
        json.dump(dict(lidar=dict(ambiguity_k=f[0], face_floor=f[1]), source=str(root), report=report),
                  open(out, "w"), indent=1)
    print(json.dumps(report, indent=1))


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
    main(sys.argv[1] if len(sys.argv) > 1 else "out/bench/lidar")
