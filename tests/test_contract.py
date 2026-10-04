"""Contract tests: every produced result.json validates against the published schema and keeps the
invariants the plan depends on (intervals contain the value, walls close the polygon, no overlaps)."""
import glob
import json
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
SCHEMA = json.load(open(ROOT / "schema" / "output.schema.json"))
RESULTS = sorted(glob.glob(str(ROOT / "out" / "bench" / "**" / "result.json"), recursive=True))


@pytest.mark.parametrize("path", RESULTS or [None])
def test_result(path):
    if path is None:
        pytest.skip("run bench/run_benchmark.py first")
    import jsonschema

    r = json.load(open(path))
    jsonschema.validate(r, SCHEMA)
    for room in r["rooms"]:
        for m in [room["floor_area"], room["ceiling_height"]] + [w["length"] for w in room["walls"]]:
            assert m["ci95"][0] <= m["value"] <= m["ci95"][1]
        P = np.array(room["polygon"])
        for w, a, b in zip(room["walls"], P, np.roll(P, -1, 0)):
            assert np.allclose(w["start"], a, atol=2e-3) and np.allclose(w["end"], b, atol=2e-3)
    assert r["property"]["room_overlap_m2"] < 0.5
