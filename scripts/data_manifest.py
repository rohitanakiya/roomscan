"""Write docs/DATA.md: what raw and derived benchmark data exists, with checksums.

    python scripts/data_manifest.py
"""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from roomscan.io.stray import load_stray  # noqa: E402

SOURCE = {"single_room": "c00a170fe1", "floor_only": "1a8384c3f6", "with_ceiling": "c7d28f72c6"}


def sha(p, n=1 << 20):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(n), b""):
            h.update(b)
    return h.hexdigest()[:16]


DOWNLOAD = '## Download (raw data is not in git: 0.9 GB)\n\nThe three provided Stray Scanner exports are in the shared folder **[roomscan_raw_data](https://1drv.ms/f/c/ce3127bdc58443c9/IgDQs2EzfcLhTo4DRMCZvou1AWGadrdYRKJlT-uDvs5hwUg?e=EAO600)** (OneDrive; GitHub release\nassets were refused at this size):\n`single_room.zip` (c00a170fe1), `single_scan_floor_only.zip` (1a8384c3f6), `single_scan_with_ceiling.zip`\n(c7d28f72c6). Unzip each into `data/raw/single_room`, `data/raw/floor_only`, `data/raw/with_ceiling`; the checksums\nbelow identify the files every reported number was computed from. Derived inputs (video/photo tiers, staged damage)\nare regenerated from them by the scripts named in each section.\n'


def main():
    D = ROOT / "data"
    L = ["# Benchmark data\n",
         "All benchmark inputs. Raw captures are the three provided Stray Scanner exports (one flat). Everything else "
         "is derived from them by scripts in this repo. Checksums: first 16 hex of SHA-256.\n",
         DOWNLOAD,
         "## Raw captures (`data/raw/`)\n",
         "| name | provided folder | frames (odometry rows) | depth frames | duration s | path m | rgb.mp4 sha | odometry sha |",
         "|---|---|---|---|---|---|---|---|"]
    for name, src in SOURCE.items():
        p = D / "raw" / name
        if not p.exists():
            continue
        cap = load_stray(p)
        nd = len(list((p / "depth").glob("*.png")))
        L.append(f"| {name} | {src} | {len(cap.frame_ids)} | {nd} | {cap.duration_s:.1f} | {cap.path_length_m:.1f} | "
                 f"{sha(p / 'rgb.mp4')} | {sha(p / 'odometry.csv')} |")
    L += ["\nCapture style (from camera pitch): `single_room` and `floor_only` aim low (floor + lower walls); "
          "`with_ceiling` aims higher and is the only capture that observes ceilings.\n",
          "## Derived tier inputs (`data/tiers/`, by `scripts/make_tier_inputs.py`)\n",
          "| capture | video file | sha | photo folders (photos) |", "|---|---|---|---|"]
    for name in SOURCE:
        v = D / "tiers" / "video" / f"{name}.mp4"
        pdir = D / "tiers" / "photos" / name
        if v.exists():
            folders = sorted(d for d in pdir.iterdir() if d.is_dir()) if pdir.exists() else []
            desc = ", ".join(f"{d.name} ({len(list(d.glob('*.jpg')))})" for d in folders)
            L.append(f"| {name} | `{v.relative_to(ROOT)}` | {sha(v)} | {desc} |")
    st = D / "staged" / "single_room_staged" / "staged_truth.json"
    if st.exists():
        t = json.load(open(st))
        L += ["\n## Staged damage (`data/staged/single_room_staged/`, by `bench/stage_damage.py`)\n",
              "| id | class | room/wall (at staging) | size | truth |", "|---|---|---|---|---|"]
        for g in t["truth"]:
            tr = f"area {g['area_m2']:.4f} m²" if g.get("area_m2") else f"length {g['length_m']:.2f} m"
            L.append(f"| {g['id']} | {g['cls']} | {g['room']}/{g['wall']} | {g['width_m']}×{g['height_m']} m | {tr} |")
    L += ["\n## Not available (and why)\n",
          "- Laser / tape ground truth: no access to the property.",
          "- Consumer-app exports for head-to-head: would require scanning the same rooms with that app.",
          "- Physically staged damage: replaced by digital staging on the 3-D wall planes (above)."]
    (ROOT / "docs" / "DATA.md").write_text("\n".join(L) + "\n")
    print("wrote docs/DATA.md")


if __name__ == "__main__":
    main()
