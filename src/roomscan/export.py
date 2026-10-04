"""Property -> JSON (schema: schema/output.schema.json)."""
from __future__ import annotations

import json
import subprocess
from pathlib import Path

import numpy as np

from .measure import Measure, combine

SCHEMA_ID = "roomscan.output/1.0"
DOOR_HEIGHT_PRIOR = (2.05, 0.06)   # m, typical interior door leaf + frame; used when no header seen


def _git_rev():
    try:
        return subprocess.check_output(["git", "rev-parse", "--short", "HEAD"], cwd=Path(__file__).parent,
                                       stderr=subprocess.DEVNULL).decode().strip()
    except Exception:
        return "unknown"


def _r(x, nd=3):
    return [round(float(v), nd) for v in x]


def opening_measures(op, E):
    width = combine(op.width, jambs=op.width_sigma, sensor=E.opening_jamb, scale=E.scale_rel * op.width)
    if op.height is not None:
        height = combine(op.height, header_fit=op.height_sigma or 0.015, sensor=E.sensor_face)
    elif op.kind == "window":
        height = Measure(1.2, 0.3, "m", {"prior_unobserved": 0.3}, observed=False,
                         note="window head not observed")
    else:
        height = Measure(DOOR_HEIGHT_PRIOR[0], DOOR_HEIGHT_PRIOR[1], "m",
                         {"prior_unobserved": DOOR_HEIGHT_PRIOR[1]}, observed=False,
                         note="door header not observed; typical door height prior")
    return width, height


def property_to_json(prop, scene, extra=None):
    E = scene.error_model
    rooms_js = []
    by_label = {r.label: r for r in prop.rooms}
    traj = prop.frame.to_plan(scene.cam_centres)[:, :2] if len(scene.cam_centres) else np.zeros((0, 2))
    from matplotlib.path import Path as MplPath

    dwell = {r.id: int(MplPath(r.polygon).contains_points(traj).sum()) for r in prop.rooms}
    primary = max(dwell, key=dwell.get) if dwell else None
    for r in prop.rooms:
        walls_js, surfaces = [], []
        H = r.ceiling_height
        for w in r.walls:
            wid = f"{r.id}-W{w['index']}"
            L = w["length"]
            area = combine(L.value * H.value, unit="m2", length=L.sigma * H.value, height=H.sigma * L.value)
            walls_js.append(dict(
                id=wid, start=_r(w["start"]), end=_r(w["end"]), length=L.to_json(), height=H.to_json(),
                face_observed=bool(w["fit"].observed),
                face_rms_m=None if not np.isfinite(w["fit"].rms) else round(w["fit"].rms, 4),
                face_points=int(w["fit"].n)))
            surfaces.append(dict(id=f"S-{wid}", kind="wall", ref=wid, gross_area=area.to_json()))
        surfaces.append(dict(id=f"S-{r.id}-floor", kind="floor", ref=r.id, gross_area=r.floor_area.to_json()))
        surfaces.append(dict(id=f"S-{r.id}-ceiling", kind="ceiling", ref=r.id, gross_area=r.floor_area.to_json()))
        ops_js = []
        for k, op in enumerate(r.openings):
            w = r.walls[op.wall_index]
            width, height = opening_measures(op, E)
            ax_t = 0 if w["orient"] == "H" else 1
            start_t, end_t = w["start"][ax_t], w["end"][ax_t]
            off = (min(op.t0, op.t1) - start_t) if end_t >= start_t else (start_t - max(op.t0, op.t1))
            nb = by_label.get(op.neighbor_label)
            ops_js.append(dict(
                id=f"{r.id}-O{k}", type=op.kind, wall_id=f"{r.id}-W{op.wall_index}",
                width=width.to_json(), height=height.to_json(),
                sill_height=None if op.sill is None else round(op.sill, 3),
                offset_from_wall_start_m=round(float(off), 3),
                connects_to=nb.id if nb is not None and nb.id != r.id else "exterior_or_unscanned",
                see_through_fraction=round(op.through_frac, 2)))
        rooms_js.append(dict(
            id=r.id, name=r.meta.get("name", r.id), primary=(r.id == primary),
            polygon=[_r(p) for p in r.polygon],
            floor_area=r.floor_area.to_json(), perimeter=r.perimeter.to_json(),
            ceiling_height=r.ceiling_height.to_json(),
            walls=walls_js, openings=ops_js, surfaces=surfaces,
            damage=r.meta.get("damage", []), concealed_damage_flags=r.meta.get("concealed", []),
            scope_items=r.meta.get("scope", [])))
    meta = {k: v for k, v in scene.meta.items() if k not in ("poses", "kf", "K_rgb", "frame_ids", "frames_dir")}
    out = dict(
        schema=SCHEMA_ID,
        capture=dict(tier=scene.tier, **meta),
        pipeline=dict(version="0.1.0", git=_git_rev()),
        units=dict(length="m", area="m2"),
        plan_frame=dict(yaw_deg=round(prop.frame.yaw_deg, 3), floor_world_y=round(prop.frame.floor_y, 4)),
        property=dict(
            footprint_area=prop.footprint.to_json(),
            rooms_count=len(prop.rooms),
            room_overlap_m2=prop.meta.get("room_overlap_m2", 0.0),
            adjacency=prop.adjacency,
            drift=scene.meta.get("drift"),
        ),
        rooms=rooms_js,
        warnings=prop.meta.get("warnings", []),
    )
    if extra:
        out.update(extra)
    return out


def write_json(obj, path):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        json.dump(obj, f, indent=1)
