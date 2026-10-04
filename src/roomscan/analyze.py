"""Scene -> measured property (rooms, walls, openings, adjacency, stitched plan)."""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .geometry.frame import PlanFrame
from .geometry.layout import LayoutParams, build_rasters, grow_to_walls, segment_rooms
from .geometry.room import build_room
from .measure import combine


@dataclass
class Property:
    tier: str
    frame: PlanFrame
    rooms: list
    adjacency: list
    rasters: object
    labels: np.ndarray
    footprint: object
    meta: dict = field(default_factory=dict)


def analyze(scene, P: LayoutParams | None = None, log=print) -> Property:
    P = P or LayoutParams()
    F = PlanFrame.fit(scene.xyz, scene.normals)
    q = F.to_plan(scene.xyz)
    nq = F.normals_to_plan(scene.normals)
    R = build_rasters(scene.views, scene.cam_centres, F, q, nq, P)
    if scene.groups:
        from .tiers.photo import segment_by_groups
        labels, stats = segment_by_groups(scene, F, R, P)
    else:
        labels, stats, _ = segment_rooms(R, P)
        labels = grow_to_walls(labels, R.wall, R.grid.res, P.grow_to_walls)
        if P.cell_geometry:
            from .geometry.cells import cells_from_labels

            labels2, _ = cells_from_labels(q, nq, R, labels)
            # keep a room only if the cell geometry kept most of it
            keep = []
            for s_ in stats:
                a0 = (labels == s_["label"]).sum()
                a1 = (labels2 == s_["label"]).sum()
                if a1 > 0.5 * a0:
                    keep.append(s_)
            stats, labels = keep, labels2
    log(f"  layout: yaw {F.yaw_deg:.1f} deg, {len(stats)} room regions")
    rooms = []
    for s in stats:
        k = s["label"]
        try:
            r = build_room(k, labels == k, R, labels, q, nq, P, scene.error_model,
                           path_len_m=scene.path_length_m, room_id=f"R{k}")
        except Exception as e:  # a single bad region must not kill the capture
            log(f"  room {k}: skipped ({e})")
            continue
        r.meta.update(s)
        if s.get("name"):
            r.meta["name"] = s["name"]
        rooms.append(r)

    # adjacency from doors/passages whose far side lands in another room
    by_label = {r.label: r for r in rooms}
    adj = {}
    for r in rooms:
        for op in r.openings:
            nb = op.neighbor_label
            if op.kind in ("door", "passage") and nb in by_label and nb != r.label:
                key = tuple(sorted((r.id, by_label[nb].id)))
                adj.setdefault(key, []).append(dict(room=r.id, wall=op.wall_index, kind=op.kind,
                                                    width=round(op.width, 3)))
    adjacency = [dict(rooms=list(k), via=v) for k, v in adj.items()]

    # footprint = union of room polygons
    from shapely.geometry import Polygon
    from shapely.ops import unary_union

    polys = [Polygon(r.polygon).buffer(0) for r in rooms if len(r.polygon) >= 3]
    U = unary_union(polys) if polys else None
    fp_area = float(U.area) if U is not None else 0.0
    fp_sigma = float(np.sqrt(sum(r.floor_area.sigma ** 2 for r in rooms))) if rooms else 0.0
    footprint = combine(fp_area, unit="m2", rooms=fp_sigma)
    overlaps = 0.0
    for i in range(len(polys)):
        for j in range(i + 1, len(polys)):
            overlaps += polys[i].intersection(polys[j]).area
    return Property(scene.tier, F, rooms, adjacency, R, labels, footprint,
                    meta=dict(room_overlap_m2=round(overlaps, 4), q_points=len(q)))
