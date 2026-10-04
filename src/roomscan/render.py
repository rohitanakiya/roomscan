"""Render the stitched whole-property plan (SVG + PNG) from the output JSON.

Rendering from the JSON (not from internal objects) guarantees the picture shows exactly what
the contract says — and lets anyone re-render a plan from a saved result.
"""
from __future__ import annotations

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Arc, Polygon as MplPoly

ROOM_FILL = ["#EAF1FB", "#FDF1E4", "#EAF6EE", "#F6EAF4", "#FBF8E3", "#E9F4F6", "#F3EDE7", "#EEF0F7"]
DAMAGE_COL = {"water_stain": "#2B6CB0", "crack": "#C53030", "mould": "#2F855A", "hole": "#6B46C1",
              "peeling_paint": "#B7791F"}


def _fmt(m):
    return f"{m['value']:.2f}"


def render_plan(result: dict, path_png: str, path_svg: str | None = None, title: str | None = None):
    rooms = result["rooms"]
    if not rooms:
        return
    allp = np.vstack([np.array(r["polygon"]) for r in rooms])
    lo, hi = allp.min(0) - 0.8, allp.max(0) + 0.8
    span = hi - lo
    fig_w = 12
    fig_h = max(6, fig_w * span[1] / max(span[0], 1e-6))
    fig, ax = plt.subplots(figsize=(fig_w, min(fig_h, 18)))
    ax.set_facecolor("white")
    wall_w = 4.5
    for k, r in enumerate(rooms):
        P = np.array(r["polygon"])
        ax.add_patch(MplPoly(P, closed=True, fc=ROOM_FILL[k % len(ROOM_FILL)], ec="none", zorder=1))
        # walls with door/window gaps
        ops_by_wall = {}
        for o in r["openings"]:
            ops_by_wall.setdefault(o["wall_id"], []).append(o)
        for w in r["walls"]:
            a, b = np.array(w["start"]), np.array(w["end"])
            L = np.linalg.norm(b - a)
            if L < 1e-6:
                continue
            d = (b - a) / L
            cuts = []
            for o in ops_by_wall.get(w["id"], []):
                t0 = o["offset_from_wall_start_m"]   # measured along start -> end
                cuts.append((max(0, t0), min(L, t0 + o["width"]["value"]), o))
            cuts.sort(key=lambda c: c[0])
            pos = 0.0
            ls = "-" if w["face_observed"] else (0, (4, 2))
            for c0, c1, o in cuts + [(L, L, None)]:
                if c0 > pos:
                    p0, p1 = a + d * pos, a + d * c0
                    ax.plot([p0[0], p1[0]], [p0[1], p1[1]], color="#1A202C", lw=wall_w, ls=ls,
                            solid_capstyle="projecting", zorder=3)
                if o is not None:
                    p0, p1 = a + d * c0, a + d * c1
                    if o["type"] == "window":
                        ax.plot([p0[0], p1[0]], [p0[1], p1[1]], color="#3182CE", lw=wall_w * 0.6, zorder=3)
                    else:
                        # door swing arc on the room side
                        nrm = np.array([-d[1], d[0]])
                        cen = np.array(r["polygon"]).mean(0)
                        if np.dot(cen - p0, nrm) < 0:
                            nrm = -nrm
                        wd = c1 - c0
                        ax.plot([p0[0], p0[0] + nrm[0] * wd], [p0[1], p0[1] + nrm[1] * wd], color="#4A5568",
                                lw=1, zorder=4)
                        th0 = np.degrees(np.arctan2(d[1], d[0]))
                        th1 = np.degrees(np.arctan2(nrm[1], nrm[0]))
                        a0, a1 = sorted([th0, th1])
                        if a1 - a0 > 180:
                            a0, a1 = a1, a0 + 360
                        ax.add_patch(Arc(p0, 2 * wd, 2 * wd, theta1=a0, theta2=a1, color="#4A5568", lw=0.8,
                                         zorder=4))
                        mid = 0.5 * (p0 + p1)
                        ax.text(mid[0], mid[1], f"{o['width']['value']:.2f}", fontsize=6, color="#C53030",
                                ha="center", va="center", zorder=6,
                                bbox=dict(fc="white", ec="none", pad=0.3, alpha=0.8))
                pos = max(pos, c1)
            # dimension label (interior side)
            if L > 0.45:
                mid = 0.5 * (a + b)
                nrm = np.array([-d[1], d[0]])
                cen = np.array(r["polygon"]).mean(0)
                if np.dot(cen - mid, nrm) < 0:
                    nrm = -nrm
                lp = mid + nrm * 0.22
                rot = np.degrees(np.arctan2(d[1], d[0]))
                if rot > 90 or rot < -90:
                    rot += 180
                ax.text(lp[0], lp[1], _fmt(w["length"]), fontsize=6.5, ha="center", va="center",
                        rotation=rot, color="#2D3748", zorder=5)
        # damage regions (centroid markers)
        for dmg in r.get("damage", []):
            c = dmg.get("plan_xy")
            if c:
                ax.scatter([c[0]], [c[1]], s=40, marker="X", color=DAMAGE_COL.get(dmg["class"], "k"), zorder=7)
        cen = np.array(r["polygon"]).mean(0)
        from shapely.geometry import Polygon

        pp = Polygon(P).representative_point()
        cen = np.array([pp.x, pp.y])
        ch = r["ceiling_height"]
        ch_txt = f"h {ch['value']:.2f}" + ("" if ch["observed"] else " (est.)")
        ax.text(cen[0], cen[1], f"{r['name']}\n{r['floor_area']['value']:.1f} m²\n{ch_txt}", fontsize=8,
                ha="center", va="center", zorder=6, weight="bold", color="#1A202C")
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    ax.set_aspect("equal")
    ax.axis("off")
    # scale bar
    sb = np.array([lo[0] + 0.3, lo[1] + 0.3])
    ax.plot([sb[0], sb[0] + 1], [sb[1], sb[1]], "k-", lw=2)
    ax.text(sb[0] + 0.5, sb[1] + 0.12, "1 m", ha="center", fontsize=8)
    fp = result["property"]["footprint_area"]
    cap = result["capture"]
    t = title or f"{cap.get('tier', '').upper()} tier — {len(rooms)} rooms — footprint {fp['value']:.1f} m² " \
                 f"[{fp['ci95'][0]:.1f}, {fp['ci95'][1]:.1f}]"
    ax.set_title(t, fontsize=11)
    fig.tight_layout()
    fig.savefig(path_png, dpi=160)
    if path_svg:
        fig.savefig(path_svg)
    plt.close(fig)
