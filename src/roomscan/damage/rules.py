"""Concealed-damage rules and scope line items.

Rules are explicit and auditable: each flag names the rule id that fired and the evidence.
Scope items are keyed to surface ids from the geometry contract and priced nowhere — they are
quantities, in the units an estimator would key into Xactimate-style software.
"""
from __future__ import annotations

RULES = {
    "CD-01": "Water stain on a ceiling: water source above (plumbing run, wet room or roof). "
             "Ceiling cavity likely wet beyond the visible stain.",
    "CD-02": "Water stain or mould on a wall within 0.30 m of the floor: rising damp or a leak behind the "
             "skirting; wall cavity / insulation at risk.",
    "CD-03": "Mould area > 0.10 m²: sustained moisture in the substrate; growth extends beyond what is visible.",
    "CD-04": "Crack starting within 0.15 m of an opening corner: possible lintel / structural movement; "
             "inspect behind the finish.",
    "CD-05": "Damage on a wall that is shared with another room: moisture or movement may also affect the "
             "adjoining room's face of the same wall.",
}

CODES = {
    # class, surface kind -> list of (code, description, quantity_fn, unit)
    ("water_stain", "wall"): [
        ("PNT-SEAL", "Stain-blocking primer over affected area + 0.3 m margin", "region_margin", "m2"),
        ("PNT-WALL", "Repaint full wall (finish to corners)", "surface_area", "m2"),
    ],
    ("water_stain", "ceiling"): [
        ("PNT-SEAL", "Stain-blocking primer over affected area + 0.3 m margin", "region_margin", "m2"),
        ("PNT-CEIL", "Repaint full ceiling", "surface_area", "m2"),
    ],
    ("crack", "wall"): [
        ("PLS-CRK", "Rake out, fill and tape crack", "length", "lm"),
        ("PNT-WALL", "Repaint full wall (finish to corners)", "surface_area", "m2"),
    ],
    ("crack", "ceiling"): [
        ("PLS-CRK", "Rake out, fill and tape crack", "length", "lm"),
        ("PNT-CEIL", "Repaint full ceiling", "surface_area", "m2"),
    ],
    ("mould", "wall"): [
        ("MLD-TRT", "Antimicrobial treatment, area + 50% margin", "region_x1.5", "m2"),
        ("PNT-WALL", "Repaint full wall (finish to corners)", "surface_area", "m2"),
    ],
    ("mould", "ceiling"): [
        ("MLD-TRT", "Antimicrobial treatment, area + 50% margin", "region_x1.5", "m2"),
        ("PNT-CEIL", "Repaint full ceiling", "surface_area", "m2"),
    ],
}
INVESTIGATE = ("INV-OPEN", "Moisture-meter survey + inspection opening (0.3 x 0.3 m)", "EA")


def concealed_flags(region, surface, room, shared_walls):
    flags = []
    c, kind = region["class"], surface["kind"]
    if c == "water_stain" and kind == "ceiling":
        flags.append("CD-01")
    if c in ("water_stain", "mould") and kind == "wall" and region["bbox"]["h_min_m"] < 0.30:
        flags.append("CD-02")
    if c == "mould" and region["area"]["value"] > 0.10:
        flags.append("CD-03")
    if c == "crack" and kind == "wall" and region.get("near_opening_corner"):
        flags.append("CD-04")
    if kind == "wall" and surface["ref"] in shared_walls:
        flags.append("CD-05")
    return [dict(rule_id=r, rule=RULES[r], surface_id=surface["id"], damage_id=region["id"],
                 evidence=f"{c} {region['area']['value']:.3f} m2 on {surface['id']}") for r in flags]


def scope_items(region, surface, surface_area, flags):
    items = []
    key = (region["class"], surface["kind"] if surface["kind"] != "floor" else "wall")
    for code, desc, qty, unit in CODES.get(key, []):
        if qty == "surface_area":
            q = surface_area
        elif qty == "region_margin":
            w, h = region["bbox"]["width_m"] + 0.6, region["bbox"]["height_m"] + 0.6
            q = min(w * h, surface_area)
        elif qty == "region_x1.5":
            q = min(region["area"]["value"] * 1.5, surface_area)
        elif qty == "length":
            q = region.get("length_m", max(region["bbox"]["width_m"], region["bbox"]["height_m"]))
        items.append(dict(code=code, description=desc, surface_id=surface["id"], quantity=round(float(q), 2),
                          unit=unit, reason=f"{region['id']} ({region['class']})"))
    if flags:
        items.append(dict(code=INVESTIGATE[0], description=INVESTIGATE[1], surface_id=surface["id"], quantity=1,
                          unit=INVESTIGATE[2], reason="concealed-damage rule(s) " + ",".join(f["rule_id"] for f in flags)))
    return items
