"""Tier-agnostic scene: what every capture tier must hand to the layout stage.

LiDAR, video and photo tiers differ only in how they produce this object. Everything after
(room segmentation, wall fitting, openings, stitching, damage, JSON) is shared code, which is
what makes "the same output contract from each tier" true by construction.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class Scene:
    tier: str                      # lidar | video | photo
    xyz: np.ndarray                # (N,3) world, metres, +y up
    normals: np.ndarray            # (N,3)
    cam_centres: np.ndarray        # (K,3) camera centres (world) for the views used
    views: list                    # [(cam_centre (3,), points_world (M,3))] for visibility
    path_length_m: float
    error_model: object            # roomscan.geometry.room.ErrorModel
    groups: list | None = None     # photo tier: list of (room_folder_name, view indices)
    images: list = field(default_factory=list)   # [(view_idx, image_path, K, T_cw)] for damage
    meta: dict = field(default_factory=dict)
