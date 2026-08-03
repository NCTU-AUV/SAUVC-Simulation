#!/usr/bin/env python3
"""Bake the pool floor and wall textures used by the `pool_ground` model.

`pool_tile_base.png` is a seamless 4x4 m patch of the venue's ceramic tiling
(16x16 tiles at 25 cm pitch). This script tiles it to the full pool extent and
draws the competition markings on top at their true metric positions: navy lane
lines with the usual end T-markers on the floor, and the waterline band plus
gutter on the walls.

Doing the markings in code rather than baking them into the source texture is
what keeps the geometry honest — Gazebo maps a <plane>'s UVs 0..1 across its
full extent, so a lane line drawn at y = 1.0 m in here lands at y = 1.0 m in
the world.

    ./make_pool_textures.py [--mm-per-px 6.0]

Colours and spacing are matched to the rulebook venue photographs
(`docs/sim-visual/reference/arena-2017.jpg`, `gate-2022.jpg`).
"""

import argparse
import os

import numpy as np
from PIL import Image

POOL_LENGTH_M = 25.0
POOL_WIDTH_M = 16.0
WALL_HEIGHT_M = 2.0

BASE_TILE_SPAN_M = 4.0   # real-world extent covered by pool_tile_base.png

LANE_SPACING_M = 2.0     # distance between lane centre lines
LANE_LINE_M = 0.25       # width of the navy lane stripe
LANE_T_OFFSET_M = 2.0    # distance from each end wall to the T cross-bar
LANE_T_LENGTH_M = 1.0    # length of the T cross-bar

GUTTER_M = 0.10          # pale coping strip along the very top of the wall
WATERLINE_M = 0.06       # navy band just below it

LANE_NAVY = np.array([26, 52, 104], np.uint8)
GUTTER = np.array([232, 236, 238], np.uint8)
WATERLINE = np.array([44, 74, 126], np.uint8)


def tiled(base: Image.Image, height_m, width_m, px_per_m) -> np.ndarray:
    """Repeat `base` across a height_m x width_m area at the given resolution."""
    patch_px = max(2, int(round(BASE_TILE_SPAN_M * px_per_m)))
    patch = np.asarray(base.resize((patch_px, patch_px), Image.LANCZOS), np.uint8)

    height_px = int(round(height_m * px_per_m))
    width_px = int(round(width_m * px_per_m))
    reps = (height_px // patch_px + 1, width_px // patch_px + 1, 1)
    return np.tile(patch, reps)[:height_px, :width_px].copy()


def make_floor(base, px_per_m) -> np.ndarray:
    image = tiled(base, POOL_WIDTH_M, POOL_LENGTH_M, px_per_m)
    height_px, width_px = image.shape[:2]

    ys = np.arange(height_px, dtype=np.float32) / px_per_m - POOL_WIDTH_M / 2.0
    xs = np.arange(width_px, dtype=np.float32) / px_per_m - POOL_LENGTH_M / 2.0

    half = LANE_LINE_M / 2.0
    lane_count = int(POOL_WIDTH_M // LANE_SPACING_M)
    for index in range(lane_count):
        centre = (index + 0.5) * LANE_SPACING_M - POOL_WIDTH_M / 2.0
        image[np.abs(ys - centre) < half, :] = LANE_NAVY

        for end in (-POOL_LENGTH_M / 2.0 + LANE_T_OFFSET_M,
                    POOL_LENGTH_M / 2.0 - LANE_T_OFFSET_M):
            bar = np.abs(xs - end) < half
            span = np.abs(ys - centre) < LANE_T_LENGTH_M / 2.0
            image[np.ix_(span, bar)] = LANE_NAVY

    return image


def make_wall(base, px_per_m) -> np.ndarray:
    """Wall panel; row 0 is the top of the wall once Gazebo maps it onto a box."""
    image = tiled(base, WALL_HEIGHT_M, POOL_LENGTH_M, px_per_m)
    from_top_m = np.arange(image.shape[0], dtype=np.float32) / px_per_m
    image[from_top_m < GUTTER_M] = GUTTER
    image[(from_top_m >= GUTTER_M) & (from_top_m < GUTTER_M + WATERLINE_M)] = WATERLINE
    return image


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    textures = os.path.abspath(os.path.join(
        here, '..', 'models', 'pool_ground', 'materials', 'textures'))

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', default=textures)
    parser.add_argument('--base', default=os.path.join(textures, 'pool_tile_base.png'))
    parser.add_argument('--mm-per-px', type=float, default=6.0)
    args = parser.parse_args()

    px_per_m = 1000.0 / args.mm_per_px
    base = Image.open(args.base).convert('RGB')
    os.makedirs(args.out, exist_ok=True)

    for name, array in (('pool_floor_competition', make_floor(base, px_per_m)),
                        ('pool_wall_competition', make_wall(base, px_per_m))):
        path = os.path.join(args.out, f'{name}.png')
        Image.fromarray(array).save(path, optimize=True)
        print(f'{path}  {array.shape[1]}x{array.shape[0]}  '
              f'{os.path.getsize(path) / 1e6:.1f} MB')


if __name__ == '__main__':
    main()
