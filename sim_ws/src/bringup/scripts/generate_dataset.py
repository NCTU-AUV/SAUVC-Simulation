#!/usr/bin/env python3
"""Generate a labelled YOLO detection dataset from the running arena.

Flies the inspection camera to random poses along a plausible mission path,
captures the wet image the perception stack would receive, and derives the
bounding boxes analytically from where Gazebo says each prop is. Labels are
therefore exact and free — no hand annotation, and no chance of the labels
drifting out of sync with the scene.

    ros2 run bringup generate_dataset.py --out /root/dataset --count 500

Output is a standard Ultralytics layout:

    <out>/images/train/*.png
    <out>/labels/train/*.txt      class cx cy w h, normalised
    <out>/data.yaml

Two things keep the labels honest rather than merely correct:

  * A prop whose light is almost entirely eaten by the water column is dropped.
    Labelling something the camera physically cannot see teaches a detector to
    hallucinate.
  * Boxes are depth-tested against the rendered depth buffer, so props hidden
    behind a pool wall are not labelled as visible.
"""

import argparse
import math
import os
import random
import sys
import time

import numpy as np
import rclpy

from capture_view import (DEPTH_TOPIC, ViewCapturer, model_poses, quaternion_from_rpy,
                          start_image_bridge, write_png)
from bridge.underwater_camera_node import WaterColumn, decode_depth

# Detector class order, matching orca_perception's perception_params.yaml.
CLASS_NAMES = ['blue_drum', 'blue_flare', 'gate', 'orange_flare',
               'red_drum', 'red_flare', 'yellow_flare']

# Axis-aligned extents of each prop in its own frame, origin on the pool floor:
# (half_x, half_y, z_min, z_max). Kept in step with the models by hand — these
# are the same numbers the SDF geometry is built from.
PROP_EXTENTS = {
    'gate': (0.79, 0.58, 0.0, 1.02),
    'orange_flare': (0.11, 0.11, 0.0, 1.54),
    'red_flare': (0.07, 0.07, 0.0, 0.873),
    'yellow_flare': (0.07, 0.07, 0.0, 0.873),
    'blue_flare': (0.07, 0.07, 0.0, 0.873),
    'red_drum': (0.30, 0.30, 0.0, 0.30),
    'blue_drum': (0.30, 0.30, 0.0, 0.30),
}

# Entity name prefix -> (class name, extents key). Entity names are shape
# independent, so a tub spawn still lands on the drum class.
ENTITY_CLASSES = [
    ('navigation_gate', 'gate', 'gate'),
    ('orange_flare', 'orange_flare', 'orange_flare'),
    ('red_flare', 'red_flare', 'red_flare'),
    ('yellow_flare', 'yellow_flare', 'yellow_flare'),
    ('blue_flare', 'blue_flare', 'blue_flare'),
    ('blue_drum', 'blue_drum', 'blue_drum'),
    ('red_drum', 'red_drum', 'red_drum'),
]

MIN_BOX_PX = 6        # boxes thinner than this are not worth a label
NEAR_PLANE_M = 0.06   # matches the inspection camera's near clip
MIN_CONTRAST = 0.022  # mean |box - surround| below this means it is lost in haze

# The 12 edges of an axis-aligned box, indexing prop_corners()' output. Corner
# i encodes (sx, sy, sz) as bits 2, 1, 0, so edges join corners differing in
# exactly one bit.
BOX_EDGES = [(i, i ^ bit) for i in range(8) for bit in (1, 2, 4) if i < (i ^ bit)]


def classify(entity_name):
    for prefix, class_name, extents_key in ENTITY_CLASSES:
        if entity_name.startswith(prefix):
            return class_name, PROP_EXTENTS[extents_key]
    return None, None


def intrinsics(width, height, horizontal_fov):
    focal = (width / 2.0) / math.tan(horizontal_fov / 2.0)
    return focal, width / 2.0, height / 2.0


def world_to_camera(points, camera_xyz, yaw, pitch):
    """Express world points in the camera frame (x right, y down, z forward).

    Gazebo cameras look along their own +x with +z up, so the rotation is
    undone first and the axes are then permuted into the usual optical frame.
    """
    relative = points - np.asarray(camera_xyz, np.float32)

    cy, sy = math.cos(-yaw), math.sin(-yaw)
    rotated = np.stack([relative[:, 0] * cy - relative[:, 1] * sy,
                        relative[:, 0] * sy + relative[:, 1] * cy,
                        relative[:, 2]], axis=1)

    cp, sp = math.cos(pitch), math.sin(pitch)
    forward = rotated[:, 0] * cp - rotated[:, 2] * sp
    up = rotated[:, 0] * sp + rotated[:, 2] * cp
    return np.stack([-rotated[:, 1], -up, forward], axis=1)


def prop_corners(pose, extents):
    x, y, z, yaw = pose
    half_x, half_y, z_min, z_max = extents
    local = np.array([[sx * half_x, sy * half_y, sz]
                      for sx in (-1, 1) for sy in (-1, 1) for sz in (z_min, z_max)],
                     np.float32)
    cy, sy_ = math.cos(yaw), math.sin(yaw)
    rotated = np.stack([local[:, 0] * cy - local[:, 1] * sy_,
                        local[:, 0] * sy_ + local[:, 1] * cy,
                        local[:, 2]], axis=1)
    return rotated + np.array([x, y, z], np.float32)


def clip_to_near_plane(corners_cam):
    """Points describing the prop's extent in front of the camera.

    Simply dropping corners behind the camera silently loses props the camera
    is right on top of — exactly the close-range frames a detector most needs.
    Clipping each box edge where it crosses the near plane keeps them.
    """
    front = [corner for corner in corners_cam if corner[2] > NEAR_PLANE_M]
    for start, end in BOX_EDGES:
        a, b = corners_cam[start], corners_cam[end]
        if (a[2] > NEAR_PLANE_M) == (b[2] > NEAR_PLANE_M):
            continue
        t = (NEAR_PLANE_M - a[2]) / (b[2] - a[2])
        front.append(a + t * (b - a))
    return np.asarray(front, np.float32) if front else None


def project_box(corners_cam, focal, cx, cy, width, height):
    """Clipped pixel-space box, or None if the prop is not in frame."""
    front = clip_to_near_plane(corners_cam)
    if front is None or len(front) < 2:
        return None

    us = focal * front[:, 0] / front[:, 2] + cx
    vs = focal * front[:, 1] / front[:, 2] + cy
    x0, x1 = float(us.min()), float(us.max())
    y0, y1 = float(vs.min()), float(vs.max())
    if x1 <= 0 or y1 <= 0 or x0 >= width or y0 >= height:
        return None

    box = (max(x0, 0.0), max(y0, 0.0), min(x1, width - 1.0), min(y1, height - 1.0))
    if box[2] - box[0] < MIN_BOX_PX or box[3] - box[1] < MIN_BOX_PX:
        return None
    return box


def occluded(box, distance_map, prop_distance):
    """True when the rendered depth across the box sits well in front."""
    x0, y0, x1, y1 = (int(round(v)) for v in box)
    patch = distance_map[max(y0, 0):max(y1 + 1, y0 + 1), max(x0, 0):max(x1 + 1, x0 + 1)]
    if patch.size == 0:
        return True
    return float(np.percentile(patch, 5)) < prop_distance - 0.6


def contrast(image, box):
    """Mean per-channel difference between the box and the ring around it.

    Attenuation alone is a poor visibility test: green barely attenuates, yet a
    prop 15 m out is invisible because the veiling light has swamped its
    contrast. Measuring the rendered frame answers the question directly, and
    keeps the labels tied to what the camera can actually resolve.
    """
    height, width = image.shape[:2]
    x0, y0, x1, y1 = (int(round(v)) for v in box)
    inner = image[y0:y1 + 1, x0:x1 + 1]
    if inner.size == 0:
        return 0.0

    pad = max(6, (x1 - x0) // 2, (y1 - y0) // 2)
    ox0, oy0 = max(x0 - pad, 0), max(y0 - pad, 0)
    ox1, oy1 = min(x1 + pad, width - 1), min(y1 + pad, height - 1)
    outer = image[oy0:oy1 + 1, ox0:ox1 + 1]
    if outer.size <= inner.size:
        return 0.0

    total = outer.reshape(-1, 3).sum(axis=0) - inner.reshape(-1, 3).sum(axis=0)
    ring_mean = total / float(outer.shape[0] * outer.shape[1] - inner.shape[0] * inner.shape[1])
    return float(np.abs(inner.reshape(-1, 3).mean(axis=0) - ring_mean).mean()) / 255.0


def sample_pose(rng, floor_z):
    """A pose along a plausible mission path, aimed roughly down-course."""
    x = rng.uniform(-11.5, 11.0)
    y = rng.uniform(-7.0, 7.0)
    z = rng.uniform(floor_z + 0.25, -0.25)
    yaw = rng.gauss(0.0, 0.9) if rng.random() < 0.75 else rng.uniform(-math.pi, math.pi)
    pitch = rng.uniform(-0.15, 0.55)
    return x, y, z, 0.0, pitch, yaw


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', default='/root/dataset')
    parser.add_argument('--count', type=int, default=200)
    parser.add_argument('--split', default='train')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--horizontal-fov', type=float, default=1.211)
    parser.add_argument('--no-bridge', action='store_true')
    parser.add_argument('--randomize-water', action='store_true',
                        help='draw a fresh water condition for every frame')
    args = parser.parse_args()

    images_dir = os.path.join(args.out, 'images', args.split)
    labels_dir = os.path.join(args.out, 'labels', args.split)
    os.makedirs(images_dir, exist_ok=True)
    os.makedirs(labels_dir, exist_ok=True)

    rng = random.Random(args.seed)
    water = WaterColumn()
    water.seed(args.seed)

    bridge = None if args.no_bridge else start_image_bridge()
    rclpy.init()
    node = ViewCapturer(water=water)
    written = 0
    try:
        node.ensure_camera()
        time.sleep(1.5)

        poses = model_poses()
        props = [(name, *classify(name)) for name in poses]
        props = [(name, cls, ext) for name, cls, ext in props if cls]
        if not props:
            node.get_logger().error('No known props in the world; nothing to label')
            return 1
        floor_z = min(poses[name][2] for name, _, _ in props)
        node.get_logger().info(f'Labelling {len(props)} props, floor at z={floor_z:.2f}')

        # Prop yaw is needed for the corner maths and `ign model -p` only gave
        # us position, so read the full pose once — props do not move.
        prop_poses = {}
        for name, _, _ in props:
            x, y, z = poses[name]
            prop_poses[name] = (x, y, z, node_yaw(name))

        for index in range(args.count):
            if args.randomize_water:
                water.randomize([0.18, 0.42, 0.035, 0.10, 0.02, 0.07],
                                [0.75, 1.25], [0.50, 0.70])

            pose = sample_pose(rng, floor_z)
            node.move_to(*pose)
            time.sleep(0.35)
            image = node.grab(settle_frames=6)
            height, width = image.shape[:2]
            focal, cx, cy = intrinsics(width, height, args.horizontal_fov)

            depth_msg = node.latest_depth
            distance_map = water.clean_range(decode_depth(depth_msg))

            x, y, z, _, pitch, yaw = pose
            lines = []
            for name, class_name, extents in props:
                corners = prop_corners(prop_poses[name], extents)
                corners_cam = world_to_camera(corners, (x, y, z), yaw, pitch)
                box = project_box(corners_cam, focal, cx, cy, width, height)
                if box is None:
                    continue

                in_front = corners_cam[corners_cam[:, 2] > NEAR_PLANE_M]
                prop_distance = float(np.median(in_front[:, 2])) if len(in_front) else NEAR_PLANE_M
                if occluded(box, distance_map, prop_distance):
                    continue
                if contrast(image, box) < MIN_CONTRAST:
                    continue

                x0, y0, x1, y1 = box
                lines.append(
                    f'{CLASS_NAMES.index(class_name)} '
                    f'{((x0 + x1) / 2) / width:.6f} {((y0 + y1) / 2) / height:.6f} '
                    f'{(x1 - x0) / width:.6f} {(y1 - y0) / height:.6f}')

            stem = f'{args.split}_{index:05d}'
            write_png(os.path.join(images_dir, f'{stem}.png'), image)
            with open(os.path.join(labels_dir, f'{stem}.txt'), 'w', encoding='utf-8') as file:
                file.write('\n'.join(lines) + ('\n' if lines else ''))
            written += 1
            if written % 25 == 0:
                node.get_logger().info(f'{written}/{args.count} frames')

        with open(os.path.join(args.out, 'data.yaml'), 'w', encoding='utf-8') as file:
            file.write(f'path: {os.path.abspath(args.out)}\n')
            file.write(f'{args.split}: images/{args.split}\n')
            file.write(f'nc: {len(CLASS_NAMES)}\n')
            file.write(f'names: {CLASS_NAMES}\n')
        node.get_logger().info(f'Wrote {written} frames to {args.out}')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if bridge is not None:
            bridge.terminate()
    return 0


def node_yaw(name: str) -> float:
    """Yaw of a model, read straight from Gazebo."""
    import subprocess

    from capture_view import gz_binary
    out = subprocess.run([gz_binary(), 'model', '-m', name, '-p'],
                         capture_output=True, text=True, check=False).stdout
    lines = out.splitlines()
    for index, line in enumerate(lines):
        if 'XYZ' in line and index + 2 < len(lines):
            rpy = lines[index + 2].strip().strip('[]').split()
            return float(rpy[2])
    return 0.0


if __name__ == '__main__':
    sys.exit(main())
