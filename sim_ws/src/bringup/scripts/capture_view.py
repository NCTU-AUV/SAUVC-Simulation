#!/usr/bin/env python3
"""Capture reference views of the running arena from arbitrary poses.

Spawns a static `inspection_camera` (RealSense-equivalent optics), teleports it
to each requested pose, and writes one PNG per pose. Used to eyeball scene
changes and to feed the offline YOLO probe.

    ros2 run bringup capture_view.py --out /root/captures \
        --view gate   3.5 0 -1.0  0 0 0 \
        --view drums 11.5 0 -1.2  0 0.3 0

Poses are `x y z roll pitch yaw` in world coordinates; a yaw of 0 looks along
+x. Entity management goes through the `ign service` CLI rather than ROS
because the launch file only bridges `/world/<world>/create` — talking to
Gazebo directly keeps this tool independent of the bridge configuration.
Requires GZ->ROS bridges for the inspection colour and depth topics, which the
script starts itself unless `--no-bridge` is given.

Captures are run through the same `WaterColumn` model the simulation applies at
runtime, so a reference shot shows the wet image the detector receives rather
than Gazebo's dry render. Pass `--dry` to see the render underneath it.
"""

import argparse
import math
import os
import shutil
import struct
import subprocess
import sys
import time
import zlib

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from bridge.underwater_camera_node import WaterColumn, decode_depth
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

WORLD = 'water_world'
CAMERA_MODEL = 'inspection_camera'
CAMERA_ENTITY = 'inspection_camera'
IMAGE_TOPIC = '/inspection/image'
DEPTH_TOPIC = '/inspection/depth_image'


def write_png(path: str, rgb: np.ndarray) -> None:
    """Write an HxWx3 uint8 array as a PNG without any imaging dependency."""
    height, width, _ = rgb.shape
    raw = b''.join(b'\x00' + rgb[row].tobytes() for row in range(height))

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (struct.pack('>I', len(data)) + tag + data
                + struct.pack('>I', zlib.crc32(tag + data) & 0xFFFFFFFF))

    with open(path, 'wb') as file:
        file.write(b'\x89PNG\r\n\x1a\n')
        file.write(chunk(b'IHDR', struct.pack('>2I5B', width, height, 8, 2, 0, 0, 0)))
        file.write(chunk(b'IDAT', zlib.compress(raw, 6)))
        file.write(chunk(b'IEND', b''))


def quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def gz_binary() -> str:
    for name in ('ign', 'gz'):
        if shutil.which(name):
            return name
    raise RuntimeError('Neither `ign` nor `gz` found on PATH')


def gz_service(service: str, reqtype: str, request: str, timeout_ms: int = 8000) -> str:
    """Call a Gazebo service and return stdout; raises on a false reply."""
    result = subprocess.run(
        [gz_binary(), 'service', '-s', f'/world/{WORLD}/{service}',
         '--reqtype', f'ignition.msgs.{reqtype}',
         '--reptype', 'ignition.msgs.Boolean',
         '--timeout', str(timeout_ms), '--req', request],
        capture_output=True, text=True, check=False)
    return (result.stdout or '') + (result.stderr or '')


class ViewCapturer(Node):
    def __init__(self, water: WaterColumn = None) -> None:
        super().__init__('view_capturer')
        self.water = water
        self.latest = None
        self.latest_depth = None
        self.latest_seq = 0
        self._settle_from = 0
        self.create_subscription(
            Image, IMAGE_TOPIC, self._on_image, qos_profile_sensor_data)
        self.create_subscription(
            Image, DEPTH_TOPIC, self._on_depth, qos_profile_sensor_data)

    def _on_image(self, msg: Image) -> None:
        self.latest = msg
        self.latest_seq += 1

    def _on_depth(self, msg: Image) -> None:
        self.latest_depth = msg

    def ensure_camera(self) -> None:
        """Create the inspection camera once; reuse it on later runs.

        Deliberately never removes an existing one. Tearing down an RGB-D
        sensor at runtime trips an Ogre-Next assertion inside
        Ogre2DepthCamera::Render ("This Datablock is still being used by some
        Renderables") and takes the whole server down with it — which surfaces
        only as sensor topics going silent. Moving the camera is enough.
        """
        if CAMERA_ENTITY in model_poses():
            self.get_logger().info('Reusing existing inspection camera')
            return

        sdf_path = os.path.join(get_package_share_directory('bringup'),
                                'models', CAMERA_MODEL, 'model.sdf')
        reply = gz_service(
            'create', 'EntityFactory',
            f'sdf_filename: "{sdf_path}" name: "{CAMERA_ENTITY}" allow_renaming: false')
        if 'true' not in reply.lower():
            raise RuntimeError(f'Failed to spawn inspection camera: {reply.strip()}')
        self.get_logger().info('Inspection camera spawned')

    def move_to(self, x, y, z, roll, pitch, yaw) -> None:
        qx, qy, qz, qw = quaternion_from_rpy(roll, pitch, yaw)
        reply = gz_service(
            'set_pose', 'Pose',
            f'name: "{CAMERA_ENTITY}" '
            f'position {{x: {x} y: {y} z: {z}}} '
            f'orientation {{x: {qx} y: {qy} z: {qz} w: {qw}}}')
        if 'true' not in reply.lower():
            raise RuntimeError(f'Failed to move inspection camera: {reply.strip()}')
        # Anything already rendered belongs to the old pose. Gazebo renders a
        # frame or two behind a pose change, and a stale frame paired with the
        # new pose's geometry is exactly how projected boxes end up floating in
        # open water.
        self._settle_from = self.latest_seq
        self.latest_depth = None

    def grab(self, settle_frames: int = 5, timeout: float = 20.0) -> np.ndarray:
        """Return a frame rendered well after the last pose change.

        Also waits for a depth frame from the same period, so callers that pair
        the two are not mixing poses.
        """
        target = self._settle_from + settle_frames
        deadline = time.time() + timeout
        while self.latest_seq < target or (self.water is not None and self.latest_depth is None):
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() > deadline:
                raise TimeoutError(f'No image on {IMAGE_TOPIC}')
        msg = self.latest
        image = np.frombuffer(msg.data, dtype=np.uint8)
        image = image.reshape(msg.height, msg.width, -1)[:, :, :3]
        if msg.encoding != 'rgb8':
            image = image[:, :, ::-1]
        if self.water is None:
            return np.ascontiguousarray(image)

        depth_msg = self.latest_depth
        if depth_msg is None:
            raise TimeoutError(f'No depth on {DEPTH_TOPIC}; cannot render water')
        distance = self.water.clean_range(decode_depth(depth_msg))
        wet = self.water.render(image.astype(np.float32) / 255.0, distance)
        return np.ascontiguousarray((wet * 255.0).astype(np.uint8))


def model_poses() -> dict:
    """Return {model_name: (x, y, z)} for every model in the running world."""
    listing = subprocess.run([gz_binary(), 'model', '--list'],
                             capture_output=True, text=True, check=False).stdout
    names = [line.strip()[2:] for line in listing.splitlines()
             if line.strip().startswith('- ')]
    poses = {}
    for name in names:
        out = subprocess.run([gz_binary(), 'model', '-m', name, '-p'],
                             capture_output=True, text=True, check=False).stdout
        for index, line in enumerate(out.splitlines()):
            if 'XYZ' in line:
                xyz = out.splitlines()[index + 1].strip().strip('[]').split()
                poses[name] = tuple(float(v) for v in xyz)
                break
    return poses


def arena_views(poses: dict) -> list:
    """Standard camera poses framing each prop, derived from where it spawned.

    Cameras approach from -x — the direction the AUV actually travels — and sit
    a fixed height above the prop's own base, so the same view list stays valid
    when the pool depth changes.
    """
    views = []

    def approach(name, target, distance, height, pitch=0.0):
        tx, ty, tz = target
        views.append([name, tx - distance, ty, tz + height, 0.0, pitch, 0.0])

    # Heights are relative to each prop's own origin. The qualification gate is
    # the odd one out: it hangs from the surface, so its origin is at z = 0 and
    # the offset has to go downwards to put the camera in the water.
    for model, distance, height in (('navigation_gate', 3.0, 0.55),
                                    ('orange_flare', 3.0, 0.75),
                                    ('red_flare', 2.0, 0.45),
                                    ('yellow_flare', 2.0, 0.45),
                                    ('blue_flare', 2.0, 0.45),
                                    ('q_gate_1', 3.5, -0.8),
                                    ('q_gate_2', 6.0, -0.6)):
        if model in poses:
            approach(model, poses[model], distance, height)

    if 'navigation_gate' in poses:
        approach('navigation_gate_far', poses['navigation_gate'], 6.0, 0.7)

    drums = [poses[n] for n in poses if 'drum' in n]
    if drums:
        centre = (min(d[0] for d in drums),
                  sum(d[1] for d in drums) / len(drums),
                  drums[0][2])
        approach('drums', centre, 3.0, 1.0, pitch=0.35)
        approach('drums_close', centre, 1.6, 0.9, pitch=0.55)

    if 'starting_zone' in poses or 'orca_auv' in poses:
        sx, sy, _ = poses.get('starting_zone', poses.get('orca_auv'))
        floor = min((p[2] for p in poses.values() if p[2] < -0.5), default=-1.6)
        views.append(['overview', sx + 0.5, sy, floor + 0.8, 0.0, 0.05, 0.0])

    return views


def start_image_bridge() -> subprocess.Popen:
    return subprocess.Popen(
        ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
         f'{IMAGE_TOPIC}@sensor_msgs/msg/Image[ignition.msgs.Image',
         f'{DEPTH_TOPIC}@sensor_msgs/msg/Image[ignition.msgs.Image'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', default='/root/captures')
    parser.add_argument('--prefix', default='')
    parser.add_argument('--no-bridge', action='store_true',
                        help='assume the image and depth bridges are already running')
    parser.add_argument('--dry', action='store_true',
                        help='capture Gazebo\'s raw render without the water model')
    parser.add_argument(
        '--view', nargs=7, action='append', default=[],
        metavar=('NAME', 'X', 'Y', 'Z', 'R', 'P', 'YAW'),
        help='view name followed by pose (repeatable)')
    parser.add_argument('--arena-views', action='store_true',
                        help='auto-generate one view per prop from its spawn pose')
    args = parser.parse_args()

    views = [[v[0]] + [float(c) for c in v[1:]] for v in args.view]
    if args.arena_views:
        views += arena_views(model_poses())
    if not views:
        parser.error('nothing to capture: pass --view and/or --arena-views')

    os.makedirs(args.out, exist_ok=True)
    bridge = None if args.no_bridge else start_image_bridge()

    rclpy.init()
    node = ViewCapturer(water=None if args.dry else WaterColumn())
    try:
        node.ensure_camera()
        time.sleep(1.5)
        for name, *pose in views:
            node.move_to(*(float(v) for v in pose))
            time.sleep(0.6)
            image = node.grab()
            path = os.path.join(args.out, f'{args.prefix}{name}.png')
            write_png(path, image)
            node.get_logger().info(f'Wrote {path}  ({image.shape[1]}x{image.shape[0]})')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if bridge is not None:
            bridge.terminate()
    return 0


if __name__ == '__main__':
    sys.exit(main())
