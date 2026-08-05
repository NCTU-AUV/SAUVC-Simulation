#!/usr/bin/env python3
"""Populate the running world with a SAUVC arena.

Prop placement follows the 2026 rulebook arena diagrams
(docs/sim-visual/reference/arena-2026-finals.png and -qualifiers.png), all
distances measured from the starting wall at x = -12.5:

    starting zone   at the wall
    orange flare    anywhere 4-8 m out
    coloured flares anywhere 8-16 m out
    navigation gate anywhere on the 16 m line
    target zone     the last ~2 m before the far wall

Randomisation is deliberate: the pinger drum order, prop positions and the
container shape all vary between attempts in the real competition, so a run
that only ever sees one layout tells you very little. Pass `seed` to make a
layout reproducible.
"""

import json
import math
import os
import random
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, Quaternion
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from ros_gz_interfaces.msg import EntityFactory
from ros_gz_interfaces.srv import SpawnEntity

# Pool geometry, metres. The floor is the deep section of the 2026 arena.
POOL_HALF_LENGTH = 12.5
POOL_HALF_WIDTH = 8.0
POOL_FLOOR_Z = -1.6

# Distances from the starting wall, per the rulebook arena diagram.
ORANGE_FLARE_ZONE = (4.0, 8.0)
COLOUR_FLARE_ZONE = (8.0, 16.0)
GATE_LINE = 16.0
TARGET_ZONE_DEPTH = 2.0

DRUM_STYLES = ('drum', 'tub')


class EntitySpawner(Node):
    def __init__(self):
        super().__init__('entity_spawner')

        self.declare_parameter(
            'arena',
            'finals',
            ParameterDescriptor(description='Arena profile to spawn: finals or qualification'),
        )
        self.declare_parameter(
            'seed',
            '',
            ParameterDescriptor(description='Optional random seed for reproducible prop placement'),
        )
        self.declare_parameter(
            'drum_style',
            'random',
            ParameterDescriptor(
                description='Target container shape: drum (round, per rulebook text), '
                            'tub (rectangular, as in the competition photos), or random'),
        )

        self.arena = self.get_parameter('arena').get_parameter_value().string_value.strip().lower()
        if self.arena not in {'finals', 'qualification'}:
            raise ValueError(f'Unsupported arena profile: {self.arena}')

        seed_text = self.get_parameter('seed').get_parameter_value().string_value.strip()
        self.rng = random.Random()
        if seed_text:
            self.rng.seed(int(seed_text))
            self.get_logger().info(f'Using random seed {seed_text}')
        else:
            self.rng.seed()

        style = self.get_parameter('drum_style').get_parameter_value().string_value.strip().lower()
        if style not in set(DRUM_STYLES) | {'random'}:
            raise ValueError(f'Unsupported drum style: {style}')
        # 用獨立的 Random 抽形狀，不要動 self.rng。從 self.rng 抽會消耗一次
        # 抽樣，把後面所有佈局抽樣平移一格 —— 於是同一個 SEED 在
        # DRUM_STYLE=random 與 DRUM_STYLE=drum 之下產生**不同的場地**，即使
        # 兩者解析出來的形狀一樣。實測 seed 42：random 得到
        # [-4.3, -2.3, -2.714, ...]，drum 得到 [0.615, -4.3, -2.3, ...]。
        # 這會讓 docs/HANDOFF.md §6 的過門軌跡（走預設 random）與
        # docs/SIM_VISUAL_FIDELITY.md §6 的偵測基準（DRUM_STYLE=drum）
        # 明明都寫 seed=42，卻不是同一個場地。
        if style == 'random':
            style_rng = random.Random()
            style_rng.seed(int(seed_text) if seed_text else None)
            self.drum_style = style_rng.choice(DRUM_STYLES)
        else:
            self.drum_style = style
        self._spawned: dict[str, str] = {}

        self.models_path = os.path.join(get_package_share_directory('bringup'), 'models')
        self.pool_floor_z = POOL_FLOOR_Z
        self.world_name = 'water_world'

        self.spawn_client = self.create_client(SpawnEntity, f'/world/{self.world_name}/create')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /world/water_world/create...')

        self.get_logger().info(
            f'Spawn service ready, launching {self.arena} arena setup '
            f'(drum style: {self.drum_style})')

    @staticmethod
    def from_start_wall(distance: float) -> float:
        """Convert a rulebook distance from the starting wall into a world x."""
        return -POOL_HALF_LENGTH + distance

    def load_model_sdf(self, model_name: str) -> str:
        sdf_path = os.path.join(self.models_path, model_name, 'model.sdf')
        with open(sdf_path, 'r', encoding='utf-8') as file:
            return file.read()

    def spawn_model(self, model_name: str, entity_name: str, x: float, y: float, z: float, yaw: float = 0.0) -> None:
        request = SpawnEntity.Request()
        entity_factory = EntityFactory()
        entity_factory.name = entity_name
        entity_factory.allow_renaming = False
        entity_factory.sdf = self.load_model_sdf(model_name)
        entity_factory.pose = Pose(
            position=Point(x=x, y=y, z=z),
            orientation=Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)),
        )
        entity_factory.relative_to = 'world'
        request.entity_factory = entity_factory

        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is None or not future.result().success:
            raise RuntimeError(f'Failed to spawn {entity_name}: {future.result()}')

        self._spawned[entity_name] = model_name
        self.get_logger().info(
            f'Spawned {entity_name} from {model_name} at x={x:.2f}, y={y:.2f}, z={z:.2f}, yaw={yaw:.2f}'
        )
        time.sleep(0.1)

    def spawn_vehicle(self, x: float, y: float, z: float, yaw: float) -> None:
        self.spawn_model('orca_auv', 'orca_auv', x, y, z, yaw=yaw)

    def spawn_qualification(self) -> None:
        start_line_x = -POOL_HALF_LENGTH + 0.04
        gate_x = self.from_start_wall(10.0)
        start_line_ys = (-3.0, 3.0)

        for index, start_y in enumerate(start_line_ys, start=1):
            self.spawn_model('qualification_start_line', f'qualification_start_line_{index}',
                             start_line_x, start_y, 0.02, yaw=math.pi / 2.0)
            # The gate hangs from the surface, so it spawns at z = 0.
            self.spawn_model('q_gate', f'q_gate_{index}', gate_x, start_y, 0.0, yaw=math.pi / 2.0)

        chosen_start_y = self.rng.choice(start_line_ys)
        self.spawn_vehicle(
            x=start_line_x + 0.5,
            y=chosen_start_y,
            z=-0.4,
            yaw=0.0,
        )
        self.get_logger().info(f'Qualification AUV spawned from starting line at y={chosen_start_y:.2f}')

    def spawn_finals(self) -> None:
        lateral_margin = 1.25
        y_limit = POOL_HALF_WIDTH - lateral_margin

        start_zone_x = self.from_start_wall(0.75)
        start_zone_y = self.rng.uniform(-y_limit, y_limit)
        gate_x = self.from_start_wall(GATE_LINE)
        gate_y = self.rng.uniform(-y_limit, y_limit)
        orange_x = self.from_start_wall(self.rng.uniform(*ORANGE_FLARE_ZONE))
        orange_y = self.rng.uniform(-y_limit, y_limit)

        self.spawn_model('starting_zone', 'starting_zone', start_zone_x, start_zone_y, 0.02, yaw=0.0)
        self.spawn_vehicle(x=start_zone_x, y=start_zone_y, z=-0.4, yaw=0.0)
        self.spawn_model('gate', 'navigation_gate', gate_x, gate_y, self.pool_floor_z,
                         yaw=math.pi / 2.0)
        self.spawn_model('orange_flare', 'orange_flare', orange_x, orange_y, self.pool_floor_z,
                         yaw=self.rng.uniform(-math.pi, math.pi))

        drum_positions = self.sample_drum_positions()
        blue_model = 'blue_tub' if self.drum_style == 'tub' else 'blue_drum'
        red_model = 'red_tub' if self.drum_style == 'tub' else 'red_drum'

        # Entity names stay shape-independent so downstream tooling (and the
        # detector's class names) do not care which variant was spawned.
        self.spawn_model(blue_model, 'blue_drum', *drum_positions[0],
                         yaw=self.rng.uniform(-0.25, 0.25))
        for index, drum_pose in enumerate(drum_positions[1:]):
            self.spawn_model(red_model, f'red_drum_{index}', *drum_pose,
                             yaw=self.rng.uniform(-0.25, 0.25))
        self.get_logger().info(
            f'Target zone: {self.drum_style} shape, blue at {drum_positions[0][:2]}')

        placed_positions = [(gate_x, gate_y), (orange_x, orange_y),
                            *[(x, y) for x, y, _ in drum_positions]]

        for model_name, entity_name in (('red_flare', 'red_flare'),
                                        ('yellow_flare', 'yellow_flare'),
                                        ('blue_flare', 'blue_flare')):
            x, y = self.sample_flare_position(placed_positions)
            placed_positions.append((x, y))
            self.spawn_model(model_name, entity_name, x, y, self.pool_floor_z, yaw=0.0)

    def sample_drum_positions(self):
        """Four slots in a line across the target zone, in randomised order.

        The target zone is the last ~2 m of the pool; drums sit in a row
        parallel to the far wall with the blue one anywhere among them.
        """
        centre_x = POOL_HALF_LENGTH - TARGET_ZONE_DEPTH / 2.0
        spacing = self.rng.uniform(1.4, 2.2)
        offset = self.rng.uniform(-1.0, 1.0)
        slots = [(centre_x + self.rng.uniform(-0.25, 0.25),
                  offset + (index - 1.5) * spacing,
                  self.pool_floor_z)
                 for index in range(4)]
        self.rng.shuffle(slots)
        return slots

    def sample_flare_position(self, occupied_xy):
        x_range = (self.from_start_wall(COLOUR_FLARE_ZONE[0]),
                   self.from_start_wall(COLOUR_FLARE_ZONE[1]))
        y_range = (-(POOL_HALF_WIDTH - 0.5), POOL_HALF_WIDTH - 0.5)
        minimum_clearance = 1.25

        for _ in range(200):
            x = self.rng.uniform(*x_range)
            y = self.rng.uniform(*y_range)
            if all(math.dist((x, y), other) >= minimum_clearance for other in occupied_xy):
                return x, y
        raise RuntimeError('Unable to find a valid position for communication flare placement')

    def spawn_all(self) -> None:
        if self.arena == 'qualification':
            self.spawn_qualification()
        else:
            self.spawn_finals()
        self.write_manifest()

    def write_manifest(self) -> None:
        """記下每個實體實際是用哪個模型生成的。

        實體名稱刻意與外形無關（tub 也叫 blue_drum），所以偵測器與下游工具
        不必在意生成的是哪個變體。但資料集產生器需要知道 —— 圓桶與方形箱的
        尺寸差很多（0.60x0.60 vs 0.66x0.44），套錯會產生系統性偏掉的標註框，
        而且因為實體名稱一模一樣，沒有任何地方會發現。

        用檔案而不是 latched topic：這個節點生成完就結束，transient_local
        的發布者會跟著消失，晚一步啟動的訂閱者收不到。
        """
        path = os.environ.get('ORCA_ARENA_MANIFEST', '/tmp/orca_arena_manifest.json')
        payload = {'arena': self.arena, 'drum_style': self.drum_style, 'entities': self._spawned}
        try:
            with open(path, 'w', encoding='utf-8') as file:
                json.dump(payload, file, indent=2, sort_keys=True)
            self.get_logger().info(f'Wrote arena manifest to {path}')
        except OSError as exc:
            self.get_logger().warning(f'Could not write arena manifest to {path}: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = EntitySpawner()
    try:
        node.spawn_all()
        time.sleep(0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
