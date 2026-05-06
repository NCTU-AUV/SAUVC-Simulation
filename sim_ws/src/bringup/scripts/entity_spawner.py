#!/usr/bin/env python3

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

        self.models_path = os.path.join(get_package_share_directory('bringup'), 'models')
        self.pool_floor_z = -2.2
        self.world_name = 'water_world'

        self.spawn_client = self.create_client(SpawnEntity, f'/world/{self.world_name}/create')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /world/water_world/create...')

        self.get_logger().info(f'Spawn service ready, launching {self.arena} arena setup')

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

        self.get_logger().info(
            f'Spawned {entity_name} from {model_name} at x={x:.2f}, y={y:.2f}, z={z:.2f}, yaw={yaw:.2f}'
        )
        time.sleep(0.1)

    def spawn_vehicle(self, x: float, y: float, z: float, yaw: float) -> None:
        self.spawn_model('orca_auv', 'orca_auv', x, y, z, yaw=yaw)

    def spawn_qualification(self) -> None:
        start_line_x = -12.46
        gate_x = -2.5
        start_line_ys = (-3.0, 3.0)

        self.spawn_model('qualification_start_line', 'qualification_start_line_1', start_line_x, start_line_ys[0], 0.02, yaw=math.pi / 2.0)
        self.spawn_model('q_gate', 'qualification_gate_1', gate_x, start_line_ys[0], 0.0, yaw=math.pi / 2.0)

        self.spawn_model('qualification_start_line', 'qualification_start_line_2', start_line_x, start_line_ys[1], 0.02, yaw=math.pi / 2.0)
        self.spawn_model('q_gate', 'qualification_gate_2', gate_x, start_line_ys[1], 0.0, yaw=math.pi / 2.0)

        chosen_start_y = self.rng.choice(start_line_ys)
        self.spawn_vehicle(
            x=start_line_x + 0.5,
            y=chosen_start_y,
            z=-0.4,
            yaw=0.0,
        )
        self.get_logger().info(f'Qualification AUV spawned from starting line at y={chosen_start_y:.2f}')

    def spawn_finals(self) -> None:
        start_zone_x = -11.75
        start_zone_y = self.rng.uniform(-6.75, 6.75)
        start_zone_pose = (start_zone_x, start_zone_y, 0.02)
        gate_x = 3.5
        gate_y = self.rng.uniform(-6.75, 6.75)
        orange_x = self.rng.uniform(-8.5, -4.5)
        orange_y = self.rng.uniform(-7.5, 7.5)

        self.spawn_model('starting_zone', 'starting_zone', *start_zone_pose, yaw=0.0)
        self.spawn_vehicle(
            x=start_zone_x,
            y=start_zone_y,
            z=-0.4,
            yaw=0.0,
        )
        self.spawn_model('gate', 'navigation_gate', gate_x, gate_y, self.pool_floor_z, yaw=math.pi / 2.0)
        self.spawn_model('orange_flare', 'orange_flare', orange_x, orange_y, self.pool_floor_z, yaw=0.0)

        drum_positions = [
            (8.20, -2.10, self.pool_floor_z),
            (8.20, -0.70, self.pool_floor_z),
            (8.20, 0.70, self.pool_floor_z),
            (8.20, 2.10, self.pool_floor_z),
        ]
        shuffled_slots = drum_positions[:]
        self.rng.shuffle(shuffled_slots)
        self.spawn_model('blue_drum', 'blue_drum', *shuffled_slots[0], yaw=0.0)
        for index, drum_pose in enumerate(shuffled_slots[1:]):
            self.spawn_model('red_drum', f'red_drum_{index}', *drum_pose, yaw=0.0)
        self.get_logger().info(f'Drum order: {shuffled_slots}')

        placed_positions = [
            (gate_x, gate_y),
            (orange_x, orange_y),
            *[(x, y) for x, y, _ in drum_positions],
        ]

        for model_name, entity_name in (
            ('red_flare', 'red_flare'),
            ('yellow_flare', 'yellow_flare'),
            ('blue_flare', 'blue_flare'),
        ):
            x, y = self.sample_flare_position(placed_positions)
            placed_positions.append((x, y))
            self.spawn_model(model_name, entity_name, x, y, self.pool_floor_z, yaw=0.0)

    def sample_flare_position(self, occupied_xy):
        x_range = (-4.5, 3.5)
        y_range = (-7.5, 7.5)
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
