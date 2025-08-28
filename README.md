# SAUVC-Gazebo-Simulation

## How to Start

1. Run `make build_image`
2. Run `make`
3. Run `ign launch -v 4 orca_auv_gazebo_simulation_ws/websocket.ign`
4. To another terminal run `make enter_container`, and run `ign gazebo -v 4 -s -r <world_file.sdf>`.
5. To [https://app.gazebosim.org/visualization](https://app.gazebosim.org/visualization) and connect to [wss://localhost:9002](wss://localhost:9002) (at least I know Safari works).

## Test Thrustes

Use `ign topic -t /orca_auv/thruster_0/set_output_force_N -m ignition.msgs.Double -p 'data: 5'`.

## Bridge to ROS2

Run `ros2 launch sauvc_pkg sauvc_ros_gz_bridge_launch.py`.

## How to Create a Model

Following [Blender SDF Exporter](https://github.com/gazebosim/gz-sim/blob/ignition-gazebo6_6.17.0/tutorials/blender_sdf_exporter.md), 
use Blender 2.82 to make the model and its texture and use 
[sdf_exporter.py](https://github.com/gazebosim/gz-sim/blob/ign-gazebo5/examples/scripts/blender/sdf_exporter.py)
to export the model to .sdf.

## Reference

[How to use web visualization](https://gazebosim.org/docs/fortress/web_visualization/)

[Simulating and Testing underwater robots in GazeboSim](https://app.theconstruct.ai/rosjects/946878/)
