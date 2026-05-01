# Orca AUV Gazebo Simulation

## How to Start

### for macOS

1. `make compose_init` (generates TLS certs, starts Docker, installs deps, and builds the workspace; first time or after source changes)
2. `make compose_launch` to run the websocket server, Gazebo server, and `ros2 launch bringup orca_ros_gz_bridge_launch.py`
3. Go to [https://app.gazebosim.org/visualization](https://app.gazebosim.org/visualization) and connect to [wss://localhost:9002](wss://localhost:9002) (Safari confirmed to work).

> Note: `mkcert` must be installed locally for certificate generation.
> By default `make compose_launch` uses `/root/sim_ws/src/bringup/worlds/water_world.sdf` and namespace `orca_auv`. Override them with `make compose_launch WORLD=/root/sim_ws/src/bringup/worlds/<world_file>.sdf NAMESPACE=<name>`.

### for Ubuntu

1. Run `nvidia-smi` to check NVIDIA driver.
2. install glxinfo and confirm OpenGL uses NVIDIA

    ```bash
    sudo apt update
    sudo apt install -y mesa-utils
    glxinfo -B | egrep -i "OpenGL vendor|OpenGL renderer|direct rendering" 
    ```

    If apt complains: dpkg was interrupted..., fix it first

    ```bash
    sudo dpkg --configure -a
    sudo apt -f install
    sudo apt update
    ```

3. Install Docker, Compose plugin, and make.

    ```bash
    sudo apt update
    sudo apt install -y docker.io docker-compose-plugin make
    sudo systemctl enable --now docker
    ```

4. Verify docker can access the NVIDIA GPU.

    `docker run --rm --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi`

5. Enter the repo

    `cd ~/workspace/SAUVC-Simulation`

6. Initialize the workspace

    ```bash
    make -f Makefile_ubuntu compose_init
    ```

7. Allow X11 connections (host)

    `xhost +local:`

8. Launch Gazebo and the ROS bridge

    `make -f Makefile_ubuntu compose_launch`

## Test Thrustes

Use `ign topic -t /orca_auv/thrusters/thruster_0/force_N -m ignition.msgs.Double -p 'data: 5'`.

## Bridge to ROS2

`make compose_launch` starts `ros2 launch bringup orca_ros_gz_bridge_launch.py` automatically.

For manual bridge-only testing, enter the container with `make compose_shell` (`make -f Makefile_ubuntu compose_shell` on Ubuntu) and run `ros2 launch bringup orca_ros_gz_bridge_launch.py`.

To use a different bridge namespace, pass `namespace:=<name>` and make sure the Gazebo model publishes topics under the same namespace.

## How to Create a Model

Following [Blender SDF Exporter](https://github.com/gazebosim/gz-sim/blob/ignition-gazebo6_6.17.0/tutorials/blender_sdf_exporter.md), 
use Blender 2.82 to make the model and its texture and use 
[sdf_exporter.py](https://github.com/gazebosim/gz-sim/blob/ign-gazebo5/examples/scripts/blender/sdf_exporter.py)
to export the model to .sdf.

## Reference

[How to use web visualization](https://gazebosim.org/docs/fortress/web_visualization/)

[Simulating and Testing underwater robots in GazeboSim](https://app.theconstruct.ai/rosjects/946878/)
