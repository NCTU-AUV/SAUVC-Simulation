# Orca AUV Gazebo Simulation

## How to Start

### for macOS

1. Run `make compose_build` to build the Docker image (first time or after Dockerfile changes).
2. Run `make compose_up` to start the container (generates TLS certs via mkcert if needed).
3. Run `make compose_init` to install dependencies and build the workspace inside the container.
4. Run `make compose_shell`, then inside the container run `ign launch -v 4 orca_auv_gazebo_simulation_ws/websocket.ign`.
5. In another terminal, run `make compose_shell` again and then `ign gazebo -v 4 -s -r <world_file.sdf>`.
6. Go to [https://app.gazebosim.org/visualization](https://app.gazebosim.org/visualization) and connect to [wss://localhost:9002](wss://localhost:9002) (Safari confirmed to work).

> Note: `mkcert` must be installed locally for certificate generation.

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

6. Replace docker-compose.yml to below

    ```bash
    services:
    orca:
        container_name: ${CONTAINER_NAME:-orca-auv-gazebo-simulation-container}
        image: ${IMAGE_NAME:-orca-auv-gazebo-simulation-image}:latest
        build:
        context: .
        dockerfile: Dockerfile
        stdin_open: true
        tty: true

        gpus: all

        environment:
        - DISPLAY=${DISPLAY}
        - QT_X11_NO_MITSHM=1
        - NVIDIA_VISIBLE_DEVICES=all
        - NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute,display
        - __GLX_VENDOR_LIBRARY_NAME=nvidia

        ports:
        - "9002:9002"

        volumes:
        - ./orca_auv_gazebo_simulation_ws:/root/orca_auv_gazebo_simulation_ws
        - ./certs:/ign-certs:ro
        - /tmp/.X11-unix:/tmp/.X11-unix:rw

        devices:
        - /dev/dri:/dev/dri

        working_dir: /root
        command: ["/bin/bash", "-lc", "tail -f /dev/null"]
    ```

8. Rebuild + start the container and initialize the workspace

    ```bash
    make -f Makefile_ubuntu compose_clean
    make -f Makefile_ubuntu compose_build
    make -f Makefile_ubuntu compose_up
    make -f Makefile_ubuntu compose_init
    ```

9. Allow X11 connections (host)

    `xhost +local:`

10. Enter the container

    `make compose_shell`

11. Launch Gazebo GUI with GPU acceleration

    `ign gazebo -v 4 /root/orca_auv_gazebo_simulation_ws/src/orca_sim_bringup/worlds/water_world.sdf`

## Test Thrustes

Use `ign topic -t /orca_auv/thruster_0/set_output_force_N -m ignition.msgs.Double -p 'data: 5'`.

## Bridge to ROS2

Run `ros2 launch orca_sim_bringup orca_ros_gz_bridge_launch.py`.

## How to Create a Model

Following [Blender SDF Exporter](https://github.com/gazebosim/gz-sim/blob/ignition-gazebo6_6.17.0/tutorials/blender_sdf_exporter.md), 
use Blender 2.82 to make the model and its texture and use 
[sdf_exporter.py](https://github.com/gazebosim/gz-sim/blob/ign-gazebo5/examples/scripts/blender/sdf_exporter.py)
to export the model to .sdf.

## Reference

[How to use web visualization](https://gazebosim.org/docs/fortress/web_visualization/)

[Simulating and Testing underwater robots in GazeboSim](https://app.theconstruct.ai/rosjects/946878/)
