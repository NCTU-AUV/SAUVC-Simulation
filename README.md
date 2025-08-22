# SAUVC-Gazebo-Simulation

## How to Start

1. Run `make build_image`
2. Run `make`
3. Run `ign launch -v 4 websocket.ign`
4. To another terminal run `make enter_container`, and run `ign gazebo -v 4 -s -r <world_file.sdf>`.
5. To [https://app.gazebosim.org/visualization](https://app.gazebosim.org/visualization) and connect to [wss://localhost:9002](wss://localhost:9002) (at least I know Safari works).

## Reference

[How to use web visualization](https://gazebosim.org/docs/fortress/web_visualization/)

[Simulating and Testing underwater robots in GazeboSim](https://app.theconstruct.ai/rosjects/946878/)
