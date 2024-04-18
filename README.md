# SpaceR Floating Platform ROS2 package

We have following packages:
- `rans_ros2`: ROS2 wrapper for RL control
- `floating_platform_control`: ROS2 package to deal with FP hardware

## docker container
We support docker to run ros nodes.

```bash
cd {/path/to/this/package}
./docker/build_docker.sh
./docker/run_docker.sh
```

## Run node
See README of each packages.