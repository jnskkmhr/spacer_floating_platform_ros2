# SpaceR Floating Platform ROS2 package

We have following packages:
- `rans_ros2`: ROS2 wrapper for RL control
- `floating_platform_control`: ROS2 package to deal with FP hardware

## docker container
We support docker to run programs in ros2 humble environment.

Before you start, make sure your default docker runtime is nvidia.
Open `/etc/docker/daemon.json` and edit it like the following.

```json
{
    "runtimes": {
        "nvidia": {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    },

    "default-runtime": "nvidia"
}
```
Then, restart docker daemon by 
```bash
sudo docker systemctl restart docker
```

Then, build docker image and run container.
```bash
cd {/path/to/this/package}
./docker/build_docker.sh jetson # jetson
./docker/build_docker.sh x86 # x86 computer
./docker/run_docker.sh
```

**NOTE** that docker support on jetson is under development.

## Run node
See README of each packages.