# SpaceR Floating Platform ROS2 package

## 1. Build (Only native install)
First install optitrack motion capture package via apt
```bash
sudo apt-get install ros-$ROS_DISTRO-vrpn-mocap
```

Then install dependencies via rosdep and build package.

```bash
cd {/path/to/ros2_ws}
rosdep init && rosdep install --from-paths src -yi
colcon build --symlink-install
```

## 2. Launch node

<details><summary><b>floating platform</b></summary>

source workspace
```bash
source install/setup.bash
```

run command (jetson orin nano)
```bash
ros2 launch floating_platform_control fp_valve_control.launch.py
```

run command (rasberry Pi4)
```bash
ros2 launch floating_platform_control fp_valve_control_rpi.launch.py
```
</details>


<details><summary><b>opti-track</b></summary>

source workspace
```bash
source install/setup.bash
```

run command
```bash
ros2 launch floating_platform_control optitrack.launch.py
```
</details>