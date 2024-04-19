# RANS ROS2 package
ROS2 package for RANS

## Install dependencies & build package (only native install)

Install dependencies
```bash
cd /home/ros2_ws
rosdep update && rosdep install --from-paths src -yi
pip3 install cvxpy hydra-core omegaconf mujoco
cd ../thirdparty/rl_games && pip3 install -e .
```

Build package
```bash
cd /home/ros2_ws
colcon build --symlink-install
```

## Run ROS node

<details><summary><b>docking</b></summary>

download weight
```bash
cd /home/ros2_ws/src/spacer_floating_platform_ros2/rans_ros2
https://drive.google.com/file/d/1M_zCYP5VczzJq0TZ1VaAcsgRxW2v_Wcf/view?usp=drive_link
wget --no-check-certificate 'https://drive.google.com/uc?export=download&id=1M_zCYP5VczzJq0TZ1VaAcsgRxW2v_Wcf' -O runs.zip
unzip runs.zip && rm runs.zip
```

source workspace
```bash
cd /home/ros2_ws
source install/setup.bash
```

run command
```bash
ros2 run rans_ros2 RL_player task=MFP_eval/MFP2D_CloseProximityDock_perturbed \
train=MFP/MFP2D_PPOmulti_dict_MLP_dock hl_task=CloseProximityDock \
checkpoint=/home/ros2_ws/src/spacer_floating_platform_ros2/rans_ros2/runs/MFP2D_perturbed_v2/nn/last_MFP2D_perturbed_v2_ep_2000_rew__113.11344_.pth --ros-args --params-file /home/ros2_ws/src/spacer_floating_platform_ros2/rans_ros2/config/config.yaml
```
</details>