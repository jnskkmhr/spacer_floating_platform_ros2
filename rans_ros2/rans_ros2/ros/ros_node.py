# std lib
from typing import Callable, NamedTuple, Optional, Union, List, Dict
from collections import deque
import numpy as np
import datetime
import torch
import os

# ros2 lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
from geometry_msgs.msg import PoseStamped, Point, Pose

from ros.ros_utils import derive_velocities
from mujoco_envs.controllers.hl_controllers import (
    PoseController,
    PositionController,
    VelocityTracker,
    DockController,
)
from mujoco_envs.environments.disturbances import (
    RandomKillThrusters,
    Disturbances,
)


class RLPlayerNode(Node):
    def __init__(
        self,
        hl_controller: Union[PositionController, PoseController, VelocityTracker, DockController],
        cfg: dict,
        map: List[int] = [2, 5, 4, 7, 6, 1, 0, 3],
        debug: bool = False,
    ) -> None:
        """
        Args:
            hl_controller (Union[PositionController, PoseController, VelocityTracker]): The high-level controller.
            map (List[int], optional): The map of the thrusters. Defaults to [2, 5, 4, 7, 6, 1, 0, 3].
            platform (Dict[str, Union[bool, dict, float, str, int]], optional): The platform configuration. Defaults to None.
            disturbances (Dict[str, Union[bool, float]], optional): The disturbances. Defaults to None.
        """
        
        super().__init__('RL_player')

        platform = cfg["task"]["env"]["platform"]
        disturbances = cfg["task"]["env"]["disturbances"]
        self.play_rate = 1 / (
            cfg["task"]["env"]["controlFrequencyInv"] * cfg["task"]["sim"]["dt"]
        )

        self.run_time = cfg["task"]["env"]["maxEpisodeLength"] / self.play_rate

        self.DR = Disturbances(disturbances, platform["seed"])
        self.TK = RandomKillThrusters(
            {
                "num_thrusters_to_kill": platform["randomization"]["max_thruster_kill"]
                * platform["randomization"]["kill_thrusters"],
                "seed": platform["seed"],
            }
        )

        # Initialize variables
        self.buffer_size = 30  # Number of samples for differentiation
        self.pose_buffer = deque(maxlen=self.buffer_size)
        self.time_buffer = deque(maxlen=self.buffer_size)

        self.debug = debug
        self.map = map
        self.hl_controller = hl_controller
        self.reset()

        # Initialize Subscriber and Publisher
        # TODO: topic name should be ros parameter?
        self.pos_sub = self.create_subscription(PoseStamped, 
                                                "/vrpn_client_node/FP_exp_RL/pose", 
                                                self.pose_callback, 
                                                10)
        
        self.goal_sub = self.create_subscription(Point, 
                                                "/spacer_floating_platform/goal", 
                                                self.goal_callback, 
                                                10)
        
        self.action_pub = self.create_publisher(ByteMultiArray, 
                                                "/spacer_floating_platform/valves/input", 
                                                10)

        # Initialize ROS message for thrusters
        self.thruster_msg = ByteMultiArray()
    
    def on_shutdown(self):
        """
        Shutdown the node and kills the thrusters while leaving the air-bearing on."""
        self.thruster_msg.data = [1, 0, 0, 0, 0, 0, 0, 0, 0]
        self.action_pub.publish(self.thruster_msg)
        self.thruster_msg.data = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.action_pub.publish(self.thruster_msg)

    def getObs(self) -> Dict[str, np.ndarray]:
        """
        returns an up to date observation buffer.

        Returns:
            Dict[str, np.ndarray]: A dictionary containing the state of the simulation.
        """

        state = {}
        state["angular_velocity"] = self.DR.noisy_observations.add_noise_on_vel(
            self.ang_vel
        )
        state["linear_velocity"] = self.DR.noisy_observations.add_noise_on_vel(
            self.lin_vel
        )
        state["position"] = self.DR.noisy_observations.add_noise_on_pos(self.pos)
        state["quaternion"] = self.quat
        return state

    def reset(self) -> None:
        """
        Resets the goal and the buffers."""

        self.ready = False
        self.hl_controller.initializeLoggers()
        self.hl_controller.time = 0
        self.count = 0

    def remap_actions(self, actions: torch.Tensor) -> List[float]:
        """
        Remaps the actions from the RL algorithm to the thrusters of the platform.

        Args:
            actions (torch.Tensor): The actions from the RL algorithm.

        Returns:
            List[float]: The actions for the thrusters."""

        return [actions[i] for i in self.map]

    def pose_callback(self, msg: Pose) -> None:
        """
        Callback for the pose topic. It updates the state of the agent.

        Args:
            msg (Pose): The pose message."""
        
        current_time = msg.header.stamp

        # Add current pose and time to the buffer
        self.pose_buffer.append(msg)
        self.time_buffer.append(current_time)

        # Calculate velocities if buffer is filled
        if len(self.pose_buffer) == self.buffer_size:
            self.get_state_from_optitrack(msg)
            self.ready = True

    def get_state_from_optitrack(self, msg: Pose) -> None:
        """
        Converts a ROS message to an observation.

        Args:
            msg (Pose): The pose message."""

        pos = msg.pose.position
        quat = msg.pose.orientation
        self.pos = [pos.x, pos.y, pos.z]
        self.quat = [quat.w, quat.x, quat.y, quat.z]
        self.lin_vel, self.ang_vel = derive_velocities(
            self.time_buffer, self.pose_buffer
        )

    def goal_callback(self, msg: Point) -> None:
        """
        Callback for the goal topic. It updates the task data with the new goal data.

        Args:
            msg (Point): The goal message."""

        self.hl_controller.setGoal(np.array([msg.x, msg.y, msg.z]))

    def get_action(self, run_time: float, lifting_active: int = 1) -> None:
        """
        Gets the action from the RL algorithm and publishes it to the thrusters.

        Args:
            lifting_active (int, optional): Whether or not the lifting thruster is active. Defaults to 1.
        """
        self.state = self.getObs()
        self.action = self.hl_controller.getAction(self.state, time=run_time)
        action = self.remap_actions(self.action)
        lifting_active = 1
        action.insert(0, lifting_active)
        self.thruster_msg.data = action
        self.action_pub.publish(self.thruster_msg)

    def print_logs(self) -> None:
        """
        Prints the logs."""

        print("=========================================")
        for key, value in self.hl_controller.logs.items():
            print(f"{key}: {value[-1]}")
    
    @staticmethod
    def clock_to_sec(clock:rclpy.time.Time)->float:
        sec, nsec = clock.seconds_nanoseconds()
        return sec + nsec*10e-9
    
    def run(self) -> None:
        """
        Runs the RL algorithm."""
        self.update_once = True
        self.rate = self.create_rate(self.play_rate)
        start_time = self.clock_to_sec(self.get_clock().now())
        run_time = self.clock_to_sec(self.get_clock().now()) - start_time
        while (rclpy.ok()) and (run_time < self.run_time):
            if self.ready:
                self.get_action(run_time)
                self.count += 1
                if self.debug:
                    self.print_logs()
            run_time = self.clock_to_sec(self.get_clock().now()) - start_time
            self.rate.sleep()
