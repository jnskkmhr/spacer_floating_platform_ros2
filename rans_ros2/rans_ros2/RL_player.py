#!/usr/bin/env python3
# coding: utf-8

__author__ = "Antoine Richard, Matteo El Hariry"
__copyright__ = (
    "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
)
__license__ = "GPL"
__version__ = "2.1.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

# std lib
from utils.reformat import omegaconf_to_dict, print_dict
from utils.hydra_utils import *
from omegaconf import DictConfig, OmegaConf
import hydra
import os

# rclpy
import rclpy

# custom lib
from mujoco_envs.controllers.discrete_LQR_controller import (
    DiscreteController,
    parseControllerConfig,
)
from mujoco_envs.controllers.RL_games_model_4_mujoco import (
    RLGamesModel,
)
from mujoco_envs.environments.mujoco_base_env import (
    MuJoCoFloatingPlatform,
    parseEnvironmentConfig,
)
from mujoco_envs.controllers.hl_controllers import hlControllerFactory
from ros.ros_node import RLPlayerNode


@hydra.main(config_name="config_mujoco", config_path="cfg")
def run(cfg: DictConfig):
    """ "
    Run the simulation.

    Args:
        cfg (DictConfig): A dictionary containing the configuration of the simulation.
    """
    
    cfg_dict = omegaconf_to_dict(cfg)

    # Create the environment
    env = MuJoCoFloatingPlatform(**parseEnvironmentConfig(cfg_dict))

    # Get the low-level controller
    if cfg_dict["use_rl"]:
        assert os.path.exists(
            cfg_dict["checkpoint"]
        ), "A correct path to a neural network must be provided to infer an RL agent."
        ll_controller = RLGamesModel(
            config=cfg_dict["train"], model_path=cfg_dict["checkpoint"]
        )
    else:
        ll_controller = DiscreteController(**parseControllerConfig(cfg_dict, env))

    dt = cfg_dict["task"]["sim"]["dt"]
    # Get the high-level controller
    hl_controller = hlControllerFactory(cfg_dict, ll_controller, dt)
    
    # create ros node.
    rclpy.init()
    node = RLPlayerNode(
        hl_controller,
        cfg=cfg_dict,
        debug=True,
    )
    
    # Run the node.
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()
    hl_controller.saveSimulationData()
    hl_controller.plotSimulation()

if __name__ == "__main__":
    # Initialize ROS node
    run()