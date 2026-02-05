import os
import time
from typing import Dict, Optional

import numpy as np

from xrobotoolkit_teleop.common.base_hardware_teleop_controller import (
    HardwareTeleopController,
)
from xrobotoolkit_teleop.hardware.interface.franka3 import (
    DEFAULT_FRANKA3_HOME_Q,
    DEFAULT_FRANKA3_IP,
    DEFAULT_GRIPPER_CLOSED_WIDTH,
    DEFAULT_GRIPPER_OPEN_WIDTH,
    Franka3DriverBase,
    Franka3Interface,
)
from xrobotoolkit_teleop.utils.geometry import R_HEADSET_TO_WORLD
from xrobotoolkit_teleop.utils.path_utils import ASSET_PATH

DEFAULT_FRANKA3_URDF_PATH = os.path.join(ASSET_PATH, "franka3", "franka3.urdf")
DEFAULT_SCALE_FACTOR = 1.0

DEFAULT_FRANKA3_MANIPULATOR_CONFIG = {
    "right_arm": {
        "link_name": "panda_hand",
        "pose_source": "right_controller",
        "control_trigger": "right_grip",
        "gripper_config": {
            "type": "parallel",
            "gripper_trigger": "right_trigger",
            "joint_names": ["panda_finger_joint1"],
            "open_pos": [DEFAULT_GRIPPER_OPEN_WIDTH],
            "close_pos": [DEFAULT_GRIPPER_CLOSED_WIDTH],
        },
    },
}


class Franka3TeleopController(HardwareTeleopController):
    def __init__(
        self,
        robot_urdf_path: str = DEFAULT_FRANKA3_URDF_PATH,
        manipulator_config: dict = DEFAULT_FRANKA3_MANIPULATOR_CONFIG,
        robot_ip: str = DEFAULT_FRANKA3_IP,
        driver: Optional[Franka3DriverBase] = None,
        use_mock_driver: bool = False,
        q_home: np.ndarray = DEFAULT_FRANKA3_HOME_Q,
        open_width: float = DEFAULT_GRIPPER_OPEN_WIDTH,
        closed_width: float = DEFAULT_GRIPPER_CLOSED_WIDTH,
        R_headset_world: np.ndarray = R_HEADSET_TO_WORLD,
        scale_factor: float = DEFAULT_SCALE_FACTOR,
        visualize_placo: bool = False,
        control_rate_hz: int = 50,
        enable_log_data: bool = True,
        log_dir: str = "logs/franka3",
        log_freq: float = 50,
    ):
        self.robot_ip = robot_ip
        self.driver = driver
        self.use_mock_driver = use_mock_driver
        self.q_home = q_home
        self.open_width = open_width
        self.closed_width = closed_width
        super().__init__(
            robot_urdf_path=robot_urdf_path,
            manipulator_config=manipulator_config,
            R_headset_world=R_headset_world,
            floating_base=False,
            scale_factor=scale_factor,
            visualize_placo=visualize_placo,
            control_rate_hz=control_rate_hz,
            enable_log_data=enable_log_data,
            log_dir=log_dir,
            log_freq=log_freq,
            enable_camera=False,
            camera_fps=0,
        )

    def _placo_setup(self):
        super()._placo_setup()
        arm_config = self.manipulator_config["right_arm"]
        ee_link_name = arm_config["link_name"]
        self.placo_arm_joint_slice = slice(
            self.placo_robot.get_joint_offset("panda_joint1"),
            self.placo_robot.get_joint_offset("panda_joint7") + 1,
        )
        self.ee_link_name = ee_link_name

    def _robot_setup(self):
        self.arm_controller = Franka3Interface(
            robot_ip=self.robot_ip,
            dt=self.dt,
            driver=self.driver,
            use_mock=self.use_mock_driver,
            q_home=self.q_home,
            open_width=self.open_width,
            closed_width=self.closed_width,
        )
        self.arm_controller.connect()
        self.arm_controller.go_home()
        time.sleep(1.0)

    def _initialize_camera(self):
        self.camera_interface = None

    def _update_robot_state(self):
        self.placo_robot.state.q[self.placo_arm_joint_slice] = self.arm_controller.get_joint_positions()

    def _send_command(self):
        if self.active.get("right_arm", False):
            q_des = self.placo_robot.state.q[self.placo_arm_joint_slice].copy()
            self.arm_controller.set_joint_positions(q_des)

        gripper_config = self.manipulator_config["right_arm"].get("gripper_config")
        if gripper_config:
            joint_name = gripper_config["joint_names"][0]
            gripper_target = float(self.gripper_pos_target["right_arm"][joint_name])
            self.arm_controller.set_gripper_width(gripper_target)

    def _get_robot_state_for_logging(self) -> Dict:
        return {
            "qpos": self.arm_controller.get_joint_positions(),
            "qvel": self.arm_controller.get_joint_velocities(),
            "qpos_des": self.placo_robot.state.q[self.placo_arm_joint_slice].copy(),
            "gripper_target": self.gripper_pos_target["right_arm"].copy(),
        }

    def _shutdown_robot(self):
        self.arm_controller.go_home()
        time.sleep(1.0)
        self.arm_controller.close()

    def _get_camera_frame_for_logging(self) -> Dict:
        return {}
