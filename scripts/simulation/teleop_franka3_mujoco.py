import os

import tyro

from xrobotoolkit_teleop.simulation.mujoco_teleop_controller import (
    MujocoTeleopController,
)
from xrobotoolkit_teleop.utils.path_utils import ASSET_PATH


def main(
    xml_path: str = os.path.join(ASSET_PATH, "franka3", "franka3_mujoco.xml"),
    robot_urdf_path: str = os.path.join(ASSET_PATH, "franka3", "franka3.urdf"),
    scale_factor: float = 1.0,
    visualize_placo: bool = True,
):
    """
    Main function to run Franka3 teleoperation in MuJoCo.
    """
    config = {
        "right_hand": {
            "link_name": "panda_hand",
            "pose_source": "right_controller",
            "control_trigger": "right_grip",
            "gripper_config": {
                "type": "parallel",
                "gripper_trigger": "right_trigger",
                "joint_names": ["panda_finger_joint1"],
                "open_pos": [0.08],
                "close_pos": [0.0],
            },
            "vis_target": "right_target",
        },
    }

    controller = MujocoTeleopController(
        xml_path=xml_path,
        robot_urdf_path=robot_urdf_path,
        manipulator_config=config,
        scale_factor=scale_factor,
        visualize_placo=visualize_placo,
    )

    joints_task = controller.solver.add_joints_task()
    joints_task.set_joints({joint: 0.0 for joint in controller.placo_robot.joint_names()})
    joints_task.configure("joints_regularization", "soft", 1e-4)

    controller.run()


if __name__ == "__main__":
    tyro.cli(main)
