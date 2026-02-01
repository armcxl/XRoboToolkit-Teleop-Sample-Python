import os

import tyro

from xrobotoolkit_teleop.simulation.mujoco_teleop_controller import MujocoTeleopController


def _default_fr3_path(*parts: str) -> str:
    return os.path.expanduser(os.path.join("~/cxl/franka_description", *parts))


def main(
    xml_path: str = _default_fr3_path("mujoco_menagerie/franka_fr3/fr3.xml"),
    robot_urdf_path: str = _default_fr3_path("mujoco_menagerie/franka_fr3/fr3.urdf"),
    link_name: str = "fr3_hand",
    vis_target: str = "target",
    scale_factor: float = 1.2,
    visualize_placo: bool = True,
):
    """
    Run the FR3 teleoperation demo in MuJoCo using the Menagerie assets.

    Override link_name/vis_target if your FR3 MJCF uses different body names.
    """
    config = {
        "right_hand": {
            "link_name": link_name,
            "pose_source": "right_controller",
            "control_trigger": "right_grip",
            "vis_target": vis_target,
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
