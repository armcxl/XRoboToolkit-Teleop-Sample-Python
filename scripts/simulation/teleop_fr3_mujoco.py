import os
import warnings

import mujoco
import tyro

from xrobotoolkit_teleop.simulation.mujoco_teleop_controller import MujocoTeleopController


def _default_fr3_path(*parts: str) -> str:
    return os.path.expanduser(os.path.join("~/cxl/franka_description", *parts))


def _resolve_vis_target(xml_path: str, vis_target: str) -> str:
    model = mujoco.MjModel.from_xml_path(xml_path)
    mocap_bodies = []
    for body_id in range(model.nbody):
        if model.body_mocapid[body_id] != -1:
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
            if name:
                mocap_bodies.append(name)

    if vis_target:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, vis_target)
        if body_id != -1 and model.body_mocapid[body_id] != -1:
            return vis_target

        if not mocap_bodies:
            raise ValueError(
                "No mocap body found in the FR3 MJCF. "
                "Please provide a valid --vis-target that is configured for mocap."
            )
        if len(mocap_bodies) == 1:
            resolved = mocap_bodies[0]
            warnings.warn(
                f"Vis target '{vis_target}' not found; falling back to '{resolved}'.",
                stacklevel=2,
            )
            return resolved
        raise ValueError(
            "Vis target not found in the MJCF. Available mocap bodies: "
            f"{', '.join(mocap_bodies)}. "
            "Please pass one via --vis-target."
        )

    if not mocap_bodies:
        raise ValueError(
            "No mocap body found in the FR3 MJCF. "
            "Please provide a valid --vis-target that is configured for mocap."
        )
    if len(mocap_bodies) == 1:
        return mocap_bodies[0]

    raise ValueError(
        "Multiple mocap bodies found in the MJCF. Available: "
        f"{', '.join(mocap_bodies)}. "
        "Please pass one via --vis-target."
    )


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
    vis_target = _resolve_vis_target(xml_path, vis_target)
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
