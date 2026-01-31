import tyro

from xrobotoolkit_teleop.hardware.franka3_teleop_controller import (
    DEFAULT_FRANKA3_MANIPULATOR_CONFIG,
    DEFAULT_FRANKA3_URDF_PATH,
    Franka3TeleopController,
)
from xrobotoolkit_teleop.hardware.interface.franka3 import (
    DEFAULT_FRANKA3_HOME_Q,
    DEFAULT_FRANKA3_IP,
)


def main(
    robot_urdf_path: str = DEFAULT_FRANKA3_URDF_PATH,
    robot_ip: str = DEFAULT_FRANKA3_IP,
    scale_factor: float = 1.0,
    visualize_placo: bool = False,
    control_rate_hz: int = 50,
    enable_log_data: bool = True,
    log_dir: str = "logs/franka3",
    use_mock_driver: bool = True,
):
    """
    Main function to run the Franka3 teleoperation.
    """
    controller = Franka3TeleopController(
        robot_urdf_path=robot_urdf_path,
        manipulator_config=DEFAULT_FRANKA3_MANIPULATOR_CONFIG,
        robot_ip=robot_ip,
        scale_factor=scale_factor,
        visualize_placo=visualize_placo,
        control_rate_hz=control_rate_hz,
        enable_log_data=enable_log_data,
        log_dir=log_dir,
        use_mock_driver=use_mock_driver,
        q_home=DEFAULT_FRANKA3_HOME_Q,
    )
    controller.run()


if __name__ == "__main__":
    tyro.cli(main)
