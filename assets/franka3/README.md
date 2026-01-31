# Franka3 assets

Place your Franka3 assets here and update script arguments if needed.

Suggested filenames for simulation and hardware:
- MuJoCo XML: `franka3_mujoco.xml`
- Placo/IK URDF: `franka3.urdf`

The URDF should include joints `panda_joint1`-`panda_joint7` and end-effector link `panda_hand`.
The MuJoCo XML should use matching names and define a mocap body (e.g. `right_target`) for
teleop target visualization.

Required names for the default teleop configuration:
- Joints: `panda_joint1`-`panda_joint7`
- End-effector link: `panda_hand`
