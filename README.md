# compliant_controllers_demos

Collection of ROS 2 demo packages for `compliant_controllers`.

## Packages

- [`fr3_real`](fr3_real/): real FR3 Docker image and Compose setup with configurable FCI address, CPU affinity, and real-time limits.
- [`fr3_onrobot`](fr3_onrobot/): real FR3 with OnRobot HEX force/torque sensor.
- [`fr3_robotiq`](fr3_robotiq/): real FR3 with Robotiq 2F-85 serial gripper.
- [`fr3_onrobot_robotiq`](fr3_onrobot_robotiq/): combined sensor and gripper.
- [`fr3_kwr75b_d405`](fr3_kwr75b_d405/): real FR3 with a KWR75B force/torque sensor, Franka Hand and D405 holder.
- [`fr3_gz_sim`](fr3_gz_sim/): FR3 Gazebo launch and controller setup.
- [`fr3_mujoco_sim`](fr3_mujoco_sim/): FR3 MuJoCo Docker demo.
- [`lbr_mujoco_sim`](lbr_mujoco_sim/): iiwa14 MuJoCo Docker demo.
- [`ur_mujoco_sim`](ur_mujoco_sim/): UR5e MuJoCo Docker demo.
- [`flexiv_gz_sim`](flexiv_gz_sim/): Rizon 4s Gazebo Fortress Docker demo.
- [`flexiv_mujoco_sim`](flexiv_mujoco_sim/): Rizon 4s MuJoCo Docker demo.
- [`ur_gz_sim`](ur_gz_sim/): UR Gazebo simulation using the binary UR simulation package.
- [`lbr_gz_sim`](lbr_gz_sim/): LBR Gazebo demo based on the LBR FRI ROS 2 stack (FRI client 1.15).

Default ROS domain IDs are 1 for Franka Gazebo and the surface-following example, 2 for LBR Gazebo, 3 for UR Gazebo, 4 for Franka MuJoCo, 5 for LBR MuJoCo, 6 for UR MuJoCo, 7 for Flexiv Gazebo, and 8 for Flexiv MuJoCo. Real Franka hardware uses domain 11. Override these defaults with `ROS_DOMAIN_ID`.

## Use case examples

- [`surface_following`](examples/surface_following/): curved-fixture following simulation with a Jupyter notebook for interactive control.
- [`spacemouse_teleop`](examples/spacemouse_teleop/): SpaceMouse teleoperation of the Cartesian controller with a Franka simulation Docker Compose setup.

## Build notes

The simulation packages install helpers from [`common/`](common/). During CMake configuration, demo packages
need to find a sibling `common/` directory.

Native build example:

```bash
colcon build --packages-select fr3_gz_sim
```

## Docker quickstart

Build the shared controller image once from the `compliant_controllers_demos` directory:

```bash
docker build -t compliant-controllers:humble-desktop-full .
```

The build imports Git repositories from [`controllers.repos`](controllers.repos). The image retains the ROS Humble desktop/GUI dependencies used by the demos.
Build dependent images again after rebuilding it.

From any simulation demo directory:

```bash
docker compose up --build
```

For a one-shot Cartesian command smoke test:

```bash
docker compose --profile demo up
```

Once the simulation is healthy, the `demo` service sends a single command to
move the end effector 5 mm along the robot base frame's positive X axis
from its current position, keeping the target Y, Z, and orientation unchanged.
The arm should make a small movement and then hold the new target pose.