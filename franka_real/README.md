# Real Franka FR3

Standalone ROS 2 Humble image for a real FR3, using the stock Franka description
and the Cartesian impedance controller (or optional joint impedance controller).
The robot and controller manager use namespace `/fr3`.

The image extends `compliant-controllers:humble-desktop-full` from the repository root Dockerfile.
It pins `franka_ros2` v2.4.0, libfranka 0.20.5, and
`franka_description` 2.7.0. The libfranka binary targets Ubuntu Jammy amd64.
Check that this stack matches the robot's installed system version before use.
The launch uses the upstream
[v2.4.0 bringup interface](https://github.com/frankarobotics/franka_ros2/blob/v2.4.0/franka_bringup/launch/franka.launch.py).

## Configure locally

From this directory:

```bash
cp .env.example .env
```

Fill in `ROBOT_IP` locally with the FCI address.

Set `USER_UID` and `USER_GID` to your host IDs (`id -u` / `id -g`).

Adjust `FRANKA_CPUSET` to CPUs present on the control host.

The default ROS domain is 11. Set `ROS_DOMAIN_ID` to match your other ROS processes.

The host must provide real-time scheduling. Set `FRANKA_MEMLOCK` for the
control host; the image does not configure the host kernel.

## Validate or build without starting

After filling in `.env`, this only validates the configuration:

```bash
docker compose config --quiet
```

If the base image is missing, build it from the `compliant_controllers_demos` directory:

```bash
docker build -t compliant-controllers:humble-desktop-full -f Dockerfile .
```

Then, from this demo directory, build the hardware image without starting it:

```bash
docker compose build
```

## Usage

When ready to activate the real robot's controller, with FCI enabled:

```bash
docker compose up
```

This connects to real hardware and activates the selected impedance controller.
There is no motion demo service or automatic restart. Stop with Ctrl-C or
`docker compose down`.

`LOAD_GRIPPER=true` enables the stock Franka hand. Set
`CONTROLLER_NAME=joint_impedance_controller` to select joint impedance.

## Compare compliance frames

The compliance-frame sender sets positional stiffness to
`diag(1000, 0, 1000)` N/m without changing the current Cartesian reference.
Only run it when the robot is in a safe, supported configuration and the
resulting anisotropic compliance is appropriate for the workcell.

Select one frame at launch, then run the sender in a second terminal.
Stop and restart the launch before changing frames. From `~/local_ws`:

```bash
source install/setup.bash

# World axes (world and fr3_link0 coincide in this demo)
CONTROLLER_DEBUG=1 ros2 launch franka_real fr3.launch.py robot_ip:=192.168.1.1 compliance_frame:=world

# Flange axes
CONTROLLER_DEBUG=1 ros2 launch franka_real fr3.launch.py robot_ip:=192.168.1.1 compliance_frame:=fr3_link8

# Hand TCP axes and compliance point; requires the stock hand description
CONTROLLER_DEBUG=1 ros2 launch franka_real fr3.launch.py robot_ip:=192.168.1.1 load_gripper:=true compliance_frame:=fr3_hand_tcp
```

Sender (same command for each frame):

```bash
source ~/local_ws/install/setup.bash
ros2 run compliant_controllers test_compliance_frame.py
# Optional gains and namespace:
ros2 run compliant_controllers test_compliance_frame.py --ros-args -r __ns:=/fr3 -p stiffness_x:=1000.0 -p stiffness_y:=0.0 -p stiffness_z:=1000.0
```

The sender waits for a subscriber on `/fr3/stiffness_pos` and publishes once.
It leaves the pose target captured at controller activation and orientation
stiffness unchanged. Start each case at the desired pose before activation.
The Y axis of the selected frame has zero positional stiffness; it will not
hold position along that direction. Other axes resist displacement. Flange
and TCP axes follow the tool orientation; world axes remain fixed. The TCP
case also moves the compliance point to the TCP, so rotation and translation
can couple. Choose a tool orientation whose Y axis differs from world Y to
make the difference visible. Ctrl-C stops the sender but does not restore gains.
An arbitrary `compliance_frame` must name a URDF link/joint rigidly attached to
`control_frame`; it cannot name another moving robot link.

For Docker, use `COMPLIANCE_FRAME=fr3_link8 docker compose up` (or empty for
world axes, or `LOAD_GRIPPER=true COMPLIANCE_FRAME=fr3_hand_tcp`). Run the sender:

```bash
docker compose exec controllers bash -lc \
  'source /opt/ros/humble/setup.bash && source /controllers_ws/install/setup.bash && ros2 run compliant_controllers test_compliance_frame.py'
```
