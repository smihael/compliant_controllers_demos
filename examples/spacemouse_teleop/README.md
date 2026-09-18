# SpaceMouse teleoperation

SpaceMouse interface for the Cartesian impedance controller.

## Franka simulation with Docker

The simulation reuses the existing `compliant-controllers:franka-gz-sim`
image. Build it first using the [Franka Gazebo instructions](../../franka_gz_sim/#docker).
Compose builds the teleoperation image.

First, identify the device (`/dev/hidraw3` is used below as an example).

```bash
grep -H SpaceMouse /sys/class/hidraw/hidraw*/device/uevent
```

From this directory, with the SpaceMouse connected:

```bash
export SPACEMOUSE_DEVICE=/dev/hidraw3

# Allow the container to display Gazebo on your local X11/XWayland desktop.
xhost +si:localuser:root
docker compose up --build -d
docker compose logs -f teleop
```

Move or twist the SpaceMouse cap to move the simulated end effector. Releasing
it holds the current target. Stop with `docker compose down`, then revoke
X11 access with `xhost -si:localuser:root`.

## Native ROS workspace

Requires `compliant_controllers_msgs` and `spacemouse_publisher` from
[franka_spacemouse](https://github.com/frankarobotics/franka_spacemouse),
with its device setup completed.

Build from your sourced ROS workspace:

```bash
colcon build --symlink-install --packages-select spacemouse_teleop
source install/setup.bash
```

Launch the robot or simulation independently, then start teleop with the same
ROS domain and robot namespace:

```bash
ros2 launch spacemouse_teleop fr3_spacemouse_teleop.launch.py
```

List available options with `--show-args`.
