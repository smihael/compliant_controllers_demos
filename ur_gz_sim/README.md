# UR Gazebo simulation

Standalone UR simulation with the compliant Cartesian impedance controller.
The default robot is `ur5e`; set `UR_TYPE=ur10e` (or another supported UR model)
to choose a different robot.

## Docker

From the `compliant_controllers_demos` repository root, build the shared controller image once:

```bash
docker build . -t compliant-controllers:humble-desktop-full
cd ur_gz_sim
docker build . -t compliant-controllers:ur-gz-sim
```

Start Gazebo with its GUI from this directory:

```bash
xhost +si:localuser:root
SHOW_GAZEBO_GUI=true docker compose up --build
```

For headless operation or a different model:

```bash
SHOW_GAZEBO_GUI=false docker compose up
UR_TYPE=ur10e docker compose up
```

`SHOW_GAZEBO_GUI` and `SHOW_RVIZ` default to `false` in Compose.
GUI forwarding uses the host X11 display.
`ROS_DOMAIN_ID` defaults to 3. Use the same ROS domain for the simulation and
command sender. `UR_BASE_IMAGE` overrides the shared image in Compose; a direct
Docker build accepts `--build-arg BASE_IMAGE=...`.

## Send a Cartesian step

Start the simulation and an optional command sender:

```bash
docker compose --profile demo up --build
```

The `demo` container reuses the simulation image, waits for healthy controllers,
then sources the workspace and runs the installed script directly:
`ros2 run compliant_controllers test_cartesian_command.py`.
It sends one 5 mm X command relative to the current pose, using `base_link` and
`tool0`, and exits. The command times out after 30 seconds if it cannot finish.

For an already-running simulation, send another step with:

```bash
docker compose run --rm demo
```

Plain `docker compose up` starts only the simulation. Keep individual offsets
below the controller's 10 mm max-step guard.

```bash
docker compose ps
docker compose logs demo
docker compose --profile demo down
```

The simulation health check requires the joint-state broadcaster and Cartesian
impedance controller to be active, and joint-state messages to be available.

## Native ROS launch

With the controller workspace and ROS Humble sourced:

```bash
sudo apt install ros-humble-ur-simulation-gz
colcon build --packages-select ur_gz_sim
source install/setup.bash
ros2 launch ur_gz_sim ur_gz.launch.py ur_type:=ur5e gazebo_gui:=true launch_rviz:=false
```

## Known issue

Some installations show idle joint oscillation that is not detected by the
standard health check.
