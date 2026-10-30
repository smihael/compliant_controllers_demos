# Franka Gazebo

FR3 Gazebo launch and controller configuration for the compliant controllers.
The `extra_urdf_file` argument accepts an optional URDF fragment for fixtures
or end-effectors supplied by a separate demo package.

## Docker

From the `compliant_controllers_demos` repository root, build the shared
controller image once:

```bash
docker build . -t compliant-controllers:humble-desktop-full
cd franka_gz_sim
docker build -f Dockerfile -t compliant-controllers:franka-gz-sim ..
```

Start the Gazebo demo with its GUI from this directory:

```bash
xhost +si:localuser:root
SHOW_GAZEBO_GUI=true docker compose up --build
```

Gazebo starts headless by default. Use `docker compose up --build` for headless
execution, or set `SHOW_RVIZ=true` to enable RViz.

### Shared settings

`ROS_DOMAIN_ID` overrides the default of 1. `FRANKA_BASE_IMAGE` overrides the
shared image for Compose builds; for a direct build use
`--build-arg BASE_IMAGE=...`.

## Send a Cartesian step

Start the simulation and send one **5 mm step along base-frame X** as soon as its
controllers are healthy:

```bash
docker compose --profile demo up --build
```

For an already-running simulation, run the step on demand:

```bash
docker compose run --rm demo
```

Each invocation sends one step relative to the current pose and exits. Repeat
the command to send another step. The sender times out after 30 seconds if it
cannot finish. Plain `docker compose up` starts only the simulation.

Use the same `ROS_DOMAIN_ID` for the simulation and command sender.

```bash
docker compose logs demo
docker compose --profile demo down
```

## Compare compliance frames

The stiffness-frame test keeps the controller's current Cartesian reference and
sets positional stiffness to `diag(1000, 0, 1000)` N/m. Build the image once,
then select a frame when starting or recreating the simulation:

```bash
# World/base axes
COMPLIANCE_FRAME='' docker compose up --build --force-recreate franka-gazebo

# Flange axes
COMPLIANCE_FRAME=fr3_link8 docker compose up --force-recreate franka-gazebo

# TCP axes (the hand must be loaded)
LOAD_GRIPPER=true COMPLIANCE_FRAME=fr3_hand_tcp \
  docker compose up --force-recreate franka-gazebo
```

An empty `compliance_frame` uses the controller's base axes; this launch
publishes an identity `world` to base transform, so those axes match world.
After each launch, run the sender in another terminal:

```bash
docker compose exec franka-gazebo bash -lc \
  'source /opt/ros/humble/setup.bash && source /controllers_ws/install/setup.bash && ros2 run compliant_controllers test_compliance_frame.py'
```
