# Franka FR3 in MuJoCo

A separate counterpart to [`franka_gz_sim`](../franka_gz_sim/), using
[ros-controls/mujoco_ros2_control](https://github.com/ros-controls/mujoco_ros2_control)
and the shared `compliant_controllers` Cartesian impedance controller.
The default is an FR3 arm **without a gripper**, gravity enabled, seven effort
interfaces, and a 1 ms physics timestep. The ROS frames remain `fr3_link0` through
`fr3_link8`. No Franka hardware driver or libfranka is needed by this demo.

## Build and run with Docker

Requires Docker Engine and Docker Compose v2. From the `compliant_controllers_demos` repository root:

```bash
# Skip this if you already built the shared controller image.
docker build -t compliant-controllers:humble-desktop-full .
cd franka_mujoco_sim
docker compose up --build -d --wait
```

Default startup is headless. Set `ROS_DOMAIN_ID` if needed (default: `4`).
Set `FRANKA_BASE_IMAGE` to override the shared base image.

## Basic commands

```bash
docker compose ps
docker compose logs --no-color -f franka-mujoco
docker compose run --rm --no-deps demo
docker compose run --rm --no-deps smoke-test
docker compose down
```

`demo` sends one 5 mm Cartesian X step. `smoke-test` verifies basic motion and
returns nonzero on failure.

One-shot test run:

```bash
docker compose --profile test up --build --abort-on-container-exit --exit-code-from smoke-test
```

## Optional X11 viewer

With a local X11 server or XWayland and `DISPLAY` set:

```bash
xhost +si:localuser:root
docker compose -f docker-compose.yml -f docker-compose.x11.yml up --build -d --wait
docker compose run --rm --no-deps demo
```

Stop the viewer and revoke X access:

```bash
docker compose down
xhost -si:localuser:root
```

## Configuration

Controller settings are in [`config/fr3_controllers.yaml`](config/fr3_controllers.yaml).
The default target is `cartesian_impedance_controller`.

Inspect ROS from the running container (the entrypoint sources both overlays):

```bash
docker compose exec franka-mujoco /franka-mujoco-entrypoint.sh ros2 control list_controllers
docker compose exec franka-mujoco /franka-mujoco-entrypoint.sh ros2 topic echo --once /joint_states
```

## Native launch (alternative)

If dependencies are installed natively, generate the model and launch:

```bash
python scripts/generate_model.py --output /tmp/fr3-mujoco-model
ros2 launch franka_mujoco_sim fr3_mujoco.launch.py model_dir:=/tmp/fr3-mujoco-model headless:=false
```
