# Universal Robots UR5e in MuJoCo

Counterpart to [`ur_gz_sim`](../ur_gz_sim/), following the
[`franka_mujoco_sim`](../franka_mujoco_sim/) setup. Uses
[ros-controls/mujoco_ros2_control](https://github.com/ros-controls/mujoco_ros2_control)
with the shared Cartesian impedance controller, gravity compensation, 6 effort
interfaces, full visual meshes, and 1 ms physics/control periods.
This project targets **ur5e** without a gripper.

## Build and test

Requires Docker Engine and Compose v2. From the `compliant_controllers_demos` repository root:

```bash
# Skip if already built:
docker build -t compliant-controllers:humble-desktop-full .
cd ur_mujoco_sim
docker compose up --build -d --wait
docker compose logs --no-color -f ur-mujoco
```

In another terminal, from this demo directory:

```bash
# Send a 5 mm base-frame X step:
docker compose run --rm --no-deps demo
# Check pose stability, then send and measure another 5 mm step:
docker compose run --rm --no-deps smoke-test
# Stop and remove the demo containers:
docker compose down
```

`demo` sends one relative 5 mm Cartesian X step. `smoke-test` checks pose
stability and motion, returning nonzero on failure. Start the simulator with
`up --wait` before using `--no-deps`, and run with no other command senders.

For a single headless build/start/test command:

```bash
docker compose --profile test up --build --abort-on-container-exit --exit-code-from smoke-test
docker compose down
```

## X11 viewer

For X11/XWayland with `DISPLAY` set:

```bash
xhost +si:localuser:root
docker compose -f docker-compose.yml -f docker-compose.x11.yml up --build -d --wait
docker compose run --rm --no-deps demo
```

```bash
docker compose down
xhost -si:localuser:root
```

## Configuration

Controller settings are in [`config/ur5e_controllers.yaml`](config/ur5e_controllers.yaml).
This demo starts `cartesian_impedance_controller` by default.

Quick runtime checks:

```bash
docker compose exec ur-mujoco /ur-mujoco-entrypoint.sh ros2 control list_controllers
docker compose exec ur-mujoco /ur-mujoco-entrypoint.sh ros2 topic echo --once /joint_states
```

## Native launch (alternative)

If dependencies are installed natively, generate the model and launch:

```bash
python scripts/generate_model.py --output /tmp/ur5e-mujoco-model
ros2 launch ur_mujoco_sim ur5e_mujoco.launch.py model_dir:=/tmp/ur5e-mujoco-model headless:=false
```
