# KUKA LBR iiwa14 in MuJoCo

Counterpart to [`lbr_gz_sim`](../lbr_gz_sim/), following the
[`franka_mujoco_sim`](../franka_mujoco_sim/) setup. Uses
[ros-controls/mujoco_ros2_control](https://github.com/ros-controls/mujoco_ros2_control)
with the shared Cartesian impedance controller, gravity compensation, 7 effort
interfaces, full visual meshes, and 1 ms physics/control periods.
This project targets **iiwa14** without a gripper.

## Build and test

Requires Docker Engine and Compose v2. From the `compliant_controllers_demos` repository root:

```bash
# Skip if already built:
docker build -t compliant-controllers:humble-desktop-full .
cd lbr_mujoco_sim
docker compose up --build -d --wait
docker compose logs --no-color -f lbr-mujoco
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

Controller settings are in [`config/iiwa14_controllers.yaml`](config/iiwa14_controllers.yaml).
This demo starts `cartesian_impedance_controller` by default.

Quick runtime checks:

```bash
docker compose exec lbr-mujoco /lbr-mujoco-entrypoint.sh ros2 control list_controllers
docker compose exec lbr-mujoco /lbr-mujoco-entrypoint.sh ros2 topic echo --once /joint_states
```

## Native launch (alternative)

If dependencies are installed natively, generate the model and launch:

```bash
python scripts/generate_model.py --output /tmp/iiwa14-mujoco-model
ros2 launch lbr_mujoco_sim iiwa14_mujoco.launch.py model_dir:=/tmp/iiwa14-mujoco-model headless:=false
```
