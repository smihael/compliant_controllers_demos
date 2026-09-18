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
