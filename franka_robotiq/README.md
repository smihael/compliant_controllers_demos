# Franka FR3 with Robotiq 2F-85

Real FR3 with a Robotiq 2F-85 serial gripper, built on
`compliant-controllers:franka-real`. Build the [base image](../franka_real/) first.

The image uses the Humble [PickNik Robotiq stack](https://github.com/PickNikRobotics/ros2_robotiq_gripper/tree/humble).

## Prepare and build

From this directory:

```bash
cp .env.example .env
```

Set `ROBOT_IP`, `USER_UID`, `USER_GID`, `FRANKA_CPUSET`, and `FRANKA_MEMLOCK`
for the target host, as described in [franka_real](../franka_real/#configure-locally).

Set `ROBOTIQ_COM_PORT` to the host serial device and `ROBOTIQ_DEVICE_GID` to
its group ID (`stat -c %g /dev/ttyUSB0`, substituting your device path).

```bash
docker compose build
```

## Usage

After reviewing `.env` and the tool configuration, enable FCI and start:

```bash
docker compose up
```

This activates the real arm controller and accessory drivers. Stop with
Ctrl-C followed by `docker compose down`.
