# Franka FR3 with OnRobot HEX and Robotiq 2F-85

Real FR3 with an OnRobot HEX sensor and Robotiq 2F-85 serial gripper, built on
`compliant-controllers:franka-real`. Build the [base image](../franka_real/) first.

## Prepare and build

From this directory:

```bash
cp .env.example .env
```

Set `ROBOT_IP`, `USER_UID`, `USER_GID`, `FRANKA_CPUSET`, and `FRANKA_MEMLOCK`
for the target host, as described in [franka_real](../franka_real/#configure-locally).

Configure the sensor using the [OnRobot settings](../franka_onrobot/#prepare-and-build)
and the gripper using the [Robotiq settings](../franka_robotiq/#prepare-and-build)
in this directory's `.env` file.

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
