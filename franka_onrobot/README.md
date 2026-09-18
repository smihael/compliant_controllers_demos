# Franka FR3 with OnRobot HEX

Real FR3 with an OnRobot HEX force/torque sensor, built on
`compliant-controllers:franka-real`. Build the [base image](../franka_real/) first.

The image uses the [OnRobot UDP driver](https://github.com/smihael/onrobot_ft_ros2)
and the sensor description from [ros2_net_ft_driver](https://github.com/gbartyzel/ros2_net_ft_driver).

## Prepare and build

From this directory:

```bash
cp .env.example .env
```

Set `ROBOT_IP`, `USER_UID`, `USER_GID`, `FRANKA_CPUSET`, and `FRANKA_MEMLOCK`
for the target host, as described in [franka_real](../franka_real/#configure-locally).

Set `ONROBOT_IP` to the sensor address. Optional settings are `ONROBOT_PORT`,
`ONROBOT_SPEED`, `ONROBOT_FILTER`, and `ONROBOT_SAMPLES_PER_REQUEST`.

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
