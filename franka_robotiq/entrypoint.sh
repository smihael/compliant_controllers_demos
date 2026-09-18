#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/${ROS_DISTRO}/setup.bash
source /controllers_ws/install/setup.bash
source /robotiq_ws/install/setup.bash
if [[ "${1:-}" == bringup ]]; then
  shift
  args=(robot_ip:="${ROBOT_IP:?Set ROBOT_IP to the FCI address}"
        controller_name:="${CONTROLLER_NAME:-cartesian_impedance_controller}"
        ee_frame:="${EE_FRAME:-fr3_link8}")
  args+=(start_robotiq:="${START_ROBOTIQ:-true}"
         com_port:="${ROBOTIQ_COM_PORT:-/dev/ttyUSB0}")
  exec ros2 launch franka_robotiq fr3_robotiq.launch.py "${args[@]}" "$@"
fi
exec "$@"
