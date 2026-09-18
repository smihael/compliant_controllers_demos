#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/${ROS_DISTRO}/setup.bash
source /controllers_ws/install/setup.bash
source /onrobot_ws/install/setup.bash
if [[ "${1:-}" == bringup ]]; then
  shift
  args=(robot_ip:="${ROBOT_IP:?Set ROBOT_IP to the FCI address}"
        controller_name:="${CONTROLLER_NAME:-cartesian_impedance_controller}"
        ee_frame:="${EE_FRAME:-fr3_link8}")
  args+=(onrobot_ip_address:="${ONROBOT_IP:?Set ONROBOT_IP to the sensor address}"
         onrobot_port:="${ONROBOT_PORT:-49152}"
         onrobot_speed:="${ONROBOT_SPEED:-10}"
         onrobot_filter:="${ONROBOT_FILTER:-4}"
         onrobot_samples_per_request:="${ONROBOT_SAMPLES_PER_REQUEST:-10}"
         onrobot_bias_on_start:="${ONROBOT_BIAS_ON_START:-false}")
  exec ros2 launch franka_onrobot fr3_onrobot.launch.py "${args[@]}" "$@"
fi
exec "$@"
