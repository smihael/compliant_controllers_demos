#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /controllers_ws/install/compliant_controllers_msgs/share/compliant_controllers_msgs/local_setup.bash
source /teleop_ws/install/local_setup.bash
exec "$@"
