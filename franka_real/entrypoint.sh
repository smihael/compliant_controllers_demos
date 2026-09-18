#!/bin/bash
set -e
source /opt/ros/"${ROS_DISTRO:-humble}"/setup.bash
source /controllers_ws/install/setup.bash
exec "$@"
