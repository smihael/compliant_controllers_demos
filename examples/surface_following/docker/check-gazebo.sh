#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
test -f /tmp/surface-ready
python3 /opt/demo/docker/check-controllers.py
timeout 3 ros2 topic echo --no-daemon --once /joint_states sensor_msgs/msg/JointState >/dev/null
