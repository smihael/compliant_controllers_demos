#!/usr/bin/env bash
set -eo pipefail
fetch() {
  git init -q "$3"
  git -C "$3" remote add origin "$1"
  git -C "$3" fetch --depth 1 origin "$2"
  git -C "$3" checkout --detach FETCH_HEAD
}
mkdir -p src
fetch https://github.com/smihael/onrobot_ft_ros2.git "$ONROBOT_VERSION" src/onrobot_ft_ros2
fetch https://github.com/gbartyzel/ros2_net_ft_driver.git "$NET_FT_VERSION" /tmp/net_ft
cp -a /tmp/net_ft/net_ft_description src/
rm -rf /tmp/net_ft
source /controllers_ws/install/setup.bash
apt-get update
rosdep install --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
rm -rf /var/lib/apt/lists/*
