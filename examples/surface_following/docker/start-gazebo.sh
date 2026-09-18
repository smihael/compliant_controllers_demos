#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /controllers_ws/install/setup.bash
source /demo_overlay/install/setup.bash

# The example is mounted at one stable path; only Franka packages are built.
rm -f /tmp/surface-ready
children=()
cleanup() {
  trap - EXIT
  kill -INT "${children[@]}" 2>/dev/null || true
  wait "${children[@]}" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 143' TERM INT
world="$(ros2 pkg prefix --share franka_gazebo_bringup)/worlds/empty_no_gravity.sdf"
ros2 launch franka_gz_sim fr3_gz.launch.py \
  world:="$world" show_gazebo_gui:="${SHOW_GAZEBO_GUI:-true}" \
  show_rviz:=false load_gripper:=false \
  extra_urdf_file:=/opt/demo/urdf/surface_probe.urdf \
  gravity_compensation_enabled:=false max_step_guard_enabled:=false &
launch_pid=$!
children+=("$launch_pid")

ready=false
for attempt in $(seq 1 60); do
  kill -0 "$launch_pid" || exit 1
  if python3 /opt/demo/docker/check-controllers.py 2>/dev/null; then
    ready=true
    break
  fi
  sleep 1
done
if [ "$ready" != true ]; then
  echo 'Timed out waiting for active demo controllers' >&2
  exit 1
fi

ros2 run ros_gz_sim create -name slope_fixture \
  -file /opt/demo/fixtures/slope_fixture.sdf \
  -x 0.38 -y -0.015 -z "${FIXTURE_BASE_Z:-0.55}" -R 1.57079632679

if [ "${ENABLE_SURFACE_CAMERA:-false}" = true ] || [ "${SHOW_CAMERA_VIEWER:-false}" = true ]; then
  ros2 run ros_gz_sim create -name topdown_camera \
    -file /opt/demo/fixtures/topdown_camera.sdf \
    -x 0.20 -y -1.00 -z 1.00 -R 0.0 -P 0.0 -Y 1.00
  ros2 run ros_gz_bridge parameter_bridge \
    '/surface_camera/image@sensor_msgs/msg/Image@gz.msgs.Image' &
  children+=("$!")
  if [ "${SHOW_CAMERA_VIEWER:-false}" = true ]; then
    ros2 run image_view image_view --ros-args -r image:=/surface_camera/image &
    children+=("$!")
  fi
fi
touch /tmp/surface-ready
wait "$launch_pid"
