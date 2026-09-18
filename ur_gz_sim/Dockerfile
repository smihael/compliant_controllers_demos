ARG BASE_IMAGE=compliant-controllers:humble-desktop-full
FROM ${BASE_IMAGE}

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ur-simulation-gz \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ur_ws
COPY . src/ur_gz_sim
SHELL ["/bin/bash", "-c"]
RUN source /controllers_ws/install/setup.bash && \
    colcon build --packages-select ur_gz_sim \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

CMD ["bash", "-c", "source /ur_ws/install/setup.bash && exec ros2 launch ur_gz_sim ur_gz.launch.py ur_type:=${UR_TYPE:-ur5e} gazebo_gui:=${SHOW_GAZEBO_GUI:-true} launch_rviz:=${SHOW_RVIZ:-false}"]
