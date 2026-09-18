ARG BASE_IMAGE=compliant-controllers:humble-desktop-full
FROM ${BASE_IMAGE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-gz-ros2-control ros-${ROS_DISTRO}-ros-gz \
      libgl1-mesa-dri libglx-mesa0 && rm -rf /var/lib/apt/lists/*
WORKDIR /flexiv_ws
ARG FLEXIV_DESCRIPTION_REF=f33331a7f75b9a25ec903674595ddfebbe1fb998
RUN mkdir -p src && git init src/flexiv_description && \
    git -C src/flexiv_description remote add origin https://github.com/flexivrobotics/flexiv_description.git && \
    git -C src/flexiv_description fetch --depth 1 origin ${FLEXIV_DESCRIPTION_REF} && \
    git -C src/flexiv_description checkout FETCH_HEAD
RUN source /controllers_ws/install/setup.bash && \
    colcon build --packages-select flexiv_description --cmake-args -DBUILD_TESTING=OFF
COPY common/ src/common/
COPY flexiv_gz_sim/ src/flexiv_gz_sim/
RUN source /controllers_ws/install/setup.bash && source install/setup.bash && \
    colcon build --packages-select flexiv_gz_sim --cmake-args -DBUILD_TESTING=OFF && \
    source install/setup.bash && \
    python3 src/flexiv_gz_sim/scripts/generate_model.py --output install/flexiv_gz_sim/share/flexiv_gz_sim/model
COPY flexiv_gz_sim/docker-entrypoint.sh /flexiv-gazebo-entrypoint.sh
ENTRYPOINT ["/flexiv-gazebo-entrypoint.sh"]
CMD ["bash", "-c", "exec ros2 launch flexiv_gz_sim rizon4s_gz.launch.py show_gazebo_gui:=${SHOW_GAZEBO_GUI:-false}"]
