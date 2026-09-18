ARG BASE_IMAGE=compliant-controllers:humble-desktop-full
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive \
    FRANKA_ROS2_VER=v2.4.0 \
    LIBFRANKA_VER=0.20.5 

ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=1000

# Install the libfranka release matching the selected franka_ros2 version.
RUN curl -L https://github.com/frankarobotics/libfranka/releases/download/${LIBFRANKA_VER}/libfranka_${LIBFRANKA_VER}_$(lsb_release -cs)_amd64.deb -o libfranka.deb \
    && apt-get install ./libfranka.deb -y \
    && rm libfranka.deb

WORKDIR /controllers_ws
#RUN git clone --branch ${FRANKA_ROS2_VER} --recursive https://github.com/frankarobotics/franka_ros2 src
RUN git clone --branch ${FRANKA_ROS2_VER} --recursive \
    https://github.com/frankarobotics/franka_ros2 /tmp/franka_ros2 \
    && find /tmp/franka_ros2 -mindepth 1 -maxdepth 1 \
        \( -path '*/franka_example_controllers' \
        -o -path '*/franka_ros2' \
        -o -path '*/franka_mobile' \
        -o -path '*/franka_mobile_sensors' \
        -o -path '*/franka_fr3_moveit_config' \
        -o -path '*/.git' \) -prune \
        -o -exec cp -a {} src/ \; \
    && rm -rf /tmp/franka_ros2
    
RUN mkdir src/libfranka && touch src/libfranka/COLCON_IGNORE
RUN mkdir src/olvx_descriptions_module && touch src/olvx_descriptions_module/COLCON_IGNORE
#RUN touch src/franka_example_controllers/COLCON_IGNORE && \
#    touch src/franka_ros2/COLCON_IGNORE && \
#    touch src/franka_mobile/COLCON_IGNORE && \
#    touch src/franka_mobile_sensors/COLCON_IGNORE && \
#    touch src/franka_fr3_moveit_config/COLCON_IGNORE

RUN if dpkg --compare-versions ${FRANKA_ROS2_VER#v} ge 2.2.0; then \
      vcs import src < src/dependency.repos --recursive --skip-existing; \
    else \
      vcs import src < src/franka.repos --recursive --skip-existing; \
    fi
    
RUN rosdep install \
    --from-paths src \
    --ignore-src \
    --rosdistro ${ROS_DISTRO} \
    --dependency-types build \
    --dependency-types buildtool \
    --skip-keys="libfranka franka_mobile franka_mobile_sensors" \
    -y

COPY common/ /controllers_ws/src/common/
COPY franka_gz_sim/ /controllers_ws/src/franka_gz_sim/

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TESTING=OFF"

SHELL ["/bin/bash", "-c"]

# Gazebo's ros2_control plugin is needed at runtime, beyond build dependencies.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-gz-ros2-control \
      ros-${ROS_DISTRO}-rqt-image-view \
    && rm -rf /var/lib/apt/lists/*

CMD ["bash", "-c", "source /controllers_ws/install/setup.bash && exec ros2 launch franka_gz_sim fr3_gz.launch.py show_gazebo_gui:=${SHOW_GAZEBO_GUI:-true} show_rviz:=${SHOW_RVIZ:-false}"]
