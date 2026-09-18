ARG BASE_IMAGE=compliant-controllers:humble-desktop-full
FROM ${BASE_IMAGE}

ARG FRI_CLIENT_VERSION=1.15
ENV FRI_CLIENT_VERSION=${FRI_CLIENT_VERSION}

RUN apt-get -o Acquire::Retries=5 update && \
    apt-get -o Acquire::Retries=5 install -y --no-install-recommends libasio-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /lbr_ws
RUN git config --global http.version HTTP/1.1 && \
    mkdir -p src && \
    vcs import src --shallow --input \
      https://raw.githubusercontent.com/lbr-stack/lbr_fri_ros2_stack/humble/\
lbr_fri_ros2_stack/repos-fri-${FRI_CLIENT_VERSION}.yaml

COPY . src/lbr_gz_sim

SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source /controllers_ws/install/setup.bash && \
    rosdep update --rosdistro ${ROS_DISTRO} && \
    apt-get -o Acquire::Retries=5 update && \
    rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

CMD ["bash", "-c", "source /lbr_ws/install/setup.bash && exec ros2 launch lbr_gz_sim lbr_gazebo.launch.py model:=iiwa14 ctrl:=cartesian_impedance_controller show_gazebo_gui:=${SHOW_GAZEBO_GUI:-true} show_rviz:=${SHOW_RVIZ:-false}"]
