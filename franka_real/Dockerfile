ARG BASE_IMAGE=compliant-controllers:humble-desktop-full
FROM ${BASE_IMAGE}

USER root
SHELL ["/bin/bash", "-c"]
ARG FRANKA_ROS2_VERSION=v2.4.0
ARG LIBFRANKA_VERSION=0.20.5
ARG FRANKA_DESCRIPTION_VERSION=2.7.0

# Use the same Franka releases as the Gazebo demo, with hardware packages only.
RUN apt-get update && \
    curl --fail --location --retry 3 \
      "https://github.com/frankarobotics/libfranka/releases/download/${LIBFRANKA_VERSION}/libfranka_${LIBFRANKA_VERSION}_$(. /etc/os-release && echo "$VERSION_CODENAME")_amd64.deb" \
      -o /tmp/libfranka.deb && \
    apt-get install -y --no-install-recommends /tmp/libfranka.deb && \
    rm /tmp/libfranka.deb

WORKDIR /controllers_ws
RUN git clone --depth 1 --branch "${FRANKA_ROS2_VERSION}" \
      https://github.com/frankarobotics/franka_ros2 /tmp/franka_ros2 && \
    for package in franka_bringup franka_gripper franka_hardware franka_msgs \
                   franka_robot_state_broadcaster franka_semantic_components; do \
      cp -a "/tmp/franka_ros2/${package}" src/; \
    done && \
    rm -rf /tmp/franka_ros2 && \
    git clone --depth 1 --branch "${FRANKA_DESCRIPTION_VERSION}" \
      https://github.com/frankarobotics/franka_description src/franka_description

COPY launch src/franka_real/launch
COPY config src/franka_real/config
COPY CMakeLists.txt package.xml src/franka_real/
# Bringup also declares mobile and example stacks unused by fr3.launch.py.
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} \
      --skip-keys "libfranka franka_mobile franka_mobile_sensors franka_example_controllers" -y && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF && \
    rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh /franka_entrypoint.sh
RUN chmod 755 /franka_entrypoint.sh
ENTRYPOINT ["/franka_entrypoint.sh"]
CMD ["bash", "-c", "exec ros2 launch franka_real fr3.launch.py robot_ip:=\"${ROBOT_IP:?Set ROBOT_IP to the FCI address}\" load_gripper:=\"${LOAD_GRIPPER:-false}\" controller_name:=\"${CONTROLLER_NAME:-cartesian_impedance_controller}\""]
