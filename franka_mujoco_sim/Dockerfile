ARG BASE_IMAGE=compliant-controllers:humble-desktop-full
FROM ${BASE_IMAGE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG MUJOCO_ROS2_CONTROL_REF=58f465b1b4634a6f366e71919189574f5d95c1e8
ARG FRANKA_DESCRIPTION_REF=2.7.0
ENV DEBIAN_FRONTEND=noninteractive

# Separate overlay: retain the shared controller installation.
WORKDIR /mujoco_ws
RUN mkdir -p src && \
    git init src/mujoco_ros2_control && \
    git -C src/mujoco_ros2_control remote add origin https://github.com/ros-controls/mujoco_ros2_control.git && \
    git -C src/mujoco_ros2_control fetch --depth 1 origin ${MUJOCO_ROS2_CONTROL_REF} && \
    git -C src/mujoco_ros2_control checkout FETCH_HEAD && \
    git clone --depth 1 --branch ${FRANKA_DESCRIPTION_REF} \
      https://github.com/frankarobotics/franka_description.git src/franka_description

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    apt-get update && apt-get install -y --no-install-recommends \
      python3-pip python3-venv libgl1-mesa-dri libglx-mesa0 && \
    rosdep update --rosdistro ${ROS_DISTRO} && \
    rosdep install -y --ignore-src --rosdistro ${ROS_DISTRO} \
      --dependency-types build --dependency-types buildtool --dependency-types exec --from-paths \
      src/mujoco_ros2_control/mujoco_ros2_control \
      src/mujoco_ros2_control/mujoco_ros2_control_msgs \
      src/mujoco_ros2_control/mujoco_ros2_control_plugins src/franka_description && \
    rm -rf /var/lib/apt/lists/*

# Python MuJoCo is used only to generate/validate MJCF at image build time.
RUN python3 -m venv --system-site-packages /opt/mujoco-model && \
    /opt/mujoco-model/bin/pip install --no-cache-dir mujoco==3.4.0
# Build upstream separately so editing this demo preserves the compilation cache.
# Ignore the lidar test dependency: it is not needed with BUILD_TESTING=OFF.
RUN source /controllers_ws/install/setup.bash && \
    colcon build --packages-select mujoco_ros2_control_msgs mujoco_ros2_control_plugins \
      mujoco_ros2_control franka_description \
      --packages-ignore mujoco_3d_lidar \
      --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
RUN /opt/mujoco-model/bin/pip install --no-cache-dir trimesh==4.11.2 pycollada==0.9.2
COPY common/ src/common/
COPY franka_mujoco_sim/ src/franka_mujoco_sim/
RUN source /controllers_ws/install/setup.bash && source install/setup.bash && \
    colcon build --packages-select franka_mujoco_sim --packages-ignore mujoco_3d_lidar --cmake-args -DBUILD_TESTING=OFF && \
    source install/setup.bash && \
    /opt/mujoco-model/bin/python src/franka_mujoco_sim/scripts/generate_model.py \
      --output install/franka_mujoco_sim/share/franka_mujoco_sim/model

COPY common/docker/mujoco-entrypoint.sh /franka-mujoco-entrypoint.sh
ENTRYPOINT ["/franka-mujoco-entrypoint.sh"]
CMD ["bash", "-c", "exec ros2 launch franka_mujoco_sim fr3_mujoco.launch.py headless:=${MUJOCO_HEADLESS:-true} controller_name:=${CONTROLLER_NAME:-cartesian_impedance_controller}"]
