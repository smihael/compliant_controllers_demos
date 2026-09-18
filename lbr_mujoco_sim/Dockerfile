ARG BASE_IMAGE=compliant-controllers:humble-desktop-full
FROM ${BASE_IMAGE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG MUJOCO_ROS2_CONTROL_REF=58f465b1b4634a6f366e71919189574f5d95c1e8
ENV DEBIAN_FRONTEND=noninteractive

# Separate overlay: retain the shared controller installation.
WORKDIR /mujoco_ws
RUN mkdir -p src && \
    git init src/mujoco_ros2_control && \
    git -C src/mujoco_ros2_control remote add origin https://github.com/ros-controls/mujoco_ros2_control.git && \
    git -C src/mujoco_ros2_control fetch --depth 1 origin ${MUJOCO_ROS2_CONTROL_REF} && \
    git -C src/mujoco_ros2_control checkout FETCH_HEAD

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    apt-get update && apt-get install -y --no-install-recommends \
      python3-pip python3-venv libgl1-mesa-dri libglx-mesa0 && \
    rosdep update --rosdistro ${ROS_DISTRO} && \
    rosdep install -y --ignore-src --rosdistro ${ROS_DISTRO} \
      --dependency-types build --dependency-types buildtool --dependency-types exec --from-paths \
      src/mujoco_ros2_control/mujoco_ros2_control \
      src/mujoco_ros2_control/mujoco_ros2_control_msgs \
      src/mujoco_ros2_control/mujoco_ros2_control_plugins && \
    rm -rf /var/lib/apt/lists/*

# Python MuJoCo is used only to generate/validate MJCF at image build time.
RUN python3 -m venv --system-site-packages /opt/mujoco-model && \
    /opt/mujoco-model/bin/pip install --no-cache-dir mujoco==3.4.0
# Build upstream separately so editing this demo preserves the compilation cache.
# Ignore the lidar test dependency: it is not needed with BUILD_TESTING=OFF.
RUN source /controllers_ws/install/setup.bash && \
    colcon build --packages-select mujoco_ros2_control_msgs mujoco_ros2_control_plugins \
      mujoco_ros2_control \
      --packages-ignore mujoco_3d_lidar \
      --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
RUN /opt/mujoco-model/bin/pip install --no-cache-dir trimesh==4.11.2 pycollada==0.9.2
ARG LBR_DESCRIPTION_REF=52f816bd59cbd2390efc890cca0ad61389e4ccb3
RUN git init /tmp/lbr-stack && \
    git -C /tmp/lbr-stack remote add origin https://github.com/lbr-stack/lbr_fri_ros2_stack.git && \
    git -C /tmp/lbr-stack fetch --depth 1 origin ${LBR_DESCRIPTION_REF} && \
    git -C /tmp/lbr-stack checkout FETCH_HEAD && \
    mv /tmp/lbr-stack/lbr_description src/lbr_description && rm -rf /tmp/lbr-stack
RUN source /controllers_ws/install/setup.bash && source install/setup.bash && \
    colcon build --packages-select lbr_description --packages-ignore mujoco_3d_lidar \
      --cmake-args -DBUILD_TESTING=OFF
COPY common/ src/common/
COPY lbr_mujoco_sim/ src/lbr_mujoco_sim/
RUN source /controllers_ws/install/setup.bash && source install/setup.bash && \
    colcon build --packages-select lbr_mujoco_sim --packages-ignore mujoco_3d_lidar --cmake-args -DBUILD_TESTING=OFF && \
    source install/setup.bash && \
    /opt/mujoco-model/bin/python src/lbr_mujoco_sim/scripts/generate_model.py \
      --output install/lbr_mujoco_sim/share/lbr_mujoco_sim/model

COPY common/docker/mujoco-entrypoint.sh /lbr-mujoco-entrypoint.sh
ENTRYPOINT ["/lbr-mujoco-entrypoint.sh"]
CMD ["bash", "-c", "exec ros2 launch lbr_mujoco_sim iiwa14_mujoco.launch.py headless:=${MUJOCO_HEADLESS:-true} controller_name:=${CONTROLLER_NAME:-cartesian_impedance_controller}"]
