# Reuse the exact message ABI from the existing simulator image.
ARG SIM_IMAGE=compliant-controllers:franka-gz-sim
FROM ${SIM_IMAGE} AS simulation

FROM ros:humble-ros-core AS builder
SHELL ["/bin/bash", "-c"]
ARG SPACEMOUSE_REV=1d041da5a34bb040f8a97192504c9137eb0b8923
RUN apt-get update && apt-get install -y --no-install-recommends \
      git cmake make g++ python3-colcon-common-extensions python3-pip \
      ros-humble-ament-cmake \
    && rm -rf /var/lib/apt/lists/*
RUN git clone https://github.com/frankarobotics/franka_spacemouse.git /tmp/spacemouse \
    && git -C /tmp/spacemouse checkout "${SPACEMOUSE_REV}" \
    && mkdir -p /teleop_ws/src \
    && cp -a /tmp/spacemouse/src/spacemouse_publisher /teleop_ws/src/ \
    && rm -rf /tmp/spacemouse
RUN pip3 install --no-cache-dir --target /python-deps \
      pyspacemouse==1.1.4 easyhid==0.0.10
COPY CMakeLists.txt package.xml /teleop_ws/src/spacemouse_teleop/
COPY launch /teleop_ws/src/spacemouse_teleop/launch
COPY scripts /teleop_ws/src/spacemouse_teleop/scripts
WORKDIR /teleop_ws
RUN source /opt/ros/humble/setup.bash && colcon build --merge-install \
      --cmake-args -DBUILD_TESTING=OFF

FROM ros:humble-ros-core
RUN apt-get update && apt-get install -y --no-install-recommends \
      libhidapi-hidraw0 ros-humble-tf2-ros-py ros-humble-launch-ros \
    && rm -rf /var/lib/apt/lists/*
COPY --from=simulation /controllers_ws/install/compliant_controllers_msgs /controllers_ws/install/compliant_controllers_msgs
COPY --from=builder /python-deps /opt/python-deps
COPY --from=builder /teleop_ws/install /teleop_ws/install
ENV PYTHONPATH=/opt/python-deps
COPY docker-entrypoint.sh /teleop-entrypoint.sh
ENTRYPOINT ["/bin/bash", "/teleop-entrypoint.sh"]
CMD ["ros2", "launch", "spacemouse_teleop", "fr3_spacemouse_teleop.launch.py", "namespace:=/", "use_sim_time:=false", "device_path:=/dev/hidraw3"]
