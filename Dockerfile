FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble

ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=1000

RUN sed -i \
    -e 's|http://archive.ubuntu.com/ubuntu/|https://mirrors.tuna.tsinghua.edu.cn/ubuntu/|g' \
    -e 's|http://security.ubuntu.com/ubuntu/|https://mirrors.tuna.tsinghua.edu.cn/ubuntu/|g' \
    /etc/apt/sources.list && \
    sed -i \
    -e 's|http://packages.ros.org/ros2/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu|g' \
    -e 's|^Types: .*|Types: deb|g' \
    /etc/apt/sources.list.d/ros2.sources

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-vcstool \
        python3-argcomplete \
        git \
        curl \
        sudo \
        build-essential \
        cmake \
        ros-dev-tools \
        mesa-utils

RUN apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-sdformat-urdf \
        ros-${ROS_DISTRO}-joint-state-publisher-gui \
        ros-${ROS_DISTRO}-ros2controlcli \
        ros-${ROS_DISTRO}-controller-interface \
        ros-${ROS_DISTRO}-hardware-interface-testing \
        ros-${ROS_DISTRO}-ament-cmake-clang-format \
        ros-${ROS_DISTRO}-ament-cmake-clang-tidy \
        ros-${ROS_DISTRO}-controller-manager \
        ros-${ROS_DISTRO}-ros2-control-test-assets \
        ros-${ROS_DISTRO}-hardware-interface \
        ros-${ROS_DISTRO}-control-msgs \
        ros-${ROS_DISTRO}-backward-ros \
        ros-${ROS_DISTRO}-generate-parameter-library \
        ros-${ROS_DISTRO}-realtime-tools \
        ros-${ROS_DISTRO}-joint-state-publisher \
        ros-${ROS_DISTRO}-joint-state-broadcaster \
        ros-${ROS_DISTRO}-joint-trajectory-controller \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-ur-description \
        ros-${ROS_DISTRO}-pinocchio \
        ros-${ROS_DISTRO}-eigenpy \
        libboost-python-dev

# Only the manifest comes from the build context; package sources come from Git.
COPY controllers.repos /controllers_ws/controllers.repos
WORKDIR /controllers_ws
RUN mkdir -p src && vcs import src --input controllers.repos && \
    vcs export src --exact > sources.repos
RUN rosdep install \
    --from-paths src \
    --ignore-src \
    --rosdistro ${ROS_DISTRO} \
    --dependency-types build \
    --dependency-types buildtool \
    -y

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TESTING=OFF"

RUN if ! getent group ${USER_GID} >/dev/null; then groupadd --gid ${USER_GID} ${USERNAME}; fi && \
    useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME} && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "source /controllers_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/${USERNAME}/.bashrc && \
    chown -R ${USER_UID}:${USER_GID} /home/${USERNAME} /controllers_ws

WORKDIR /controllers_ws

SHELL ["/bin/bash", "-c"]
