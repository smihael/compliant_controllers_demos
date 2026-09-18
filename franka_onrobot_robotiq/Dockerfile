ARG BASE_IMAGE=compliant-controllers:franka-real
FROM ${BASE_IMAGE}

USER root
SHELL ["/bin/bash", "-c"]
ARG ONROBOT_VERSION=1b3a064dda926c58da079a1baf66c2036af41072
ARG NET_FT_VERSION=9e8c0c872172c6f594eece68a2c1c5598fc81b4b
ARG ROBOTIQ_VERSION=a29c69b7516b7a97737b5e43e118e6598ace0832
ARG SERIAL_VERSION=d8d160678aa0b31cdf467c052b954fa287cc6cdf

WORKDIR /onrobot_robotiq_ws
COPY CMakeLists.txt package.xml src/franka_onrobot_robotiq/
COPY launch/fr3_onrobot_robotiq.launch.py src/franka_onrobot_robotiq/launch/
COPY urdf src/franka_onrobot_robotiq/urdf
COPY install_dependencies.sh /tmp/install_dependencies.sh
RUN bash /tmp/install_dependencies.sh && rm /tmp/install_dependencies.sh

COPY entrypoint.sh /onrobot_robotiq_entrypoint.sh
RUN chmod 755 /onrobot_robotiq_entrypoint.sh
ENTRYPOINT ["/onrobot_robotiq_entrypoint.sh"]
CMD ["bringup"]
