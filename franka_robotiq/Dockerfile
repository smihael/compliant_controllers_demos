ARG BASE_IMAGE=compliant-controllers:franka-real
FROM ${BASE_IMAGE}

USER root
SHELL ["/bin/bash", "-c"]
ARG ROBOTIQ_VERSION=a29c69b7516b7a97737b5e43e118e6598ace0832
ARG SERIAL_VERSION=d8d160678aa0b31cdf467c052b954fa287cc6cdf

WORKDIR /robotiq_ws
COPY CMakeLists.txt package.xml src/franka_robotiq/
COPY launch/fr3_robotiq.launch.py src/franka_robotiq/launch/
COPY urdf src/franka_robotiq/urdf
COPY install_dependencies.sh /tmp/install_dependencies.sh
RUN bash /tmp/install_dependencies.sh && rm /tmp/install_dependencies.sh

COPY entrypoint.sh /robotiq_entrypoint.sh
RUN chmod 755 /robotiq_entrypoint.sh
ENTRYPOINT ["/robotiq_entrypoint.sh"]
CMD ["bringup"]
