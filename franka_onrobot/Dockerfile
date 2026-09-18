ARG BASE_IMAGE=compliant-controllers:franka-real
FROM ${BASE_IMAGE}

USER root
SHELL ["/bin/bash", "-c"]
ARG ONROBOT_VERSION=1b3a064dda926c58da079a1baf66c2036af41072
ARG NET_FT_VERSION=9e8c0c872172c6f594eece68a2c1c5598fc81b4b

WORKDIR /onrobot_ws
COPY CMakeLists.txt package.xml src/franka_onrobot/
COPY launch/fr3_onrobot.launch.py src/franka_onrobot/launch/
COPY urdf src/franka_onrobot/urdf
COPY install_dependencies.sh /tmp/install_dependencies.sh
RUN bash /tmp/install_dependencies.sh && rm /tmp/install_dependencies.sh

COPY entrypoint.sh /onrobot_entrypoint.sh
RUN chmod 755 /onrobot_entrypoint.sh
ENTRYPOINT ["/onrobot_entrypoint.sh"]
CMD ["bringup"]
