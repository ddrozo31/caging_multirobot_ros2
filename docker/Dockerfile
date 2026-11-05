# ==================================================================================================
# Caging simulator in a container (caging_multirobot_ros2).
#
# The frozen simulator (tag v1.0: src/ of this repository) on ROS 2 Jazzy and Gazebo Harmonic,
# built once inside the image, so a student needs only Docker to run the lecture demos.
#
#   docker compose -f docker/compose.yaml build          # from the repository root
#   docker compose -f docker/compose.yaml run --rm test  # 32 unit tests inside the container
#
# Steps of the image:
#   1. base: osrf/ros:jazzy-desktop (ROS 2 Jazzy, RViz, Ubuntu 24.04), upgraded to the current release
#   2. a normal user with the same UID/GID as the person who builds it, so files written to
#      mounted folders keep their owner
#   3. the dependencies declared in the packages' package.xml, installed by rosdep
#      (Gazebo Harmonic comes with ros_gz_sim, ros2_control with gz_ros2_control)
#   4. colcon build of the five caging_* packages
# ==================================================================================================
FROM osrf/ros:jazzy-desktop

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy \
    WS=/home/ros/ros2_ws

# ---- 2. user: Ubuntu 24.04 images already have "ubuntu" with UID 1000; replace it by our user ----
RUN if id ubuntu >/dev/null 2>&1; then userdel -r ubuntu; fi \
 && groupadd --gid ${USER_GID} ${USERNAME} \
 && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
 && apt-get update && apt-get install -y --no-install-recommends sudo \
 && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME} \
 && rm -rf /var/lib/apt/lists/*

# ---- 2b. bring every ROS 2 package of the base image to the current Jazzy release ----
# The base image is an older snapshot; packages installed later by rosdep are built against newer
# libraries (e.g. Fast-CDR) and fail at run time with "undefined symbol" unless the base is upgraded.
# mesa-utils: `glxinfo -B` shows which GPU draws Gazebo and RViz (GUI check)
RUN apt-get update && apt-get dist-upgrade -y \
 && apt-get install -y --no-install-recommends mesa-utils \
 && rm -rf /var/lib/apt/lists/*

# ---- 3. dependencies from the package.xml files (copied alone, so this layer is cached) ----
COPY --chown=${USERNAME}:${USERNAME} src/caging_description/package.xml       ${WS}/src/caging_description/
COPY --chown=${USERNAME}:${USERNAME} src/caging_gazebo/package.xml            ${WS}/src/caging_gazebo/
COPY --chown=${USERNAME}:${USERNAME} src/caging_mecanum_controller/package.xml ${WS}/src/caging_mecanum_controller/
COPY --chown=${USERNAME}:${USERNAME} src/caging_control/package.xml           ${WS}/src/caging_control/
COPY --chown=${USERNAME}:${USERNAME} src/caging_bringup/package.xml           ${WS}/src/caging_bringup/
RUN apt-get update \
 && rosdep update --rosdistro ${ROS_DISTRO} \
 && rosdep install --from-paths ${WS}/src --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && rm -rf /var/lib/apt/lists/*

# ---- 4. the simulator itself, built as the normal user ----
USER ${USERNAME}
WORKDIR ${WS}
COPY --chown=${USERNAME}:${USERNAME} src ${WS}/src
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && colcon build --event-handlers console_direct- \
 && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc \
 && echo "source ${WS}/install/setup.bash" >> /home/${USERNAME}/.bashrc

COPY --chown=${USERNAME}:${USERNAME} docker/entrypoint.sh /home/${USERNAME}/entrypoint.sh
ENTRYPOINT ["/home/ros/entrypoint.sh"]
CMD ["bash"]

# ---- 5. labels of the published image; last, so a new revision does not rebuild the layers ----
ARG VERSION=local
ARG REVISION=unknown
LABEL org.opencontainers.image.title="caging_multirobot_ros2" \
      org.opencontainers.image.description="Caging simulator (Tejada et al. 2025) on ROS 2 Jazzy and Gazebo Harmonic, Universidad EIA" \
      org.opencontainers.image.authors="David Rozo-Osorio (Universidad EIA)" \
      org.opencontainers.image.licenses="Apache-2.0" \
      org.opencontainers.image.source="https://github.com/ddrozo31/caging_multirobot_ros2" \
      org.opencontainers.image.version="${VERSION}" \
      org.opencontainers.image.revision="${REVISION}"
