# Image ROS 2 Jazzy pour le poste de developpement (Ubuntu 26.04, sans Jazzy natif).
#
# Elle sert a deux choses : rviz face au robot reel, et la simulation Gazebo
# Harmonic. Une seule image, pour que les deux usages partagent exactement les
# memes versions de nav2, slam_toolbox et ros2_control que le Pi.
#
# Reconstruire : ./alidade_viz/docker/run.sh build
FROM osrf/ros:jazzy-desktop

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      ros-jazzy-joint-state-publisher-gui \
      ros-jazzy-xacro \
      ros-jazzy-ros-gz \
      ros-jazzy-gz-ros2-control \
      ros-jazzy-ros2-control \
      ros-jazzy-ros2-controllers \
      ros-jazzy-slam-toolbox \
      ros-jazzy-navigation2 \
      ros-jazzy-rplidar-ros \
      ros-jazzy-usb-cam \
      ros-jazzy-image-rotate \
      ros-jazzy-image-transport-plugins \
      ros-jazzy-aruco-opencv \
      ros-jazzy-aruco-opencv-msgs \
      ros-jazzy-rmw-cyclonedds-cpp \
      ros-jazzy-diagnostic-updater \
      ros-jazzy-diagnostic-aggregator \
      ros-jazzy-rqt-robot-monitor \
      python3-psutil \
      ros-jazzy-rosbridge-suite \
      python3-flask \
      python3-waitress \
      ros-jazzy-teleop-twist-keyboard \
      python3-serial \
      python3-yaml \
      x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Rendu logiciel : le conteneur n'a pas les pilotes GPU de l'hote.
ENV LIBGL_ALWAYS_SOFTWARE=1 \
    QT_X11_NO_MITSHM=1 \
    XDG_RUNTIME_DIR=/tmp/runtime-root

RUN mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root

WORKDIR /ws
