#!/usr/bin/env bash
# Conteneur ROS 2 Jazzy pour le poste de developpement (Ubuntu 26.04, pas de
# Jazzy natif). Meme distribution que le Pi, donc rviz et la simulation y
# parlent le meme DDS que le robot.
#
#   ./run.sh build             (re)construit l'image alidade:jazzy
#   ./run.sh up                demarre le conteneur et compile le workspace
#   ./run.sh rviz [robot|slam|nav] [args]   rviz face au robot reel
#   ./run.sh sim [args]        simulation Gazebo (ex. nav:=true), avec rviz dans un second terminal :
#   ./run.sh rviz nav use_sim_time:=true
#   ./run.sh teleop            clavier -> diff_drive_controller (TwistStamped)
#   ./run.sh test              colcon test
#   ./run.sh frames            genere out/frames.pdf via tf2_tools view_frames
#   ./run.sh shell             shell dans le conteneur
#   ./run.sh down              supprime le conteneur
#
# Multi-machines : le conteneur est en --net=host et source system/env/alidade_env.sh,
# comme le Pi. ALIDADE_ROBOT_HOST (defaut alidade.local) est resolu ici, sur
# l'hote, parce que le conteneur ne sait pas resoudre les noms mDNS.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
IMAGE=alidade:jazzy
NAME=alidade_jazzy
XAUTH="$HOME/.docker.xauth"
ROBOT_HOST="${ALIDADE_ROBOT_HOST:-alidade.local}"

setup_xauth() {
  rm -f "$XAUTH"; touch "$XAUTH"
  xauth nlist "${DISPLAY:?DISPLAY non defini}" | sed -e 's/^..../ffff/' | xauth -f "$XAUTH" nmerge -
  chmod 644 "$XAUTH"
}

resolve_robot() {
  # IP du robot pour les pairs CycloneDDS ; on garde le nom si la resolution echoue.
  getent ahostsv4 "$ROBOT_HOST" 2>/dev/null | awk 'NR==1 {print $1}' || true
}

in_container() {
  # Environnement complet : ROS, workspace, DDS multi-machines.
  docker exec -it \
    -e ALIDADE_ROBOT_HOST="$(resolve_robot)" \
    "$NAME" bash -lc "source /ws/install/setup.bash && source /ws/src/alidade_ros2/system/env/alidade_env.sh && $*"
}

case "${1:-up}" in
  build)
    docker build -t "$IMAGE" -f "$REPO/alidade_viz/docker/Dockerfile" "$REPO/alidade_viz/docker"
    ;;
  up)
    setup_xauth
    mkdir -p "$REPO/out"
    docker rm -f "$NAME" >/dev/null 2>&1 || true
    docker run -d --name "$NAME" \
      -e DISPLAY="$DISPLAY" \
      -e XAUTHORITY=/root/.Xauthority \
      -v "$XAUTH:/root/.Xauthority:ro" \
      -v "$REPO:/ws/src/alidade_ros2" \
      -v "$REPO/out:/out:rw" \
      --net=host --ipc=host \
      "$IMAGE" sleep infinity
    docker exec "$NAME" bash -lc \
      'cd /ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install'
    ;;
  rviz)
    view="${2:-robot}"; shift; [ $# -gt 0 ] && shift
    in_container "ros2 launch alidade_viz rviz.launch.py view:=$view $*"
    ;;
  sim)
    shift
    in_container "ros2 launch alidade_sim sim.launch.py $*"
    ;;
  teleop)
    in_container "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -p frame_id:=base_footprint -r cmd_vel:=/diff_drive_controller/cmd_vel"
    ;;
  test)
    docker exec -it "$NAME" bash -lc 'cd /ws && source install/setup.bash && colcon test && colcon test-result --verbose'
    ;;
  frames)
    in_container "cd /out && ros2 run tf2_tools view_frames"
    ;;
  shell) in_container "bash" ;;
  down)  docker rm -f "$NAME" ;;
  *)     echo "usage: $0 {build|up|rviz [robot|slam|nav]|sim [args]|teleop|test|frames|shell|down}" >&2; exit 1 ;;
esac
