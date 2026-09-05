# Environnement ROS 2 commun au robot et au poste de developpement.
#
# Source par le service systemd, par alidade_viz/docker/run.sh dans le
# conteneur, et a ajouter au ~/.bashrc du Pi :
#     source ~/alidade_ws/src/alidade_ros2/system/env/alidade_env.sh
#
# Les deux machines doivent avoir le meme ROS_DOMAIN_ID et le meme RMW.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Pairs de decouverte declares en dur (voir dds/cyclonedds.xml) : la
# decouverte multicast est trop peu fiable en WiFi. Un hote peut se citer
# lui-meme sans dommage, ce qui permet des valeurs par defaut symetriques.
export ALIDADE_ROBOT_HOST="${ALIDADE_ROBOT_HOST:-alidade.local}"
export ALIDADE_DEV_HOST="${ALIDADE_DEV_HOST:-$(hostname).local}"

_alidade_system_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export CYCLONEDDS_URI="file://${_alidade_system_dir}/dds/cyclonedds.xml"
unset _alidade_system_dir
