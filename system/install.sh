#!/usr/bin/env bash
# Integration systeme sur le Pi : paquets apt, regles udev, service systemd.
# A lancer une fois, depuis le depot clone, apres avoir compile le workspace :
#
#   sudo ./system/install.sh [utilisateur] [workspace]
#
# Par defaut l'utilisateur est celui qui a appele sudo et le workspace est
# le parent de src/alidade_ros2.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
USER_NAME="${1:-${SUDO_USER:-$USER}}"
WORKSPACE="${2:-$(cd "$HERE/../../.." && pwd)}"

[ "$(id -u)" -eq 0 ] || { echo "a lancer avec sudo" >&2; exit 1; }
[ -d "$WORKSPACE/install" ] || echo "attention : $WORKSPACE/install absent, compiler d'abord avec colcon build" >&2

echo "== paquets apt"
apt-get update
grep -vE '^\s*(#|$)' "$HERE/apt-packages.txt" | xargs apt-get install -y --no-install-recommends

echo "== droits serie et video pour $USER_NAME"
usermod -aG dialout,video "$USER_NAME"

echo "== regles udev"
install -m 644 "$HERE/udev/99-alidade.rules" /etc/udev/rules.d/
udevadm control --reload
udevadm trigger

echo "== service systemd (utilisateur $USER_NAME, workspace $WORKSPACE)"
sed -e "s|@USER@|$USER_NAME|g" -e "s|@WORKSPACE@|$WORKSPACE|g" \
    "$HERE/systemd/alidade.service" > /etc/systemd/system/alidade.service
if [ ! -f /etc/default/alidade ]; then
  cat > /etc/default/alidade <<'DEFAULTS'
# Arguments passes a robot.launch.py par le service alidade.
ALIDADE_LAUNCH_ARGS="nav:=true"
# Pairs DDS (voir system/env/alidade_env.sh) ; a renseigner si les noms .local ne suffisent pas.
#ALIDADE_DEV_HOST=portable.local
#ROS_DOMAIN_ID=7
DEFAULTS
fi
systemctl daemon-reload
systemctl enable alidade

echo
echo "fait. Rebrancher les peripheriques (ou redemarrer) puis verifier :"
echo "  ls -l /dev/rplidar /dev/pico /dev/camera"
echo "  sudo systemctl start alidade && journalctl -u alidade -f"
