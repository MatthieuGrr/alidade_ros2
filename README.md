# alidade_ros2

Pile ROS 2 Jazzy d'**Alidade**, base mobile différentielle : Raspberry Pi 5,
Pico 2 sous [diffdrive_pico_firmware](https://github.com/matthieu/diffdrive_pico_firmware),
RPLIDAR C1, BNO055. Odométrie par `ros2_control`, cartographie `slam_toolbox`,
navigation `nav2`, simulation Gazebo Harmonic à parité avec le robot réel.

L'alidade est la règle de visée qu'on pose sur la planchette du topographe
pour relever un azimut et tracer une carte — l'ancêtre du LiDAR rotatif.

```
alidade_ros2/
├── alidade_description/   export Onshape corrigé, xacro, constantes physiques par révision
├── alidade_hardware/      interface ros2_control vers le Pico (C++), banc, faux Pico
├── alidade_bringup/       launch et configuration, communs robot réel / simulation
├── alidade_sim/           Gazebo Harmonic : arène, LiDAR C1 simulé, gz_ros2_control
├── alidade_viz/           vues rviz, Dockerfile et run.sh du poste de développement
└── system/                udev, systemd, CycloneDDS, install.sh pour le Pi
```

## Architecture

```
                 Raspberry Pi 5 (Jazzy)                              Poste (Docker Jazzy)
 ┌──────────────────────────────────────────────────────┐        ┌──────────────────────┐
 │ rplidar_ros ──/scan──▶ slam_toolbox ──map→odom──▶ TF │  WiFi  │ rviz2                │
 │                          ▲                            │◀──────▶│  « 2D Goal Pose »    │
 │ ros2_control_node        │ /map                       │Cyclone │  ou                  │
 │  ├ PicoSystem ◀──USB──▶ Pico 2 (ticks, IMU brute)     │  DDS   │ Gazebo + même pile   │
 │  ├ diff_drive_controller ─odom→base_footprint─▶ TF    │        └──────────────────────┘
 │  ├ joint_state_broadcaster ──/joint_states            │
 │  └ imu_sensor_broadcaster ──/imu/data_raw             │
 │ nav2 (6 nœuds) ──Twist──▶ cmd_vel_stamper ──TwistStamped──▶ diff_drive_controller
 │ usb_cam ──/camera/raw/image──▶ ImageFlipNode ──/camera/image (droite) + camera_info
 │ system_monitor (Pi, cadences) ─┐
 │ PicoSystem, nav2 ──/diagnostics─┴─▶ diagnostic_aggregator ──/diagnostics_agg
 └──────────────────────────────────────────────────────┘
```

Frontière d'abstraction : le Pico ne connaît que des *ticks* et des *ticks/s*.
Toute la cinématique (rayon, entraxe, counts par tour) vit ici, dans un seul
fichier : `alidade_description/config/alidade_v1.yaml`.

## Déploiement sur un Pi fraîchement installé

Prérequis : Ubuntu 24.04 server arm64 avec ROS 2 Jazzy installé par apt
(`ros-jazzy-ros-base` suffit), accès ssh, nom d'hôte `alidade` de préférence.

```bash
# 1. Workspace et sources
mkdir -p ~/alidade_ws/src && cd ~/alidade_ws/src
git clone <url>/alidade_ros2.git
cd ~/alidade_ws

# 2. Paquets système (apt, udev, systemd) — une seule fois
sudo ./src/alidade_ros2/system/install.sh

# 3. Compilation
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

# 4. Environnement, à ajouter au ~/.bashrc
source ~/alidade_ws/install/setup.bash
source ~/alidade_ws/src/alidade_ros2/system/env/alidade_env.sh
```

Se déconnecter et se reconnecter (groupe `dialout`), brancher le LiDAR et le
Pico, vérifier `ls -l /dev/rplidar /dev/pico`. La règle udev du Pico
reconnaît le descripteur USB `diffdrive_pico_firmware` que le firmware
annonce ; un Pico flashé avec autre chose n'apparaît pas en `/dev/pico`.

### Première mise en route, étage par étage

1. **Le Pico seul, sans ROS.** Robot surélevé.
   ```bash
   ros2 run alidade_hardware pico_bench.py monitor     # trames à 50 Hz, 0 rejet, accel z ≈ 9.81
   ros2 run alidade_hardware pico_bench.py wheels      # chaque roue avance quand on le lui demande
   ros2 run alidade_hardware pico_bench.py drive 500 500 3   # distance en m d'après la révision
   ```
   Une roue qui recule ou qui compte à l'envers se corrige dans le firmware
   (`MOT_x_INVERT`, `ENC_x_INVERT`, mode `t 3`), jamais ici.

2. **La base sous ros2_control.**
   ```bash
   ros2 launch alidade_bringup robot.launch.py
   ros2 control list_controllers        # trois contrôleurs active
   ros2 topic hz /joint_states          # 50 Hz
   ros2 topic hz /scan                  # ~10 Hz, frame_id laser
   ros2 topic echo /imu/data_raw --once # ≈ 9.81 m/s² sur z à l'arrêt
   ros2 topic hz /camera/image          # 15 Hz, frame_id camera_upright_optical_frame
   ros2 run tf2_ros tf2_echo base_footprint wheel_left_link
   ```
   L'état d'ensemble est dans `/diagnostics_agg`, regroupé par sous-système
   (Système, Pico, Capteurs, Navigation) ; sur le poste,
   `ros2 run rqt_robot_monitor rqt_robot_monitor` l'affiche en arbre. Les
   seuils (température, rail 5 V, WiFi, cadences) sont dans la section
   `telemetry` de la révision.

   La caméra est montée tête en bas : `/camera/raw/image` est à l'envers
   (repère optique physique), `/camera/image` est remise à l'endroit par
   `ImageFlipNode` et c'est elle qu'on consomme. Sans `/dev/camera`, le
   launch le dit et continue sans caméra.

   **Calibration de la caméra**, une fois, sur le flux droit (le nœud de
   retournement recopie le `camera_info` sans corriger le point principal,
   donc on calibre ce qu'on consomme), mise au point fixe comme en marche :
   ```bash
   ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 \
       --no-service-check image:=/camera/image
   ```
   Le YAML produit remplace `alidade_description/config/camera_v1.yaml`
   (provisoire : focale nominale, sans distorsion) ; reporter ensuite
   `2·atan(w/2fx)` dans `camera.sim_horizontal_fov` pour que Gazebo voie
   la même ouverture.
   Puis un mètre au sol :
   ```bash
   ros2 topic pub -r 20 /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
       '{twist: {linear: {x: 0.2}}}'
   ros2 topic echo /diff_drive_controller/odom --field pose.pose.position
   ```
   ou au clavier : `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r cmd_vel:=/diff_drive_controller/cmd_vel`.

3. **Cartographie** : `robot.launch.py slam:=true`, puis rviz sur le poste
   (`./alidade_viz/docker/run.sh rviz slam`).

4. **Navigation** : `robot.launch.py nav:=true` et l'outil « 2D Goal Pose »
   de rviz (`run.sh rviz nav`). Un but posé sur un obstacle échoue en
   silence côté rviz ; le message est dans le journal du `planner_server`
   (« Failed to create plan »).

5. **Au boot** : `sudo systemctl enable --now alidade`. Les arguments du
   launch se règlent dans `/etc/default/alidade`. Journaux :
   `journalctl -u alidade -f`.

### Arguments de `robot.launch.py`

| Argument | Défaut | Rôle |
|---|---|---|
| `slam` | false | slam_toolbox en mode mapping, transitions lifecycle pilotées |
| `nav` | false | nav2 (implique slam) |
| `mock` | false | `mock_components` à la place du Pico, pour tester sans matériel |
| `lidar` | true | démarre rplidar_ros |
| `hardware_revision` | v1 | sélectionne `alidade_<rev>.yaml` |
| `pico_port` | /dev/pico | port série du Pico |
| `use_sim_time` | false | ne jamais le passer à true sur le robot réel |

`sim.launch.py` prend les mêmes `slam`, `nav`, `hardware_revision`, plus
`headless` et `world`. La même commande produit le même comportement des deux
côtés : un seul jeu de fichiers de configuration, rendu au lancement.

## Poste de développement

Ubuntu 26.04 ne propose que la distribution *lyrical* ; rviz et Gazebo
tournent donc en Jazzy dans Docker, la même distribution que le Pi.

```bash
./alidade_viz/docker/run.sh build        # image alidade:jazzy (une fois)
./alidade_viz/docker/run.sh up           # conteneur + colcon build
./alidade_viz/docker/run.sh rviz nav     # face au robot réel
./alidade_viz/docker/run.sh sim nav:=true               # simulation, terminal 1
./alidade_viz/docker/run.sh rviz nav use_sim_time:=true # terminal 2
./alidade_viz/docker/run.sh teleop
./alidade_viz/docker/run.sh test
```

Le conteneur est en `--net=host` et source `system/env/alidade_env.sh`
comme le Pi.

`ros2 run alidade_sim measure_odom.py` compare l'odométrie à la vérité
terrain lue avec `gz model -m alidade -p` : −0.1 % en ligne droite, −2.1 % en
rotation (le disque de collision de 4 mm des roues explique le résidu en
rotation). Le robot doit avoir de l'espace devant lui : contre un mur les
roues patinent et la mesure ne veut plus rien dire.

## Multi-machines : CycloneDDS avec pairs déclarés

Les deux machines partagent `ROS_DOMAIN_ID` (7) et `rmw_cyclonedds_cpp`, via
`system/env/alidade_env.sh` et `system/dds/cyclonedds.xml`.

Choix retenu : **CycloneDDS, multicast coupé, pairs déclarés en dur**. La
découverte multicast SPDP est perdue ou filtrée par beaucoup de points
d'accès WiFi, et les nœuds se voient un coup sur deux. L'alternative, le
Discovery Server de FastDDS, demande un processus serveur à démarrer avant
les autres et la variable `ROS_SUPER_CLIENT` pour que `ros2 topic` voie
quelque chose ; la configuration Cyclone est symétrique et sans serveur.

Chaque machine cite l'autre par `ALIDADE_ROBOT_HOST` (défaut `alidade.local`)
et `ALIDADE_DEV_HOST` (défaut `<hostname>.local`, résolu par avahi, installé
par `install.sh`). Une machine peut se citer elle-même sans dommage. Sur le
Pi, renseigner le nom du poste dans `/etc/default/alidade` ou exporter la
variable avant de lancer ; sur le poste, `run.sh` résout le nom du robot en
adresse IP avant d'entrer dans le conteneur, qui ne sait pas résoudre le mDNS.

Si Cyclone choisit la mauvaise interface (docker0, VPN), forcer `wlan0` dans
la section `General/Interfaces` de `cyclonedds.xml`.

## Révisions matérielles

La version vit à trois endroits, et nulle part ailleurs : le champ
`<version>` des `package.xml` et les tags git ; le fichier de révision
`alidade_description/config/alidade_v<n>.yaml` ; cette section. Aucun numéro
de version dans les noms de paquets, de repères TF ou de topics.

| Révision | Date | Contenu |
|---|---|---|
| v1 | 2026-09 | GA25-370 24 V 170 tr/min, réducteur 34:1, encodeurs 11 PPR (1496 counts/tour mesurés), roues Ø 64 mm, entraxe 0.1910 m, RPLIDAR C1, BNO055 en mode IMU, châssis 1.598 kg |

Une révision aux roues différentes est un second YAML, sélectionné par
`hardware_revision:=v2` ; le build produit un URDF corrigé par révision.

### Chaîne de description

`urdf/onshape/alidade.urdf` est l'export Onshape brut, réexportable tel quel.
À chaque build, `scripts/fix_urdf.py` rejoue ce que l'exporteur ne sait pas
faire : racine `root → base_footprint`, joints homonymes de links (URDF le
tolère, SDF non), masses et inerties réelles (l'export sort à 1 kg/m³
uniforme, donc chaque tenseur est juste à un facteur près), primitives de
collision, URI `package://`. Il vérifie le centre de masse (y = −0.1 mm) et
les poses des repères principaux contre la révision, et fait échouer le build
au-delà d'un millimètre.

Le xacro `alidade.urdf.xacro` inclut cet URDF et ajoute ce qui dépend de la
cible : plugin ros2_control (Pico, mock ou Gazebo), et en simulation le
LiDAR `gpu_lidar` aux valeurs de la datasheet du C1, l'IMU et les frictions.

`controllers.yaml`, `nav2.yaml`, `slam.yaml`, `rplidar.yaml` sont des gabarits
dont les marqueurs `${section.cle}` sont remplis au lancement depuis la
révision. Un marqueur non résolu fait échouer le lancement.

## Interface matérielle

`alidade_hardware/PicoSystem` est une `SystemInterface` ros2_control :

- un thread lit les trames `S` à 50 Hz, vérifie le checksum XOR, garde la
  dernière et compte les rejets ;
- `read()` convertit ticks → radians (2π / 1496) et l'IMU brute en SI
  (100 LSB/(m/s²), 16 LSB/(°/s)) ; pas d'orientation dans la trame, le
  broadcaster publie une covariance d'orientation à −1 ;
- `write()` envoie `v <l> <r>` à **chaque** cycle, y compris nul, pour tenir
  le chien de garde de 300 ms du firmware ; les consignes sont bornées au
  régime nominal du moteur, le firmware n'appliquant aucune limite ;
- `on_deactivate()` envoie `s` ;
- une trame absente depuis 500 ms ou un port disparu (USB débranché) fait
  passer le composant en erreur : les contrôleurs sont désactivés, le
  processus survit, et `/diagnostics` porte l'état du lien. Reprise :
  `sudo systemctl restart alidade`, ou à la main
  `ros2 control set_hardware_component_state AlidadeSystem active` puis
  `ros2 control switch_controllers --activate joint_state_broadcaster diff_drive_controller imu_sensor_broadcaster`.

Tester sans matériel : `faux_pico.py` rejoue le firmware (parseur strict,
PID identifié sur le robot, chien de garde) sur un pseudo-terminal.

```bash
ros2 run alidade_hardware faux_pico.py --link /tmp/pico
ros2 launch alidade_bringup robot.launch.py lidar:=false pico_port:=/tmp/pico
```

Le controller_manager avertit qu'il ne peut pas prendre la priorité temps
réel (`Could not enable FIFO RT scheduling policy`) : sans conséquence sur
ce robot ; pour le faire taire, ajouter l'utilisateur au groupe `realtime`
avec la limite `rtprio` correspondante dans `/etc/security/limits.d/`.

## Pièges connus

- `diff_drive_controller` (Jazzy) n'accepte que du `TwistStamped` ; nav2 sort
  du `Twist`. `cmd_vel_stamper.py` fait le joint. `teleop_twist_keyboard`
  prend `-p stamped:=true`.
- `slam_toolbox` est un nœud lifecycle sans autostart : `slam.launch.py`
  envoie configure puis activate.
- nav2 est lancé à la main (six nœuds), pas via `navigation_launch.py` dont
  le `docking_server` et le `waypoint_follower` font tomber tout le bringup.
- Les spawners ont `--switch-timeout 60` : sinon un controller_manager lent
  laisse `joint_state_broadcaster` inactif, sans `/joint_states` ni TF des
  roues.
- Un `ros2 topic pub --once` vers `/goal_pose` se perd dans la découverte
  DDS : publier en répété, ou cliquer dans rviz.
- Le Pi 5 est livré sans pile RTC : au boot, l'horloge repart de l'heure du
  dernier arrêt et NTP la recale d'un coup quelques secondes plus tard, en
  plein bringup. Les nœuds déjà créés restent vivants mais disparaissent du
  graphe DDS (plus d'odométrie ni de TF, nav2 abandonne), reproductible dès
  que le robot est resté éteint un moment. Le service attend la synchro
  (60 s au plus) ; le vrai remède est une pile ML2020 sur le connecteur BAT
  et `dtparam=rtc_bbat_vchg=3000000` dans `/boot/firmware/config.txt`.
- `angle_compensate: true` dans `rplidar.yaml` fixe le nombre de points par
  tour à 360 (1°), vérifié sur le matériel : le pilote rééchantillonne à
  `int(1e6 / 200 µs / 10 / 360) = 1` × 360 quelle que soit la cadence.
  `slam_toolbox` rejette les scans dont la taille varie d'un tour à l'autre,
  donc `false` (≈ 500 points bruts, ± quelques-uns) n'est pas une option. La
  simulation, elle, produit les 500 points de `samples_per_rev`.
- `rplidar_ros` échoue au démarrage trois fois sur quatre (« Cannot start
  scan: '80008002' », timeout du SDK : le pilote lance `startScan` juste
  après la mise en route du moteur, sans attendre ni réessayer), à froid
  comme à chaud. Le `respawn` finit par passer, mais le premier scan arrive
  de 15 s à plus de deux minutes après le lancement (jusqu'à 14 échecs
  consécutifs observés). L'alimentation est hors de cause : le rail 5 V reste
  à 4,9 V pendant les tentatives. Conséquence sans garde-fou : la costmap
  globale de nav2 expire (60 s) en attendant le repère `map`, que
  `slam_toolbox` ne publie qu'après un premier scan, et le lifecycle manager
  abandonne tout le bringup sans jamais réessayer. D'où `wait_for_scan.py` :
  nav2 ne démarre qu'après le premier `/scan`. Ne jamais tuer le pilote sans
  lui laisser envoyer `Stop` (SIGKILL) : le suivant reste bloqué à la
  connexion tant que le C1 diffuse.
- Le BNO055 lâche dès que les moteurs tournent : le firmware du Pico met gyro
  et accéléromètre à zéro quand la lecture I2C échoue et publie ces zéros
  comme des mesures. Mesuré : 8 % de trames nulles au repos, puis une rafale
  de plusieurs centaines d'échecs consécutifs en mouvement, après laquelle le
  composant est déclaré absent et ne revient pas toujours (bus I2C bloqué).
  Rien ne consomme `/imu/data_raw` aujourd'hui ; ne pas y brancher de fusion
  avant d'avoir corrigé le firmware (récupération du bus, drapeau de validité
  dans la trame S).
- La C920 expose deux nœuds `video4linux` : la capture (`index` 0) et un flux
  de métadonnées UVC (`index` 1) qui n'est pas une caméra. La règle udev ne
  lie `/dev/camera` qu'au premier. Mise au point **fixe** obligatoire : en
  autofocus la C920 change ses intrinsèques en permanence.
- `ImageFlipNode` (`rotation_steps: 2`) diffuse lui-même un TF du repère
  d'entrée vers `output_frame_id`, et ne corrige pas `cx`/`cy`. D'où le
  montage : le repère CAO `camera_optical_frame` est retourné par `mount_rpy`
  pour décrire l'image brute, `camera_upright_optical_frame` est déclaré dans
  l'URDF avec la même rotation (le TF du nœud et celui de l'URDF concordent,
  pas de double parent), et la calibration se fait sur `/camera/image`.
- Gazebo : `empty.sdf` n'embarque ni `gz-sim-sensors-system` ni
  `gz-sim-imu-system` ; `arena.sdf` les déclare. Les roues ont un disque de
  collision de 4 mm, pas la largeur réelle du pneu, sinon l'entraxe
  efficace tombe de 13.8 %.

## Tests

```bash
colcon test && colcon test-result --verbose
```

`alidade_description` : xacro des trois cibles, `check_urdf`, absence de
noms partagés link/joint, constantes issues de la révision.
`alidade_bringup` : rendu de tous les gabarits. `alidade_hardware` : gtest du
protocole depuis le vecteur de checksum documenté et les formes invalides.
