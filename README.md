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
 │                                              └──▶ aruco_tracker ──/aruco_detections + TF
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
   donc on calibre ce qu'on consomme), mise au point fixe comme en marche.
   La mire se génère aux cotes exactes — damier plutôt que ChArUco, dont les
   marqueurs 4x4 seraient confondus avec les faces du cube :
   ```bash
   ./alidade_description/scripts/make_checkerboard.py mire.pdf   # A4 paysage, cases de 25 mm
   ```
   L'imprimer **à 100 %**, sans « ajuster à la page », vérifier au réglet que
   les 9 cases font bien 225 mm entre les deux traits, et la coller sur un
   support rigide : une feuille qui gondole fausse la distorsion sans que
   rien ne le signale. Puis :
   ```bash
   ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 \
       --no-service-check image:=/camera/image
   ```
   Le YAML produit remplace `alidade_description/config/camera_v1.yaml`
   (provisoire : focale nominale, sans distorsion) ; reporter ensuite
   `2·atan(w/2fx)` dans `camera.sim_horizontal_fov` pour que Gazebo voie
   la même ouverture.

   **Marqueurs ArUco** (`aruco:=true`, actif par défaut avec la caméra). Le
   détecteur est composé dans le même conteneur que la caméra, donc les
   images ne sont jamais sérialisées. Cible : un cube de 50 mm portant sur
   chaque face un marqueur `DICT_4X4_50` de 40 mm (côté du carré noir),
   numéroté de 1 à 6.
   ```bash
   ros2 run alidade_bringup aruco_bench.py watch      # identifiants, cadence, distances
   ros2 run alidade_bringup aruco_bench.py sweep      # portée et justesse, sans robot ni caméra
   ros2 run alidade_bringup aruco_bench.py cube 0.40 35 20   # rend le cube, vérifie sa géométrie
   ros2 run rqt_image_view rqt_image_view /aruco_tracker/debug   # marqueurs entourés
   ```
   La **détection** ne dépend pas de la calibration, seulement les **poses** :
   tant que `camera_v1.yaml` est nominal, `/aruco_detections` et les TF
   `marker_<id>` portent un biais d'échelle. Portée mesurée sur images
   parfaites : 0,15 à 1,5 m, la limite basse venant du champ de vision et la
   haute d'un côté apparent tombé à 14 px. Sur un cube posé au sol, la
   caméra étant fixe et horizontale à 11,5 cm, le cube sort du champ par le
   bas quand l'avant du robot est à moins de ~23 cm : l'approche finale se
   fera à l'odométrie.

   **Vérifier que le support de caméra est d'aplomb.** L'URDF le suppose
   horizontal, et une inclinaison ne se voit pas à l'image : elle se traduit
   par une erreur de hauteur sur tout ce qu'on détecte. Poser le cube au sol
   à ~50 cm, puis :
   ```bash
   ros2 run tf2_ros tf2_echo base_footprint marker_<id>
   ```
   La composante z doit valoir la demi-arête du cube, soit **25 mm**. Mesuré
   sur le robot : 62 mm avec un support légèrement incliné, 21 mm une fois
   redressé — et la stabilité de l'orientation passe au même moment d'une
   amplitude de 20° à 3°. Un millimètre d'erreur à 43 cm vaut 0,13° de
   dévers, la mesure est donc bien plus sensible que l'œil.

   La géométrie du cube est dans `alidade_bringup/config/cube.yaml`, décrite
   face par face (normale sortante et direction du haut du motif) plutôt que
   par des matrices, pour se relire sur le cube posé devant soi ;
   `test_cube.py` vérifie que chaque face définit bien un repère orthonormé
   direct et que les six forment un cube. Précision de la pose du cube sur
   images parfaites, par PnP conjoint sur toutes les faces vues : 2 mm à
   0,40 m, 15 mm à 0,70 m, 44 mm à 1,00 m. Pour saisir un cube de 50 mm, il
   faut donc s'en approcher à 40 cm ou moins avant de conclure à l'odométrie.
   Puis un mètre au sol :
   ```bash
   ros2 topic pub -r 20 /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
       '{twist: {linear: {x: 0.2}}}'
   ros2 topic echo /diff_drive_controller/odom --field pose.pose.position
   ```
   ou au clavier : `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r cmd_vel:=/diff_drive_controller/cmd_vel`.

3. **Cartographie seule** : `robot.launch.py slam:=true`, puis rviz sur le
   poste (`./alidade_viz/docker/run.sh rviz slam`).

4. **Pile autonome et modes** : `robot.launch.py nav:=true` charge tout
   (slam_toolbox, map_server, amcl, map_saver, nav2) sans rien activer, et
   le **gestionnaire de modes** décide. Il démarre en `IDLE` (ou
   `mode:=MAPPING`) et expose :
   ```bash
   ros2 topic echo /robot/state                                   # mode, localisation, nav2, santé
   ros2 service call /mode_manager/set_mode alidade_msgs/srv/SetMode "{mode: 2}"                    # MAPPING
   ros2 service call /mode_manager/save_map alidade_msgs/srv/SaveMap "{name: salon}"
   ros2 service call /mode_manager/list_maps alidade_msgs/srv/ListMaps "{}"
   ros2 service call /mode_manager/set_mode alidade_msgs/srv/SetMode "{mode: 3, map_name: salon}"   # LOCALIZATION
   ros2 service call /mode_manager/set_mode alidade_msgs/srv/SetMode "{mode: 0}"                    # IDLE
   ```
   **Pilotage manuel** : le joystick publie sur `/cmd_vel_nav`, jamais sur
   `/cmd_vel`. Toute commande passe alors par la même chaîne que la
   navigation, lissage puis moniteur de collision. Ces deux nœuds forment un
   groupe de cycle de vie séparé (`lifecycle_manager_safety`) qui démarre avec
   la pile et n'est **jamais** mis en pause : sans cela, `TELEOP` mettait tout
   nav2 en pause et plus personne n'écoutait `/cmd_vel_smoothed`.

   Modes : `IDLE` 0, `TELEOP` 1, `MAPPING` 2 (slam_toolbox), `LOCALIZATION` 3
   (amcl sur une carte de `~/alidade_maps/<nom>/`, pose initiale à donner
   sur `/initialpose`), `EXPLORATION` 4 (cartographie plus l'explorateur de
   frontières [frontier_exploration_ros2](https://github.com/mertgulerx/frontier_exploration_ros2),
   cloné par `system/sources.sh`, réglé dans `config/exploration.yaml` : il
   pousse ses buts nav2, publie `/exploration_complete` quand il n'y a plus
   de frontière et rentre au point de départ ; le gestionnaire sauvegarde
   alors la carte sous `exploration_<date>` et repasse en `MAPPING`) et
   `DOCKING` 5 (réservé). `TELEOP` **préserve** la source de localisation en
   cours : conduire à la main au milieu d'une cartographie puis y revenir
   reprend la même carte, et l'exploration repart où elle en était. Une seule
   source de `map → odom` à la fois, et nav2 ne tourne que quand ce repère
   existe dans un mode qui navigue : sans cette règle sa costmap globale
   expire à l'activation et son lifecycle manager abandonne pour de bon.
   La sauvegarde écrit `map.pgm` + `map.yaml` (map_server) et le posegraph
   de slam_toolbox. Un but posé sur un obstacle échoue en silence côté rviz ;
   le message est dans le journal du `planner_server` (« Failed to create
   plan »).

5. **Interface web** (avec `nav:=true`) : `http://alidade.local:5000`, depuis
   un téléphone ou le poste. C'est [Vizanti](https://github.com/MoffKalast/vizanti)
   (branche ros2, cloné à un commit épinglé par `system/sources.sh` à côté
   d'`alidade_ros2`, servi par le robot, rosbridge sur le port 5001) avec un
   profil opérateur (`alidade_ui/config/operator.json`) : widget **Alidade**
   (mode et sous-état, boutons de mode, choix et sauvegarde de carte,
   localisation, nav2, capteurs, santé), carte, scan, TF, pose initiale,
   but, joystick sur `/cmd_vel_smoothed`, caméra, diagnostics. Le widget
   Alidade vit dans `alidade_ui/templates/alidade/` ; `sources.sh` le lie
   dans le dossier `public` de Vizanti, seul endroit où le serveur cherche
   ses templates.

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
  envoie configure puis activate, et **ré-active à chaque passage par
  `inactive`** ; avec `autostart:=false` (pile `nav:=true`) c'est le
  gestionnaire de modes qui le pilote, et désactivé il cesse bien de publier
  `map → odom` et `/map` (vérifié : 0 message en 5 s).
- rosbridge coupe tout appel de service à 5 s : les services du gestionnaire
  de modes répondent donc dès que la demande est acceptée (`set_mode`,
  `save_map`) et la suite se lit dans `/robot/state` (`transitioning`,
  `substate`). Vizanti découvre ses templates par un `os.walk` qui ne suit
  pas les liens de dossiers : `sources.sh` crée un vrai dossier par widget et
  y lie chaque fichier.
- nav2 après une pause se relance par `RESUME`, pas `STARTUP` : ce dernier
  tente de reconfigurer des nœuds déjà configurés (« Unable to start
  transition 1 from current state inactive »).
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
- **`image_transport` applique les remappages DEUX FOIS** : il résout le nom
  de base lui-même, puis `create_publisher` le résout encore. Une règle dont
  la cible est la source d'une autre se mord la queue — avec
  `rotated/image → image` et `image → raw/image`, le nœud de retournement
  republiait sa sortie sur sa propre entrée : `/camera/image` restait vide, la
  boucle coûtait un demi-cœur et sa gigue faisait passer la périodicité du
  `controller_manager` de 2 à 13 Hz d'écart-type. D'où `camera.launch.py` :
  les deux nœuds vivent dans `/camera/raw`, où les noms par défaut tombent
  juste, et une seule règle chacun.
- Les nœuds `image_transport` sont **paresseux** : sans abonné sur leurs
  images, ils n'émettent plus rien, pas même leur `camera_info`. `/camera/image`
  muet est donc normal quand personne ne regarde. La supervision surveille
  `/camera/raw/camera_info` (le pilote, lui, publie en continu) pour ne pas
  forcer l'encodage JPEG en permanence.
- Ne jamais compter des messages en s'abonnant à un flux lourd : une image
  640×480 en `rgb8` fait 900 ko, que CycloneDDS fragmente en centaines de
  datagrammes. En `best_effort`, un fragment perdu jette le message entier, et
  le compteur ne voyait **rien** passer alors que le flux tournait.
- La pose d'un marqueur plan **seul** est ambiguë : deux solutions PnP
  presque équivalentes, entre lesquelles le détecteur bascule d'une image à
  l'autre. Mesuré sur le cube immobile à 43 cm : distance stable à ±1,3 mm,
  mais orientation oscillant sur 3° (et sur 20° quand le support de caméra
  était de travers). La position d'un marqueur isolé est donc exploitable,
  son orientation non — c'est l'argument pour combiner plusieurs faces.
- La détection est très sensible au **contre-jour**. Face à une baie vitrée,
  l'exposition automatique se règle sur la fenêtre et sous-expose la pièce :
  taux de détection tombé à 23 % puis à zéro quand le soleil a tourné, contre
  100 % après avoir simplement déplacé le robot dans une zone à éclairage
  homogène. La compensation de contre-jour de la C920 **aggrave** les choses
  (3,8 %). Déplacer le robot vaut mieux que régler la caméra.
- `aruco_opencv` ne sait décrire que des **damiers plans** (`GridBoard` :
  `markers_x`, `markers_y`, `separation`), et son message `MarkerPose` ne
  porte qu'un identifiant et une pose, pas les coins détectés. Un cube, dont
  les six marqueurs sont sur six plans, n'est donc pas descriptible : la pose
  du cube devra venir de notre code, en combinant les faces visibles.
- Un marqueur ArUco a besoin d'une **marge blanche** d'au moins un module
  autour de lui. Trop mince sur un fond contrasté, cette marge forme son
  propre contour et le détecteur s'y accroche au lieu du carré noir — le
  marqueur devient alors invisible **de près** et détecté de loin, ce qui est
  contre-intuitif et coûte cher à diagnostiquer.
- Le détecteur ne raffine pas les coins par défaut
  (`cornerRefinementMethod: NONE`). Sur un marqueur de 40 mm vu à 1 m,
  soit 21 px de côté, un pixel d'erreur fait 6 % de distance. Mesuré de 0,2 à
  1,5 m : erreur maximale 6,0 % au pixel contre 2,1 % en sous-pixellique,
  moyenne 2,3 % contre 0,9 %. D'où `cornerRefinementMethod: 1` dans
  `aruco.yaml`.
- Le Pico compte les ticks depuis **son** démarrage, pas depuis celui de ROS.
  Sans origine prise à l'activation, les positions de roue valaient plus de
  1500 rad dès la première lecture et `diff_drive_controller` démarrait son
  odométrie à plusieurs mètres et dizaines de degrés de l'origine.
  `PicoSystem::on_activate` mémorise donc les compteurs comme référence.
- `amcl` **garde son filtre** d'une désactivation à l'autre et republie
  aussitôt la pose de la session précédente, éventuellement issue d'une autre
  carte. Le gestionnaire le croyait localisé et lançait la planification
  dessus : il le redémarre donc complètement (nettoyage, configuration,
  activation) à chaque entrée en `LOCALIZATION`.
- nav2 rejette toute pose dont le quaternion s'écarte de la norme 1 de plus
  de 1e-4 (« Received initialpose message is malformed »). Un quaternion
  saisi à la main avec trois décimales suffit à se faire jeter : le calculer,
  ne pas l'arrondir.
- Une carte ne se construit qu'en **roulant**. Un LiDAR 360° qui tourne sur
  place ne découvre rien, et la carte reste identique — ce n'est pas une
  panne. Mesuré : 912 cellules connues à l'arrêt, 8363 après 1,6 m de
  trajet. Une carte trop pauvre ne permet pas à AMCL de converger ;
  au-dessus de ~400 cellules de mur, l'écart-type tombe à 6 cm en un mètre.
- Le moniteur de collision utilise un polygone **`approach`** et non `stop` :
  il projette l'empreinte le long de la commande reçue et ne freine que si
  *cette* trajectoire touche quelque chose. Avec un `stop` circulaire, toute
  commande était coupée dès qu'un point entrait dans le rayon, quel que soit
  son sens — collé à une vitre que le LiDAR voit mal, le robot ne pouvait même
  plus reculer. Vérifié : bloqué en avant à 0,005 m/s, dégagement arrière à
  0,152 m/s. Une vitre reste invisible au LiDAR ; pour l'interdire vraiment il
  faut un masque `keepout` (filtre de costmap nav2), pas encore en place.
- Les seuils de diagnostic de `ros2_control` visent une boucle temps réel. Ici
  elle est cadencée par les trames USB du Pico : les moyennes sont excellentes
  (20 à 83 µs d'exécution pour 20 ms de cycle, périodicité 50,06 Hz ± 1,99),
  mais de rares pics de 6 ms au démarrage gonflent définitivement des
  statistiques cumulées. Seuls les écarts-types sont élargis dans
  `controllers.yaml`, jamais ceux des moyennes.
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
