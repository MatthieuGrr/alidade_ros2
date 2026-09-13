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
 │  ├ diff_drive_controller ──/diff_drive_controller/odom (roues)  └───────────────┘
 │  ├ joint_state_broadcaster ──/joint_states            │
 │  └ imu_sensor_broadcaster ──/imu/data_raw ─┐          │
 │ gyro_odom (cap du gyro, distance des roues)◀┘──/odom + odom→base_footprint─▶ TF
 │ nav2 (6 nœuds) ──Twist──▶ cmd_vel_stamper ──TwistStamped──▶ diff_drive_controller
 │ usb_cam ──/camera/image_raw + camera_info──▶ aruco_tracker ──/aruco_detections + TF
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
   ros2 topic hz /camera/camera_info    # 15 Hz, frame_id camera_optical_frame
   ros2 run tf2_ros tf2_echo base_footprint wheel_left_link
   ```
   L'état d'ensemble est dans `/diagnostics_agg`, regroupé par sous-système
   (Système, Pico, Capteurs, Navigation) ; sur le poste,
   `ros2 run rqt_robot_monitor rqt_robot_monitor` l'affiche en arbre. Les
   seuils (température, rail 5 V, WiFi, cadences) sont dans la section
   `telemetry` de la révision.

   La caméra est montée à l'endroit, inclinée vers le sol (25° en CAO,
   28,2° mesurés, voir plus bas) : `/camera/image_raw` sort du pilote et se
   consomme telle quelle. Sans `/dev/camera`, le launch le dit et continue
   sans caméra.

   **Mise au point et calibration de la caméra**, à refaire à chaque
   changement de support ou de mise au point : la focale de la C920 varie de
   8 % sur la course de mise au point, une calibration ne vaut que pour la
   valeur `camera.focus` avec laquelle elle a été faite.

   La mire se génère aux cotes exactes — damier plutôt que ChArUco, dont les
   marqueurs 4x4 seraient confondus avec les faces du cube :
   ```bash
   ./alidade_description/scripts/make_checkerboard.py mire.pdf   # A4 paysage, cases de 25 mm
   ```
   L'imprimer **à 100 %**, sans « ajuster à la page », vérifier au réglet que
   les 9 cases font bien 225 mm entre les deux traits, et la coller sur un
   support rigide : une feuille qui gondole fausse la distorsion sans que
   rien ne le signale.

   *Mise au point*, sur le robot, mire tenue immobile bien en face de la
   caméra (donc penchée de 25°), à deux distances qui encadrent l'usage :
   ```bash
   ros2 run alidade_bringup camera_bench.py focus 0 10 20 25 30 35 40 50 60
   ```
   Retenir la valeur qui reste la plus nette aux deux distances. Mesuré le
   11 sept. 2026 : optimum 35 à 30 cm, 25 à 51 cm, d'où `focus: 30`.

   *Calibration*, sur le poste : le calibrateur a besoin d'un écran, et
   l'image brute ne passe pas proprement en WiFi. On relaie donc le JPEG,
   décodé localement, avec une qualité montée à 95 le temps de l'opération :
   ```bash
   ros2 param set /camera/usb_cam .image_raw.compressed.jpeg_quality 95   # sur le robot
   ros2 run alidade_bringup camera_bench.py relay                         # poste, terminal 1
   ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 \
       --no-service-check --ros-args -r image:=/calib/image               # poste, terminal 2
   ```
   Couvrir toute l'image, **coins compris**, à plusieurs distances et avec
   des inclinaisons de 30 à 45°, en marquant un temps d'arrêt à chaque pose.
   Puis **CALIBRATE**, et **SAVE** (pas COMMIT, qui voudrait écrire dans le
   pilote) : l'archive `/tmp/calibrationdata.tar.gz` contient `ost.yaml` et
   les vues retenues. `ost.yaml` remplace
   `alidade_description/config/camera_v1.yaml`, en gardant
   `camera_name: alidade_c920` ; reporter ensuite `2·atan(w/2fx)` dans
   `camera.sim_horizontal_fov` pour que Gazebo voie la même ouverture.
   Résultat du 11 sept. 2026 : 0,32 px de reprojection sur 45 vues,
   fx = 644,5 px — et non les 530 nominaux, ni les 690 estimés auparavant
   en autofocus.

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
   caméra étant fixe, inclinée de 28,2° (mesurés) et son axe optique à
   10,3 cm du sol, le cube reste entier dans l'image de **8 à 42 cm** de la
   face avant du robot (focale calibrée, fy = 646 px) ; l'approche se
   travaille à 30 cm. Au-delà, il faut le chercher au LiDAR ou sur la
   carte ; en deçà, l'approche finale se fait à l'odométrie.

   **Vérifier que le support de caméra est conforme à la CAO.** L'URDF le
   suppose incliné de 25° pile et sans dévers, et un écart ne se voit pas à
   l'image : il se traduit par une erreur de hauteur sur tout ce qu'on
   détecte. Poser le cube au sol à ~30 cm, au milieu du champ, puis :
   ```bash
   ros2 run tf2_ros tf2_echo base_footprint marker_<id>
   ```
   La composante z doit valoir la demi-arête du cube, soit **25 mm**. Mesuré
   sur le robot : 62 mm avec un support légèrement incliné, 21 mm une fois
   redressé — et la stabilité de l'orientation passe au même moment d'une
   amplitude de 20° à 3°. Un millimètre d'erreur à 43 cm vaut 0,13° de
   dévers, la mesure est donc bien plus sensible que l'œil.

   Ce contrôle suppose la caméra **calibrée** : caméra inclinée, une erreur
   de focale fausse la profondeur le long de l'axe optique, et l'erreur se
   reporte sur z (72 mm lus avec la focale nominale). Caméra calibrée, le
   11 sept. 2026, cube à 30 cm de la face avant : 42,5 mm pour la face
   tournée vers le robot et 68,9 mm pour le dessus, soit 18 mm de trop sur
   les deux. Une caméra à 28,2° au lieu des 25° de la CAO explique les deux
   hauteurs et la distance à la fois ; la correction est dans
   `camera.mount_rpy` de la révision. Si on ne sait pas d'où vient l'écart,
   la hauteur mesurée donne directement l'angle à corriger.

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
   ros2 topic echo /odom --field pose.pose.position
   ```
   ou au clavier : `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r cmd_vel:=/diff_drive_controller/cmd_vel`.

   **Odométrie : la distance vient des roues, le cap du gyromètre.**
   `gyro_odom.py` publie `/odom` et le repère `odom → base_footprint` ;
   `diff_drive_controller` ne publie plus ce repère mais garde son odométrie
   roues, et comparer les deux montre ce que le gyromètre apporte. Le calcul
   est dans `alidade_bringup/odometry.py`, testé sans ROS. Sans échantillon
   frais — ou sur une trame de zéros, que le firmware envoie quand la lecture
   I²C échoue et qui est impossible sous gravité — le cap revient aux roues,
   et le nœud le dit dans le journal.

   **Calibrer l'odométrie**, après tout changement de pneus, de plancher, de
   firmware moteur ou d'IMU. Dégager un mètre autour du robot, puis :
   ```bash
   ros2 run alidade_bringup odom_calib.py rotate 2 0.6     # échelle en rotation
   ros2 run alidade_bringup odom_calib.py rotate -2 0.6    # et dans l'autre sens
   ros2 run alidade_bringup odom_calib.py straight 1.0     # cap et distance en ligne droite
   ```
   Le juge est le LiDAR contre la scène : corrélation circulaire des profils
   de scan pour les tours complets, ICP 2D pour la translation. Les résultats
   se reportent dans la révision : `kinematics` pour les roues,
   `imu.gyro_scale` pour le gyromètre. Mesuré le 12 sept. 2026, trois essais
   concordants à 0,1 % : roues 0,9975 à 1,0061, gyromètre **1,029** (d'où
   `gyro_scale: 0.9718`, l'erreur de sensibilité du BNO055, donnée à ±3 %),
   et **fusion 0,9989** après correction. En ligne droite, cap juste à 0,3°
   près pour la fusion, 0,4° pour les roues, sur 1 m.

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
   `DOCKING` 5 (approche du cube, voir plus bas). `TELEOP` et `DOCKING`
   **préservent** la source de localisation en cours : conduire à la main au milieu d'une cartographie puis y revenir
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
   localisation, nav2, capteurs, santé, et en DOCKING le bouton Dégager),
   carte, scan, TF, pose initiale, but, joystick sur `/cmd_vel_nav`,
   caméra, cube détecté (`/cube/markers`), diagnostics. Le widget
   Alidade vit dans `alidade_ui/templates/alidade/` ; `sources.sh` le lie
   dans le dossier `public` de Vizanti, seul endroit où le serveur cherche
   ses templates.

   **Docking sur le cube** (`DOCKING` 5) : amener le robot à portée, cube
   dans le champ et à moins de ~12° de l'axe, de 8 à 42 cm devant ; puis le
   bouton DOCKING du widget Alidade (ou `set_mode` 5). Refusé si le cube
   n'est pas vu. Le robot s'arrête à **10 cm** de la face du cube la plus
   tournée vers lui, dans son axe ; « Dégager » le fait reculer de 15 cm,
   et redemander DOCKING relance une approche. Deux nœuds :
   - `cube_detector.py` : pose du cube depuis les faces vues, et pose
     d'arrêt de `base_footprint` devant la face visée (`/cube/dock_pose`,
     `/cube/center`, marqueurs `/cube/markers` pour Vizanti : cube orange,
     rouge si ses faces se contredisent, flèche verte à la pose d'arrêt) ;
   - `cube_docking.py` : actions `/dock_robot` et `/undock_robot` (types
     `nav2_msgs`). Mise en place à l'odométrie dans l'axe de la face, 20 cm
     en retrait, si le robot en est trop loin ; puis approche asservie sur la
     vision jusqu'à la **ligne** d'arrêt ; contrôle de l'arrivée, reprise si
     hors tolérance (1,5 cm, 5°).

   Réglages dans `alidade_bringup/config/docking.yaml`, logique testée sans
   ROS dans `alidade_bringup/docking.py` et `cube.py`. Essais sur le robot
   le 11 sept. 2026, tous réussis du premier coup : départ dans l'axe,
   arrivée à 1 mm et 1,1° en 2,9 s ; départ à 9° et 3,4 cm, 2 mm et 1,0° en
   3,0 s ; départ décalé de 7,7 cm (mise en place), 8 mm et 4,2° en 8,5 s.
   Écart final mesuré par la caméra : 10,4 à 10,6 cm.

   Sans robot, `aruco_bench.py fake-cube <x> <y> <cap>` publie en simulation
   les détections d'un cube posé dans `odom`, avec la latence mesurée sur le
   robot : toute la chaîne se rejoue dans Gazebo.

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
./alidade_viz/docker/run.sh rviz nav use_sim_time:=true # terminal 2 (domaine de la simulation)
./alidade_viz/docker/run.sh teleop
./alidade_viz/docker/run.sh test
```

Le conteneur est en `--net=host` et source `system/env/alidade_env.sh`
comme le Pi. La **simulation**, et le rviz qui la regarde, tournent sur un
domaine DDS à part (17) sans le robot pour pair : sur le domaine du robot,
les commandes de vitesse simulées (nav2, docking) atteindraient ses moteurs
dès qu'il est allumé. Un script qui lance la simulation à la main doit
faire de même (`ROS_DOMAIN_ID=17 ALIDADE_ROBOT_HOST=127.0.0.1`).

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

**Réexporter depuis Onshape** : l'exportateur vide le dossier `urdf/` qu'on
lui désigne (les trois xacro écrits à la main disparaissent) et y écrit
`main_assembly.urdf`. Après chaque export :
```bash
cd alidade_description
git checkout HEAD -- urdf/alidade.urdf.xacro urdf/gazebo.xacro urdf/ros2_control.xacro
mv urdf/main_assembly.urdf urdf/onshape/alidade.urdf
touch urdf/onshape/alidade.urdf   # mv garde la date : sans ça le build ne relance pas fix_urdf
```
Exporter les maillages en résolution grossière : en fine, le même robot
pèse 116 Mo de STL (2,3 M triangles) contre 16 Mo, pour rien de visible
dans rviz.

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
- **usb_cam 0.8 règle la caméra sous des noms de commandes V4L2 périmés**
  (`focus_auto`, `exposure_auto`, `white_balance_temperature_auto`), que les
  noyaux 6.x ont renommés (`focus_automatic_continuous`, `auto_exposure`,
  `white_balance_automatic`). Le journal dit « unknown control 'focus_auto' »
  puis « Permission denied » sur `focus_absolute`, et la caméra reste en
  **autofocus** : c'était le cas jusqu'au 11 sept. 2026, `autofocus: false`
  n'avait jamais été appliqué. `camera.launch.py` pose donc lui-même
  l'autofocus, la mise au point et l'anti-scintillement (50 Hz en France,
  60 Hz d'usine) avant de démarrer le pilote. Les deux autres messages
  « unknown control » restent, sans effet : exposition et balance des blancs
  sont voulues automatiques, et c'est leur réglage d'usine. Vérifier avec
  `v4l2-ctl -d /dev/camera --get-ctrl=focus_automatic_continuous,focus_absolute`.
- **Ne pas remapper un nœud `image_transport`** : il applique les règles de
  façon incohérente. Au topic de base, DEUX FOIS : il résout le nom
  lui-même, puis `create_publisher` le résout encore, si bien qu'une règle
  dont la cible est la source d'une autre se mord la queue. Vécu du temps où
  la caméra était montée tête en bas : avec `rotated/image → image` et
  `image → raw/image`, le nœud de retournement republiait sa sortie sur sa
  propre entrée ; l'image restait vide, la boucle coûtait un demi-cœur et sa
  gigue faisait passer la périodicité du `controller_manager` de 2 à 13 Hz
  d'écart-type. Aux topics de ses plugins, JAMAIS : avec `image_raw → image`,
  l'image partait bien sur `/camera/image` mais le JPEG restait sur
  `/camera/image_raw/compressed`, et l'interface, abonnée à
  `/camera/image/compressed`, n'affichait plus rien. D'où `camera.launch.py`
  sans aucun remappage : les noms par défaut du pilote sous `/camera` sont
  les bons.
- En `mjpeg2rgb`, usb_cam décode le MJPEG de la C920 et `image_transport`
  réencode un JPEG pour `/camera/image_raw/compressed` : le flux compressé coûte
  du CPU, mais seulement quand quelqu'un le regarde. Le mode `mjpeg`
  republierait le flux de la caméra sans réencodage, mais sans image brute
  pour le détecteur.
- **L'accéléromètre du BNO055 n'est pas calibré et ne sert à rien tel quel.**
  Le firmware du Pico n'écrit aucun décalage (la calibration en six positions
  n'a jamais été faite, et l'état `CALIB_STAT` ne sort que sur l'UART du Pico,
  pas dans la trame). Mesuré, robot posé à plat et Pico redémarré au sol :
  gravité 9,606 m/s² au lieu de 9,807, roulis apparent 7,5° et tangage 4,2°
  alors que le niveau donne 1°. Reproductible. Seul le **gyromètre** est
  exploité, et lui est excellent : biais 0,005 °/s, soit 0,3° de dérive par
  minute, bruit 0,1 °/s, et zéro trou dans le flux sur trois essais de tours
  complets depuis le passage de l'I²C à 100 kHz.
- **Une lecture I²C ratée envoie des zéros**, pas un drapeau : vu du Pi, une
  IMU en panne ressemble à une IMU immobile. `odometry.usable()` rejette donc
  les six zéros exacts, `accel z` nul étant impossible sous gravité. Un vrai
  drapeau de validité dans la trame S reste à faire côté firmware.
- **Le LiDAR met de 15 s à 7 minutes à démarrer**, en enchaînant les
  « Cannot start scan: 80008002 » avant de réussir (18 à 38 échecs observés,
  respawn du pilote). Il répond pourtant (firmware, état de santé) et sa
  configuration n'a pas changé. Écarté : la caméra n'y est pour rien (même
  échec avec `camera:=false`). Tant que `/scan` n'arrive pas, `nav.launch`
  attend, donc ni chaîne de sécurité ni mouvement : c'est voulu, mais il faut
  le savoir. Piste restante : mécanique ou alimentation, depuis le nouveau
  support.
- **Débrancher le Pico arrête toute la pile**, par `BindsTo=dev-pico.device`
  dans le service : c'est la sécurité voulue, les moteurs se coupent. Mais
  l'arrêt est propre, donc `Restart=on-failure` ne relançait rien : d'où
  `WantedBy=dev-pico.device dev-rplidar.device`, qui fait repartir la pile au
  rebranchement. `systemctl reenable alidade` après mise à jour du service.
- Ne jamais compter des messages en s'abonnant à un flux lourd : une image
  640×480 en `rgb8` fait 900 ko, que CycloneDDS fragmente en centaines de
  datagrammes. En `best_effort`, un fragment perdu jette le message entier, et
  le compteur ne voyait **rien** passer alors que le flux tournait. La
  supervision surveille donc `/camera/camera_info`, quelques centaines
  d'octets publiés en continu par le pilote.
- **Les moteurs ne tiennent pas les basses vitesses.** Consigne tenue 2 s,
  mesuré le 11 sept. 2026 : 0,15 rad/s ne fait rien, 0,25 rad/s donne
  0,08 ; 2 cm/s donne 0,6 cm/s, et à 1,5 ou 4 cm/s une roue démarre avant
  l'autre (7 à 9° de dérive). Fiable à partir de 0,45 rad/s et 5 cm/s : ce
  sont les planchers du docking. Le vrai remède est dans le firmware du
  Pico (compensation de zone morte).
- **usb_cam date ses images d'environ 0,5 s trop tôt.** Âge apparent d'une
  détection à sa réception : 790 ms, très stable ; latence réelle, mesurée
  par une rotation sur place (instant où l'odométrie voit le mouvement, puis
  la caméra) : environ 300 ms. Tout ce qui recale une détection sur le
  mouvement du robot doit donc partir de l'heure de réception moins la
  latence mesurée (`detection_latency`), jamais de l'horodatage de l'image.
- Le cap d'une face de cube vue seule **bascule de ±8°** d'une image à
  l'autre, alors que son centre est stable à 1-2 mm. Une pose d'arrêt à
  25 cm du centre change ces 8° en 3 cm latéraux : le docking lisse donc le
  centre et le cap séparément (`docking.TargetFilter`), avec un seuil
  d'aberration large, car ce bruit est bimodal.
- La pose d'un marqueur plan **seul** est ambiguë : deux solutions PnP
  presque équivalentes, entre lesquelles le détecteur bascule d'une image à
  l'autre. Mesuré sur le cube immobile à 43 cm : distance stable à ±1,3 mm,
  mais orientation oscillant sur 3° (et sur 20° quand le support de caméra
  était de travers). La position d'un marqueur isolé est donc exploitable,
  son orientation non — c'est l'argument pour combiner plusieurs faces.
  **De loin, c'est pire, et `aruco_opencv` choisit le plus souvent la mauvaise
  solution** : cube vu en coin à 70 cm–1 m, marqueurs de 20 px, orientation
  fausse sur 80 % des images, avec des bascules de 60°. Les faces se
  contredisaient, et la face visée par le docking changeait sans cesse.
  Leurs **positions**, elles, restent justes au millimètre. D'où
  `cube.estimate_on_floor`, dans le repère du robot où le sol est connu :
  deux faces verticales voisines donnent le cap par le segment qui joint leurs
  centres ; une face et le dessus, par la direction du dessus vers la face ;
  une face seule n'est crue qu'à moins de 45 cm. Le dessus se reconnaît à la
  hauteur de son marqueur, pas à son orientation. Mesuré sur la même scène :
  cap de la pose d'arrêt passé de −52°/+52° (bascules) à une dispersion de
  8,6° sur une seule face, hystérésis de face portée à 30°.
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
