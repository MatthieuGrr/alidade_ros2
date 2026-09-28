"""Camera Intel RealSense D435i : acquisition et detection ArUco dans un conteneur.

Topics, sous /camera, aux noms par defaut du pilote :
    color/image_raw, color/camera_info   flux couleur, repere camera.frame
    depth/image_rect_raw                 profondeur brute
    aligned_depth_to_color/image_raw     profondeur recalee sur la couleur
et, avec aruco:=true :
    /aruco_detections               identifiants et poses des marqueurs vus
    /aruco_tracker/debug            image annotee, publiee seulement si on la regarde
    TF marker_<id>                  un repere par marqueur, sous celui de l'image
    /cube/dock_pose, /cube/markers  face retenue et pose d'arret (cube_detector.py)

Les memes noms servent en simulation (voir alidade_description/urdf/gazebo.xacro
et le pont de alidade_sim) : le detecteur ne sait pas de quel cote il tourne.

Les noeuds sont composes ensemble pour que les images circulent en
intra-processus : une image 640x480 en rgb8 pese 900 ko, soit 13.8 Mo/s a
15 Hz qu'il serait absurde de serialiser entre deux processus de la meme
machine. Le conteneur est multi-thread, sinon la detection, plus lourde que
le reste, retarderait l'acquisition.

CE QUI A DISPARU AVEC LA C920, et qu'il est inutile de chercher :

  - les commandes V4L2. usb_cam 0.8 reglait l'autofocus et la frequence secteur
    sous des noms que les noyaux 6.x ne connaissent plus, et il fallait les
    rejouer a la main. librealsense parle a la camera par son propre protocole ;
    il n'y a ni autofocus ni exposition a rattraper.
  - le fichier de calibration. Le D435i sort ses intrinseques d'usine et le
    pilote publie un camera_info calibre.
  - la regle udev /dev/camera. Le capteur n'est pas un simple peripherique
    video : on le cherche sur le bus USB par son identifiant fabricant.

Sans camera branchee, le launch ne demarre rien et le dit : elle n'est pas
vitale, on ne veut ni respawn en boucle ni service en echec.
"""
import glob

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

from alidade_bringup import params

INTEL = '8086'          # identifiant fabricant Intel, commun a toute la gamme RealSense


def realsense_presente() -> bool:
    """Une RealSense est-elle enumeree sur le bus USB ?

    On lit /sys plutot que d'appeler lsusb : c'est le noyau qui repond, sans
    dependre d'un paquet (usbutils) qui peut ne pas etre installe sur le Pi.
    """
    for chemin in glob.glob('/sys/bus/usb/devices/*/idVendor'):
        try:
            if open(chemin).read().strip().lower() == INTEL:
                return True
        except OSError:
            continue
    return False


def setup(context):
    hw = params.load_revision(context.launch_configurations['hardware_revision'])
    use_sim_time = params.as_bool(context.launch_configurations['use_sim_time'])
    if not realsense_presente():
        return [LogInfo(msg='camera : aucune RealSense sur le bus USB '
                            '(cable ? alimentation ? port USB 3 ?)')]

    aruco = params.as_bool(context.launch_configurations['aruco'])
    with open(params.render_config('camera.yaml', hw, use_sim_time)) as f:
        cfg = yaml.safe_load(f)

    nodes, extra = [], []
    if aruco:
        with open(params.render_config('aruco.yaml', hw, use_sim_time)) as f:
            aruco_cfg = yaml.safe_load(f)
        # Variante « autostart » : un detecteur est un capteur, il n'a aucun
        # etat a piloter. La variante lifecycle existe et servira le jour ou
        # le gestionnaire de modes n'allumera la detection qu'en approche.
        nodes.append(ComposableNode(
            package='aruco_opencv', plugin='aruco_opencv::ArucoTrackerAutostart',
            name='aruco_tracker', namespace='',
            parameters=[aruco_cfg['aruco_tracker']['ros__parameters']],
            extra_arguments=[{'use_intra_process_comms': True}],
        ))
        # Hors du conteneur : il ne lit que les detections, quelques centaines
        # d'octets, et c'est du Python.
        extra.append(Node(package='alidade_bringup', executable='cube_detector.py',
                          output='screen',
                          parameters=[params.render_config('docking.yaml', hw, use_sim_time)]))

    # Le noeud porte le nom de la camera et vit a la racine : ses topics
    # tombent donc sous /<camera.name>/, soit /camera/color/image_raw. Le
    # meme `camera_name` est passe en parametre (camera.yaml) et a la macro
    # URDF (alidade.urdf.xacro) : les trois doivent coincider, sinon les
    # images et les TF decrivent deux capteurs differents sans rien signaler.
    return extra + [ComposableNodeContainer(
        name='camera_container', namespace='camera',
        package='rclcpp_components', executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                name=hw['camera']['name'], namespace='',
                parameters=[cfg['camera']['ros__parameters']],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            *nodes,
        ],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware_revision', default_value='v2'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('aruco', default_value='true',
                              description='detection des marqueurs ArUco du cube'),
        OpaqueFunction(function=setup),
    ])
