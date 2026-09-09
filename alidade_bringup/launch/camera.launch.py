"""Camera C920 : acquisition, redressement et detection ArUco dans un conteneur.

Topics, sous /camera :
    raw/image, raw/camera_info      image brute, a l'envers, repere optique physique
    raw/image/compressed            le MJPEG de la camera tel quel (pas de reencodage)
    image, camera_info              image droite, repere camera.upright_frame :
                                    c'est le flux a consommer (interface, detecteurs)
et, avec aruco:=true :
    /aruco_detections               identifiants et poses des marqueurs vus
    /aruco_tracker/debug            image annotee, publiee seulement si on la regarde
    TF marker_<id>                  un repere par marqueur, sous celui de l'image

Les trois noeuds sont composes ensemble pour que les images circulent en
intra-processus : une image 640x480 en rgb8 pese 900 ko, soit 13.8 Mo/s a
15 Hz qu'il serait absurde de serialiser entre deux processus de la meme
machine. Le conteneur est multi-thread, sinon la detection, plus lourde que
le reste, retarderait l'acquisition.

Sans /dev/camera (regle udev), le launch ne demarre rien et le dit : la camera
n'est pas vitale, on ne veut ni respawn en boucle ni service en echec.
"""
import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from alidade_bringup import params


def setup(context):
    hw = params.load_revision(context.launch_configurations['hardware_revision'])
    use_sim_time = params.as_bool(context.launch_configurations['use_sim_time'])
    device = hw['camera']['device']
    if not os.path.exists(device):
        return [LogInfo(msg=f"camera : {device} absent, pas de camera (regle udev ? cable ?)")]

    aruco = params.as_bool(context.launch_configurations['aruco'])
    with open(params.render_config('camera.yaml', hw, use_sim_time)) as f:
        cfg = yaml.safe_load(f)

    nodes = []
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

    # Les deux noeuds vivent dans /camera/raw, ou les noms par defaut tombent
    # deja juste : une seule regle de remappage chacun, et surtout aucune dont
    # la cible soit la source d'une autre. image_transport resout le nom de
    # base AVANT de creer ses publications, si bien qu'une regle s'applique
    # DEUX fois : avec des regles qui se repondaient, le noeud de retournement
    # republiait sa sortie sur sa propre entree (boucle, et /camera/image vide).
    return [ComposableNodeContainer(
        name='camera_container', namespace='camera',
        package='rclcpp_components', executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            # Aucun remappage : ses noms par defaut donnent deja
            # /camera/raw/image_raw (+ /compressed) et /camera/raw/camera_info.
            ComposableNode(
                package='usb_cam', plugin='usb_cam::UsbCamNode', name='usb_cam',
                namespace='camera/raw',
                parameters=[cfg['usb_cam']['ros__parameters']],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            # Lit ces memes topics, et sort sur /camera/image (+ /compressed)
            # et /camera/camera_info, ce dernier deduit par image_transport.
            ComposableNode(
                package='image_rotate', plugin='image_rotate::ImageFlipNode', name='image_flip',
                namespace='camera/raw',
                parameters=[cfg['image_flip']['ros__parameters']],
                remappings=[('image', 'image_raw'), ('rotated/image', '/camera/image')],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            *nodes,
        ],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware_revision', default_value='v1'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('aruco', default_value='true',
                              description='detection des marqueurs ArUco du cube'),
        OpaqueFunction(function=setup),
    ])
