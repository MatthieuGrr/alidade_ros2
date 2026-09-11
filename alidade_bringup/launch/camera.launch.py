"""Camera C920 : acquisition et detection ArUco dans un conteneur.

Topics, sous /camera :
    image, camera_info              image droite, repere camera.frame : c'est
                                    le flux a consommer (interface, detecteurs)
    image/compressed                JPEG reencode par image_transport, pour l'interface
et, avec aruco:=true :
    /aruco_detections               identifiants et poses des marqueurs vus
    /aruco_tracker/debug            image annotee, publiee seulement si on la regarde
    TF marker_<id>                  un repere par marqueur, sous celui de l'image

Les noeuds sont composes ensemble pour que les images circulent en
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

    # Une seule regle de remappage, et qui ne se compose avec aucune autre :
    # image_transport resout le nom de base AVANT de creer ses publications, si
    # bien qu'une regle s'y applique DEUX fois (voir README). Le camera_info
    # est deduit par image_transport a cote de l'image : /camera/camera_info.
    return [ComposableNodeContainer(
        name='camera_container', namespace='camera',
        package='rclcpp_components', executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='usb_cam', plugin='usb_cam::UsbCamNode', name='usb_cam',
                namespace='camera',
                parameters=[cfg['usb_cam']['ros__parameters']],
                remappings=[('image_raw', 'image')],
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
