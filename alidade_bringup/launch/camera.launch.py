"""Camera C920 : usb_cam et ImageFlipNode dans un meme conteneur.

Topics, sous /camera :
    raw/image, raw/camera_info      image brute, a l'envers, repere optique physique
    raw/image/compressed            le MJPEG de la camera tel quel (pas de reencodage)
    image, camera_info              image droite, repere camera.upright_frame :
                                    c'est le flux a consommer (interface, detecteurs)

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

    rendered = params.render_config('camera.yaml', hw, use_sim_time)
    with open(rendered) as f:
        cfg = yaml.safe_load(f)

    # Les deux noeuds vivent dans /camera/raw, ou les noms par defaut tombent
    # deja juste : une seule regle de remappage chacun, et surtout aucune dont
    # la cible soit la source d'une autre. image_transport resout le nom de
    # base AVANT de creer ses publications, si bien qu'une regle s'applique
    # DEUX fois : avec des regles qui se repondaient, le noeud de retournement
    # republiait sa sortie sur sa propre entree (boucle, et /camera/image vide).
    return [ComposableNodeContainer(
        name='camera_container', namespace='camera',
        package='rclcpp_components', executable='component_container',
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
        ],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware_revision', default_value='v1'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=setup),
    ])
