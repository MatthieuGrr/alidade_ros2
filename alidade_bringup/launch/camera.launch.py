"""Camera C920 : acquisition et detection ArUco dans un conteneur.

Topics, sous /camera, aux noms par defaut du pilote :
    image_raw, camera_info          image droite, repere camera.frame (non
                                    rectifiee, d'ou raw) : le flux a consommer
    image_raw/compressed            JPEG reencode par image_transport, pour l'interface
et, avec aruco:=true :
    /aruco_detections               identifiants et poses des marqueurs vus
    /aruco_tracker/debug            image annotee, publiee seulement si on la regarde
    TF marker_<id>                  un repere par marqueur, sous celui de l'image
    /cube/dock_pose, /cube/markers  face retenue et pose d'arret (cube_detector.py)

Les noeuds sont composes ensemble pour que les images circulent en
intra-processus : une image 640x480 en rgb8 pese 900 ko, soit 13.8 Mo/s a
15 Hz qu'il serait absurde de serialiser entre deux processus de la meme
machine. Le conteneur est multi-thread, sinon la detection, plus lourde que
le reste, retarderait l'acquisition.

Sans /dev/camera (regle udev), le launch ne demarre rien et le dit : la camera
n'est pas vitale, on ne veut ni respawn en boucle ni service en echec.
"""
import os
import subprocess

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

from alidade_bringup import params


def v4l2_controls(cam):
    """Applique les commandes V4L2 qu'usb_cam 0.8 rate.

    Le pilote les regle sous leurs anciens noms (focus_auto, exposure_auto,
    white_balance_temperature_auto) que les noyaux 6.x ne connaissent plus :
    « unknown control 'focus_auto' », puis « Permission denied » sur
    focus_absolute puisque l'autofocus est reste actif. La camera faisait donc
    sa mise au point en continu, et aucune calibration n'aurait tenu. Deux
    appels : focus_absolute est refuse tant que l'autofocus tourne. usb_cam
    reecrit ensuite focus_absolute a la meme valeur, sans dommage.
    """
    controls = [('focus_automatic_continuous', int(cam['autofocus'])),
                ('power_line_frequency', {50: 1, 60: 2}[cam['power_line_hz']])]
    if not cam['autofocus']:
        controls.append(('focus_absolute', cam['focus']))
    errors = []
    for name, value in controls:
        out = subprocess.run(['v4l2-ctl', '-d', cam['device'], '-c', f'{name}={value}'],
                             capture_output=True, text=True)
        if out.returncode != 0 or out.stderr.strip():
            errors.append(f'{name}={value} : {out.stderr.strip() or out.returncode}')
    return errors


def setup(context):
    hw = params.load_revision(context.launch_configurations['hardware_revision'])
    use_sim_time = params.as_bool(context.launch_configurations['use_sim_time'])
    device = hw['camera']['device']
    if not os.path.exists(device):
        return [LogInfo(msg=f"camera : {device} absent, pas de camera (regle udev ? cable ?)")]

    aruco = params.as_bool(context.launch_configurations['aruco'])
    notes = [LogInfo(msg=f'camera : commande V4L2 refusee, {e}') for e in v4l2_controls(hw['camera'])]
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

    # AUCUN remappage : image_transport les applique de facon incoherente
    # (deux fois au nom de base, jamais aux topics de ses plugins), voir README.
    # Les noms par defaut, sous /camera, sont deja ceux qu'on veut.
    return notes + extra + [ComposableNodeContainer(
        name='camera_container', namespace='camera',
        package='rclcpp_components', executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='usb_cam', plugin='usb_cam::UsbCamNode', name='usb_cam',
                namespace='camera',
                parameters=[cfg['usb_cam']['ros__parameters']],
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
