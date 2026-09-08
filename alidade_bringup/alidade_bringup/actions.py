"""Briques de launch partagees par le robot reel et la simulation.

Les deux cibles enchainent exactement les memes controleurs, le meme SLAM et
la meme navigation ; seul le controller_manager change d'hote (processus
ros2_control_node sur le robot, plugin Gazebo en simulation).
"""
from __future__ import annotations

from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from alidade_bringup.params import bringup_share

# Dans l'ordre de demarrage. Les arguments ROS d'un controleur passent par le
# spawner : c'est ainsi que l'IMU sort sur le topic conventionnel /imu/data_raw
# plutot que sur /imu_sensor_broadcaster/imu.
CONTROLLERS = [
    ('joint_state_broadcaster', []),
    ('diff_drive_controller', []),
    ('imu_sensor_broadcaster', ['-r /imu_sensor_broadcaster/imu:=/imu/data_raw']),
]


def spawner(name: str, use_sim_time: bool, controller_ros_args: list | None = None) -> Node:
    """Un spawner de controleur avec des delais larges.

    --switch-timeout est le point critique. Le basculement vers 'active' est
    traite dans la boucle du controller_manager ; si celui-ci demarre lentement
    (Gazebo en rendu logiciel, Pi charge) les 5 s par defaut ne suffisent pas et
    le spawner sort en erreur. Le maillon suivant s'enchainant sur la fin du
    process, succes ou echec, on se retrouvait avec diff_drive_controller actif
    mais joint_state_broadcaster inactif : plus de /joint_states, plus de TF
    des roues, et rviz qui affiche « No transform from wheel_*_link ».
    """
    arguments = [name,
                 '--controller-manager', '/controller_manager',
                 '--controller-manager-timeout', '60',
                 '--switch-timeout', '60']
    for arg in controller_ros_args or []:
        arguments += ['--controller-ros-args', arg]
    return Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=arguments,
    )


def controller_chain(use_sim_time: bool) -> tuple[list, Node]:
    """Spawners enchaines dans l'ordre ; retourne les actions et le dernier
    spawner, sur la fin duquel on peut accrocher la suite."""
    spawners = [spawner(name, use_sim_time, ros_args) for name, ros_args in CONTROLLERS]
    actions = [spawners[0]]
    for previous, following in zip(spawners, spawners[1:]):
        actions.append(RegisterEventHandler(
            OnProcessExit(target_action=previous, on_exit=[following])))
    return actions, spawners[-1]


def scan_gate(use_sim_time: bool) -> Node:
    """Processus qui se termine au premier /scan (ou apres dix minutes).

    nav2 s'enchaine sur sa fin : sans scan, slam_toolbox ne publie pas le
    repere map, la costmap globale expire a l'activation et le gestionnaire
    de cycle de vie abandonne le bringup sans jamais reessayer. Le LiDAR reel
    met de 15 s a plus de deux minutes a demarrer (voir wait_for_scan.py).
    """
    return Node(
        package='alidade_bringup', executable='wait_for_scan.py', name='wait_for_scan',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'topic': '/scan', 'timeout': 600.0}],
    )


def mode_manager(use_sim_time: bool, mode: str) -> Node:
    return Node(
        package='alidade_bringup', executable='mode_manager.py', name='mode_manager',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'initial_mode': mode}],
    )


def autonomy(revision: str, use_sim_time: bool, last_spawner, *, slam: bool, nav: bool, mode: str) -> list:
    """Ce qui vient apres les controleurs, identique sur le robot et en simulation.

    slam seul : slam_toolbox autonome, actif des le depart (cartographie rapide,
    sans gestionnaire). nav : tout est charge mais inactif -- slam_toolbox,
    map_server, amcl, map_saver -- et le gestionnaire de modes decide ; nav2
    n'est lance qu'apres le premier scan et reste en pause tant qu'aucun
    repere map n'existe.
    """
    common = dict(hardware_revision=revision, use_sim_time=use_sim_time)
    launched = []
    if nav:
        launched.append(after(last_spawner, [
            include('slam.launch.py', autostart=False, **common),
            include('localization.launch.py', **common),
            mode_manager(use_sim_time, mode),
            # L'interface n'a de sens qu'avec le gestionnaire ; elle se
            # retire seule si Vizanti n'est pas dans le workspace.
            include('ui.launch.py'),
        ]))
        gate = scan_gate(use_sim_time)
        launched.append(after(last_spawner, [gate]))
        launched.append(after(gate, [include('nav.launch.py', autostart=False, **common)]))
    elif slam:
        launched.append(after(last_spawner, [include('slam.launch.py', **common)]))
    return launched


def include(launch_file: str, **arguments) -> IncludeLaunchDescription:
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup_share() / 'launch' / launch_file)),
        launch_arguments={k: str(v) for k, v in arguments.items()}.items(),
    )


def after(action, entities: list) -> RegisterEventHandler:
    return RegisterEventHandler(OnProcessExit(target_action=action, on_exit=entities))
