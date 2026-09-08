"""Pile nav2 minimale d'Alidade.

Volontairement ecrite a la main plutot que d'inclure
nav2_bringup/navigation_launch.py : celui-ci demarre en plus un docking_server
et un waypoint_follower, et son gestionnaire de cycle de vie abandonne le
bringup entier si l'un d'eux echoue a se configurer.

Chaine des commandes, la meme pour la navigation et le pilotage manuel :

    controller_server  --(cmd_vel_nav)--> velocity_smoother
    ou joystick (TELEOP)                  --(cmd_vel_smoothed)--> collision_monitor
                                          --(cmd_vel, Twist)--> cmd_vel_stamper
                                          --(TwistStamped)--> diff_drive_controller

Rien ne doit publier directement sur /cmd_vel : ce serait court-circuiter le
moniteur de collision, donc rouler sans garde-fou.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

from alidade_bringup import params

# Deux groupes, deux gestionnaires de cycle de vie. La chaine de securite
# protege TOUT mouvement, y compris le pilotage manuel : elle ne depend que de
# l'odometrie et du scan (le polygone du moniteur est statique, pas issu d'une
# costmap), donc elle demarre avec la pile et n'est jamais mise en pause. La
# planification, elle, a besoin de map -> odom et n'a de sens qu'en navigation :
# c'est le gestionnaire de modes qui l'active.
SAFETY = [
    'velocity_smoother',
    'collision_monitor',
]
NAVIGATION = [
    'controller_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
]


def setup(context):
    hw = params.load_revision(context.launch_configurations['hardware_revision'])
    use_sim_time = params.as_bool(context.launch_configurations['use_sim_time'])
    autostart = params.as_bool(context.launch_configurations['autostart'])
    config = params.render_config('nav2.yaml', hw, use_sim_time)

    # Les noeuds nav2 publient sur /tf absolu ; on les ramene au topic relatif
    # pour qu'ils restent dans le namespace courant.
    remap = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    common = {'output': 'screen', 'parameters': [config, {'use_sim_time': use_sim_time}]}

    nodes = [
        Node(package='nav2_controller', executable='controller_server',
             remappings=remap + [('cmd_vel', 'cmd_vel_nav')], **common),
        Node(package='nav2_planner', executable='planner_server', remappings=remap, **common),
        Node(package='nav2_behaviors', executable='behavior_server',
             remappings=remap + [('cmd_vel', 'cmd_vel_nav')], **common),
        Node(package='nav2_bt_navigator', executable='bt_navigator', remappings=remap, **common),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             remappings=remap + [('cmd_vel', 'cmd_vel_nav')], **common),
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             remappings=remap, **common),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_safety', output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                          'node_names': SAFETY}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             # Sans autostart, c'est le gestionnaire de modes qui appelle
             # STARTUP/PAUSE : la planification ne vit pas sans repere map.
             parameters=[{'use_sim_time': use_sim_time, 'autostart': autostart,
                          'node_names': NAVIGATION}]),
        # nav2 (Jazzy) sort du Twist ; diff_drive_controller n'accepte que du
        # TwistStamped, et aucun des deux n'expose de parametre pour ceder.
        Node(package='alidade_bringup', executable='cmd_vel_stamper.py', name='cmd_vel_stamper',
             output='screen',
             parameters=[{'input': '/cmd_vel', 'output': '/diff_drive_controller/cmd_vel',
                          'frame_id': 'base_footprint', 'use_sim_time': use_sim_time}]),
    ]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware_revision', default_value='v1'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='le lifecycle manager active nav2 seul ; false : gestionnaire de modes'),
        OpaqueFunction(function=setup),
    ])
