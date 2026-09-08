"""Localisation sur carte connue : map_server, amcl et map_saver_server.

Trois noeuds lifecycle, laisses 'unconfigured' : le gestionnaire de modes les
configure et les active quand un mode en a besoin (LOCALIZATION), et active
map_saver des le depart pour pouvoir sauvegarder la carte en cours de
cartographie. Ils partagent nav2.yaml avec la navigation.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import LifecycleNode

from alidade_bringup import params


def setup(context):
    hw = params.load_revision(context.launch_configurations['hardware_revision'])
    use_sim_time = params.as_bool(context.launch_configurations['use_sim_time'])
    config = params.render_config('nav2.yaml', hw, use_sim_time)
    remap = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    common = {'namespace': '', 'output': 'screen',
              'parameters': [config, {'use_sim_time': use_sim_time}], 'remappings': remap}
    return [
        LifecycleNode(package='nav2_map_server', executable='map_server', name='map_server', **common),
        LifecycleNode(package='nav2_amcl', executable='amcl', name='amcl', **common),
        LifecycleNode(package='nav2_map_server', executable='map_saver_server', name='map_saver', **common),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware_revision', default_value='v1'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=setup),
    ])
