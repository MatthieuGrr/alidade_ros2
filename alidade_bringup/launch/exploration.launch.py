"""Explorateur de frontieres (frontier_exploration_ros2), inactif au depart.

Le gestionnaire de modes le demarre et l'arrete par son service
/control_exploration (mode EXPLORATION) et ecoute /exploration_complete. Sans
le paquet dans le workspace (system/sources.sh), le launch le dit et
continue : EXPLORATION sera alors refuse par le gestionnaire.
"""
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch_ros.actions import Node

from alidade_bringup import params


def setup(context):
    try:
        get_package_share_directory('frontier_exploration_ros2')
    except PackageNotFoundError:
        return [LogInfo(msg='exploration : frontier_exploration_ros2 absent (system/sources.sh puis colcon build)')]
    hw = params.load_revision(context.launch_configurations['hardware_revision'])
    use_sim_time = params.as_bool(context.launch_configurations['use_sim_time'])
    return [Node(
        package='frontier_exploration_ros2', executable='frontier_explorer', name='frontier_explorer',
        output='screen', parameters=[params.render_config('exploration.yaml', hw, use_sim_time)],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware_revision', default_value='v2'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=setup),
    ])
