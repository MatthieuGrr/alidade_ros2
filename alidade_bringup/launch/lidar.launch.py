"""RPLIDAR C1 sur /dev/rplidar, publie /scan dans le repere 'laser'."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

from alidade_bringup import params


def setup(context):
    hw = params.load_revision(context.launch_configurations['hardware_revision'])
    use_sim_time = params.as_bool(context.launch_configurations['use_sim_time'])
    config = params.render_config('rplidar.yaml', hw, use_sim_time)
    return [Node(
        package='rplidar_ros', executable='rplidar_composition', name='rplidar',
        output='screen', parameters=[config],
        # Le pilote redemarre si le LiDAR decroche ; le service systemd fait
        # de meme pour le reste de la pile.
        respawn=True, respawn_delay=2.0,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware_revision', default_value='v1'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=setup),
    ])
