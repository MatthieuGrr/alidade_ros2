"""RPLIDAR C1 sur /dev/rplidar, publie /scan dans le repere 'laser'.

Pilote : sllidar_ros2, construit depuis les sources (system/sources.sh), car
aucun paquet apt ne sait piloter un C1 -- pas davantage sous le nom rplidar_ros.
config/sllidar.yaml explique pourquoi et ce que ca a coute.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

from alidade_bringup import params


def setup(context):
    hw = params.load_revision(context.launch_configurations['hardware_revision'])
    use_sim_time = params.as_bool(context.launch_configurations['use_sim_time'])
    config = params.render_config('sllidar.yaml', hw, use_sim_time)
    # Le nom du noeud doit valoir la cle de tete de sllidar.yaml, sinon les
    # parametres ne lui sont pas appliques et il demarre sur ses defauts
    # (/dev/ttyUSB0), sans rien signaler.
    return [Node(
        package='sllidar_ros2', executable='sllidar_node', name='sllidar',
        output='screen', parameters=[config],
        # Le pilote redemarre si le LiDAR decroche ; le service systemd fait
        # de meme pour le reste de la pile.
        respawn=True, respawn_delay=2.0,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware_revision', default_value='v2'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=setup),
    ])
