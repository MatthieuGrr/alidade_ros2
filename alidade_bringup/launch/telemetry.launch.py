"""Supervision : system_monitor (Pi, cadences capteurs) et diagnostic_aggregator.

Inclus par robot.launch.py et sim.launch.py, sans condition : l'etat du robot
doit exister avant toute interface qui l'affiche.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

from alidade_bringup import params


def setup(context):
    hw = params.load_revision(context.launch_configurations['hardware_revision'])
    use_sim_time = params.as_bool(context.launch_configurations['use_sim_time'])
    return [
        Node(
            package='alidade_bringup', executable='system_monitor.py', name='system_monitor',
            output='screen', parameters=[params.render_config('telemetry.yaml', hw, use_sim_time)],
        ),
        Node(
            package='diagnostic_aggregator', executable='aggregator_node', name='diagnostic_aggregator',
            output='screen', parameters=[params.render_config('diagnostics.yaml', hw, use_sim_time)],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware_revision', default_value='v2'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=setup),
    ])
