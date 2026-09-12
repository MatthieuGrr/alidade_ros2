"""Controleurs seuls, pour un demarrage a la main ou un test.

Le controller_manager doit deja tourner (ros2_control_node ou plugin Gazebo)
et publier /robot_description ; ce launch se contente d'enchainer les spawners.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from alidade_bringup import actions, params


def setup(context):
    use_sim_time = params.as_bool(context.launch_configurations['use_sim_time'])
    chain, _ = actions.controller_chain(use_sim_time, context.launch_configurations['hardware_revision'])
    return chain


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware_revision', default_value='v1'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=setup),
    ])
