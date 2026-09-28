"""Affiche le modele dans rviz2 avec les sliders de joints, pour un controle visuel.

    ros2 launch alidade_description display.launch.py
    ros2 launch alidade_description display.launch.py hardware_revision:=v2 target:=sim
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = Path(get_package_share_directory('alidade_description'))

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', str(pkg / 'urdf' / 'alidade.urdf.xacro'),
            ' hardware_revision:=', LaunchConfiguration('hardware_revision'),
            ' target:=', LaunchConfiguration('target'),
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument('hardware_revision', default_value='v2'),
        DeclareLaunchArgument('target', default_value='real'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             output='screen', condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='rviz2', executable='rviz2', output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', str(pkg.parent / 'alidade_viz' / 'rviz' / 'robot.rviz')]),
    ])
