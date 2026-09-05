"""rviz2 sur le poste de developpement.

    ros2 launch alidade_viz rviz.launch.py                  # vue robot (modele, TF, scan)
    ros2 launch alidade_viz rviz.launch.py view:=slam       # + carte
    ros2 launch alidade_viz rviz.launch.py view:=nav        # + costmap, plan ; outil « 2D Goal Pose »
    ros2 launch alidade_viz rviz.launch.py view:=nav use_sim_time:=true   # face a la simulation
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg = Path(get_package_share_directory('alidade_viz'))
    return LaunchDescription([
        DeclareLaunchArgument('view', default_value='robot', choices=['robot', 'slam', 'nav']),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['-d', PathJoinSubstitution([str(pkg / 'rviz'), [LaunchConfiguration('view'), '.rviz']])],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
    ])
