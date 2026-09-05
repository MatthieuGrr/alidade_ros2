"""Pile complete du robot reel.

    ros2 launch alidade_bringup robot.launch.py                # base + LiDAR + IMU
    ros2 launch alidade_bringup robot.launch.py slam:=true     # + cartographie
    ros2 launch alidade_bringup robot.launch.py nav:=true      # + navigation (implique slam)
    ros2 launch alidade_bringup robot.launch.py mock:=true     # sans Pico (mock_components)

Ordre de demarrage : robot_state_publisher et controller_manager, puis les
trois controleurs l'un apres l'autre, puis le SLAM et la navigation une fois
l'odometrie disponible. Le LiDAR demarre en parallele, il ne depend de rien.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

from alidade_bringup import actions, params


def setup(context):
    revision = context.launch_configurations['hardware_revision']
    mock = params.as_bool(context.launch_configurations['mock'])
    slam = params.as_bool(context.launch_configurations['slam'])
    nav = params.as_bool(context.launch_configurations['nav'])
    lidar = params.as_bool(context.launch_configurations['lidar'])
    use_sim_time = params.as_bool(context.launch_configurations['use_sim_time'])
    pico_port = context.launch_configurations['pico_port']

    hw = params.load_revision(revision)
    controllers_file = params.render_config('controllers.yaml', hw, use_sim_time)
    urdf = params.robot_description(revision, 'real', mock, pico_port)

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': urdf, 'use_sim_time': use_sim_time}],
    )

    # Le controller_manager recoit l'URDF par le topic /robot_description
    # (comportement Jazzy) : il n'a besoin que du YAML des controleurs.
    controller_manager = Node(
        package='controller_manager', executable='ros2_control_node', output='screen',
        parameters=[controllers_file, {'use_sim_time': use_sim_time}],
    )

    chain, last_spawner = actions.controller_chain(use_sim_time)

    launched = [robot_state_publisher, controller_manager, *chain]

    if lidar:
        launched.append(actions.include('lidar.launch.py',
                                        hardware_revision=revision, use_sim_time=use_sim_time))

    # nav2 a besoin de map -> odom : activer nav sans slam n'aurait pas de sens.
    followers = []
    if slam or nav:
        followers.append(actions.include('slam.launch.py',
                                         hardware_revision=revision, use_sim_time=use_sim_time))
    if nav:
        followers.append(actions.include('nav.launch.py',
                                         hardware_revision=revision, use_sim_time=use_sim_time))
    if followers:
        launched.append(actions.after(last_spawner, followers))

    return launched


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware_revision', default_value='v1',
                              description='selectionne alidade_<rev>.yaml'),
        DeclareLaunchArgument('mock', default_value='false',
                              description='mock_components a la place du Pico'),
        DeclareLaunchArgument('slam', default_value='false'),
        DeclareLaunchArgument('nav', default_value='false',
                              description='navigation nav2 (implique slam)'),
        DeclareLaunchArgument('lidar', default_value='true',
                              description='demarre rplidar_ros'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('pico_port', default_value='/dev/pico'),
        OpaqueFunction(function=setup),
    ])
