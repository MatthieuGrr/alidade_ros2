"""Pile complete du robot reel.

    ros2 launch alidade_bringup robot.launch.py                # base + LiDAR + IMU
    ros2 launch alidade_bringup robot.launch.py slam:=true     # + cartographie seule
    ros2 launch alidade_bringup robot.launch.py nav:=true      # + pile autonome et gestionnaire de modes
    ros2 launch alidade_bringup robot.launch.py nav:=true mode:=MAPPING
    ros2 launch alidade_bringup robot.launch.py mock:=true     # sans Pico (mock_components)

Ordre de demarrage : robot_state_publisher et controller_manager, puis les
trois controleurs l'un apres l'autre, puis (nav:=true) le SLAM, la
localisation et le gestionnaire de modes une fois l'odometrie disponible, et
nav2 une fois le premier scan recu -- nav2 restant en pause tant que le
gestionnaire n'a pas de repere map a lui donner. Le LiDAR et la camera
demarrent en parallele, ils ne dependent de rien -- mais tout le monde
depend du LiDAR.
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
    camera = params.as_bool(context.launch_configurations['camera'])
    aruco = context.launch_configurations['aruco']
    mode = context.launch_configurations['mode']
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

    chain, last_spawner = actions.controller_chain(use_sim_time, revision)

    launched = [robot_state_publisher, controller_manager, *chain,
                actions.include('telemetry.launch.py',
                                hardware_revision=revision, use_sim_time=use_sim_time)]

    if lidar:
        launched.append(actions.include('lidar.launch.py',
                                        hardware_revision=revision, use_sim_time=use_sim_time))
    if camera:
        # Se retire de lui-meme si /dev/camera n'existe pas.
        launched.append(actions.include('camera.launch.py', hardware_revision=revision,
                                        use_sim_time=use_sim_time, aruco=aruco))

    launched += actions.autonomy(revision, use_sim_time, last_spawner, slam=slam, nav=nav, mode=mode)
    return launched


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware_revision', default_value='v2',
                              description='selectionne alidade_<rev>.yaml'),
        DeclareLaunchArgument('mock', default_value='false',
                              description='mock_components a la place du Pico'),
        DeclareLaunchArgument('slam', default_value='false',
                              description='cartographie seule, slam_toolbox actif des le depart'),
        DeclareLaunchArgument('nav', default_value='false',
                              description='pile autonome complete : SLAM, AMCL, nav2 et gestionnaire de modes'),
        DeclareLaunchArgument('mode', default_value='IDLE',
                              description='mode initial du gestionnaire (IDLE, MAPPING, LOCALIZATION...)'),
        DeclareLaunchArgument('lidar', default_value='true',
                              description='demarre sllidar_ros2'),
        DeclareLaunchArgument('camera', default_value='true',
                              description='demarre la RealSense si elle est branchee'),
        DeclareLaunchArgument('aruco', default_value='true',
                              description='detection des marqueurs ArUco (avec la camera)'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('pico_port', default_value='/dev/pico'),
        OpaqueFunction(function=setup),
    ])
