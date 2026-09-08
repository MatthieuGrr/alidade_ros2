"""Simulation Gazebo Harmonic d'Alidade, a parite avec le robot reel.

    ros2 launch alidade_sim sim.launch.py               # simulation seule
    ros2 launch alidade_sim sim.launch.py slam:=true    # + cartographie
    ros2 launch alidade_sim sim.launch.py nav:=true     # + navigation (implique slam)
    ros2 launch alidade_sim sim.launch.py headless:=true  # serveur seul, sans fenetre

C'est la meme pile que sur le robot : memes controleurs, meme controllers.yaml,
meme SLAM, meme nav2. Seul le plugin materiel change (gz_ros2_control), et
use_sim_time vaut true partout.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from alidade_bringup import actions, params


def setup(context):
    revision = context.launch_configurations['hardware_revision']
    slam = params.as_bool(context.launch_configurations['slam'])
    nav = params.as_bool(context.launch_configurations['nav'])
    mode = context.launch_configurations['mode']
    headless = params.as_bool(context.launch_configurations['headless'])
    world = context.launch_configurations['world']
    use_sim_time = True

    hw = params.load_revision(revision)
    # Le plugin gz_ros2_control lit le YAML des controleurs par chemin : on le
    # rend d'abord, puis on passe ce chemin au xacro.
    controllers_file = params.render_config('controllers.yaml', hw, use_sim_time)
    urdf = params.robot_description(revision, 'sim', controllers_file=controllers_file)

    gz_args = ['-r', '-v3']
    if headless:
        gz_args += ['-s', '--headless-rendering']
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory('ros_gz_sim')) / 'launch' / 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ' '.join(gz_args + [world])}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': urdf, 'use_sim_time': use_sim_time}],
    )

    # Pose le robot juste au-dessus du sol : il retombe de quelques millimetres
    # au premier pas, ce qui evite de le faire naitre en contact.
    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-name', 'alidade', '-topic', 'robot_description', '-z', '0.02'],
    )

    # Sans ce pont, le temps simule n'atteint jamais les noeuds ROS. L'IMU ne
    # passe pas ici : gz_ros2_control la lit et imu_sensor_broadcaster la publie,
    # comme sur le robot reel.
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # Camera simulee, deja droite : memes topics que la sortie du
            # noeud de retournement sur le robot reel.
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    chain, last_spawner = actions.controller_chain(use_sim_time)

    launched = [
        # Permet a Gazebo de resoudre les URI package:// des maillages.
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(params.description_share().parent)),
        gazebo, robot_state_publisher, bridge, spawn,
        # Le controller_manager n'existe qu'une fois le modele instancie dans
        # Gazebo : on enchaine donc les spawners sur la fin du spawn.
        actions.after(spawn, chain),
        actions.include('telemetry.launch.py', hardware_revision=revision, use_sim_time=use_sim_time),
    ]
    # Meme enchainement que sur le robot reel (voir actions.autonomy) : le
    # SLAM apres l'odometrie, nav2 apres le premier scan -- instantane ici.
    launched += actions.autonomy(revision, use_sim_time, last_spawner, slam=slam, nav=nav, mode=mode)
    return launched


def generate_launch_description():
    default_world = str(Path(get_package_share_directory('alidade_sim')) / 'worlds' / 'arena.sdf')
    return LaunchDescription([
        DeclareLaunchArgument('hardware_revision', default_value='v1'),
        DeclareLaunchArgument('slam', default_value='false'),
        DeclareLaunchArgument('nav', default_value='false',
                              description='pile autonome complete : SLAM, AMCL, nav2 et gestionnaire de modes'),
        DeclareLaunchArgument('mode', default_value='IDLE',
                              description='mode initial du gestionnaire (IDLE, MAPPING, LOCALIZATION...)'),
        DeclareLaunchArgument('headless', default_value='false', description='serveur Gazebo seul'),
        DeclareLaunchArgument('world', default_value=default_world,
                              description='empty.sdf ne contient pas gz-sim-sensors-system : le LiDAR y resterait muet'),
        OpaqueFunction(function=setup),
    ])
