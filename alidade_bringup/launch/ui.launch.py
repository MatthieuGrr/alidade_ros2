"""Interface web : Vizanti (branche ros2, clone par system/sources.sh) servi
depuis le robot, avec sa passerelle rosbridge.

    http://<robot>:5000        interface (profil operateur par defaut)
    ws://<robot>:5001          rosbridge

Le profil operateur (alidade_ui/config/operator.json) prepose les widgets :
etat et modes Alidade, carte, scan, TF, pose initiale, but, joystick sur
/cmd_vel_smoothed (jamais /cmd_vel : le collision_monitor reste dans la
boucle), camera, diagnostics. Sans Vizanti dans le workspace, le launch le
dit et continue : l'interface n'est pas vitale.
"""
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def setup(context):
    try:
        vizanti = get_package_share_directory('vizanti_server')
    except PackageNotFoundError:
        return [LogInfo(msg='interface : vizanti_server absent (system/sources.sh puis colcon build), pas d interface')]
    layout = Path(get_package_share_directory('alidade_ui')) / 'config' / context.launch_configurations['layout']
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f'{vizanti}/launch/vizanti_server.launch.py'),
        launch_arguments={
            'port': context.launch_configurations['port'],
            'port_rosbridge': context.launch_configurations['port_rosbridge'],
            'default_widget_config': str(layout),
        }.items(),
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='5000'),
        DeclareLaunchArgument('port_rosbridge', default_value='5001'),
        DeclareLaunchArgument('layout', default_value='operator.json',
                              description='profil de widgets dans alidade_ui/config'),
        OpaqueFunction(function=setup),
    ])
