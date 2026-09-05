"""slam_toolbox en mode cartographie, avec ses transitions de cycle de vie.

slam_toolbox est un noeud lifecycle sans parametre d'autostart : livre a
lui-meme il reste en 'unconfigured', sans souscription a /scan ni publication
de /map, et rien dans les logs ne le signale. On pilote donc explicitement
configure puis activate.
"""
import lifecycle_msgs.msg
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

from alidade_bringup import params


def setup(context):
    hw = params.load_revision(context.launch_configurations['hardware_revision'])
    use_sim_time = params.as_bool(context.launch_configurations['use_sim_time'])
    config = params.render_config('slam.yaml', hw, use_sim_time)

    slam = LifecycleNode(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', namespace='', output='screen',
        parameters=[config, {'use_sim_time': use_sim_time}],
    )

    def transition(transition_id):
        return EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(slam), transition_id=transition_id))

    activate_when_inactive = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam, goal_state='inactive',
        entities=[transition(lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE)],
    ))
    # Le handler est enregistre avant le noeud, sinon la transition vers
    # 'inactive' peut passer avant qu'on l'ecoute.
    return [activate_when_inactive, slam,
            transition(lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware_revision', default_value='v1'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=setup),
    ])
