"""Start task adapters alongside an already running mobile_dual_nero simulation."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from mobile_dual_nero.model import compose
from mobile_dual_nero.planning import planning_config


def setup(context):
    config = planning_config(*compose(LaunchConfiguration('mount_xyz').perform(context),
                                      LaunchConfiguration('mount_rpy').perform(context)))
    nodes = [Node(package='nero_task_executor', executable='sim_arm_server', name=side+'_task_arm',
                  output='screen', parameters=[config.to_dict(), {'use_sim_time': True, 'side': side}])
             for side in ('left', 'right')]
    nodes.append(Node(package='nero_task_executor', executable='task_executor', output='screen',
        parameters=[{'use_sim_time': True, 'backend': 'simulation',
                     'state_file': LaunchConfiguration('state_file')}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mount_xyz', default_value='0 0 0.275'),
        DeclareLaunchArgument('mount_rpy', default_value='0 0 0'),
        DeclareLaunchArgument('state_file', default_value=''),
        OpaqueFunction(function=setup)])
