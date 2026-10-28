from pathlib import Path
import json
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nero_dual_bringup.model import compose


def setup(context):
    geometry = {name: float(LaunchConfiguration(name).perform(context))
                for name in ('support_height', 'cabinet_height', 'cabinet_width',
                             'cabinet_depth', 'cabinet_front')}
    from nero_dual_bringup.scene import scene_objects
    scene_objects(**geometry)  # Validate dimensions before constructing either CAN driver.
    config, controllers = compose(geometry['support_height'])
    own = Path(get_package_share_directory('nero_dual_bringup'))
    right_camera = json.loads((own/'config/right_d405_handeye.json').read_text())
    xyz = [right_camera['T_gripper_camera'][i][3] for i in range(3)]
    quaternion = right_camera['quaternion_xyzw']
    driver = Path(get_package_share_directory('agx_arm_ctrl'))/'launch/start_single_agx_arm.launch.py'
    with tempfile.NamedTemporaryFile(mode='w', prefix='nero_dual_controllers_', suffix='.yaml', delete=False) as file:
        yaml.safe_dump(controllers, file)
        controller_path = file.name
    actions = []
    scene = Node(package='nero_dual_bringup', executable='cabinet_scene',
                 output='screen', parameters=[geometry])
    rviz = Node(package='rviz2', executable='rviz2', output='log',
                condition=IfCondition(LaunchConfiguration('use_rviz')),
                arguments=['-d', str(own/'config/dual.rviz')], parameters=[config])
    for side in ('left', 'right'):
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(driver)),
            launch_arguments={'namespace': side, 'can_port': LaunchConfiguration(side+'_can'),
                'arm_type': 'nero', 'effector_type': 'agx_gripper', 'fast_mode': 'false',
                'auto_enable': LaunchConfiguration('auto_enable'), 'control_enabled': 'false'}.items()))
        actions.append(Node(package='agx_arm_moveit', executable='agx_arm_control_gate',
            namespace=side, output='screen', parameters=[{
                'gate_service_name': 'control_enable',
                'status_topics': [f'/{side}_{part}_controller/follow_joint_trajectory/_action/status'
                                  for part in ('arm', 'gripper')]}]))
    actions += [
        Node(package='tf2_ros', executable='static_transform_publisher',
            name='right_d405_handeye', output='screen',
            arguments=['--frame-id', 'right_gripper_flange', '--child-frame-id',
                       'right_handeye_color_optical', '--x', str(xyz[0]), '--y', str(xyz[1]),
                       '--z', str(xyz[2]), '--qx', str(quaternion[0]), '--qy', str(quaternion[1]),
                       '--qz', str(quaternion[2]), '--qw', str(quaternion[3])]),
        Node(package='nero_dual_bringup', executable='joint_bridge', output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[{'robot_description': config['robot_description']}]),
        Node(package='moveit_ros_move_group', executable='move_group', output='screen', parameters=[config,
            {'publish_robot_description_semantic': True, 'publish_planning_scene': True,
             'publish_geometry_updates': True, 'publish_state_updates': True, 'publish_transforms_updates': True}]),
        Node(package='controller_manager', executable='ros2_control_node',
            parameters=[{'robot_description': config['robot_description']}, controller_path],
            remappings=[('joint_states', '/dual_control/joint_states')]),
        scene,
        rviz,
    ]
    for name in ('joint_state_broadcaster', 'left_arm_controller', 'right_arm_controller',
                 'left_gripper_controller', 'right_gripper_controller'):
        actions.append(Node(package='controller_manager', executable='spawner',
            arguments=[name, '--controller-manager', '/controller_manager']))
    return actions


def generate_launch_description():
    defaults = {'left_can': 'can1', 'right_can': 'can0', 'auto_enable': 'true',
                'use_rviz': 'true', 'support_height': '0.28',
                'cabinet_height': '0.60', 'cabinet_width': '1.20',
                'cabinet_depth': '0.40', 'cabinet_front': '0.50'}
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        OpaqueFunction(function=setup),
    ])
