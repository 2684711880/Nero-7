from moveit_configs_utils import MoveItConfigsBuilder


def planning_config(description, semantic):
    config = (MoveItConfigsBuilder('dual_nero', package_name='dual_nero_moveit_config')
              .robot_description(file_path='urdf/dual_nero.urdf.xacro')
              .robot_description_semantic(file_path='config/dual_nero.srdf')
              .robot_description_kinematics(file_path='config/kinematics.yaml')
              .trajectory_execution(file_path='config/moveit_controllers.yaml')
              .joint_limits(file_path='config/joint_limits.yaml')
              .planning_pipelines(pipelines=['ompl'])
              .to_moveit_configs())
    config.robot_description = {'robot_description': description}
    config.robot_description_semantic = {'robot_description_semantic': semantic}
    return config
