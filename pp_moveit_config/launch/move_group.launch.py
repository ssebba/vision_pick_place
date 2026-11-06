"""MoveIt move_group (and optionally RViz) for the pick-and-place cell.

`arm_controller` selects which ros2_control controller executes the arm trajectories:
the stock joint_trajectory_controller (`arm_controller`) or the computed-torque controller
of this repository (`ctc_controller`); both expose FollowJointTrajectory.
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import yaml


def moveit_configs(arm_controller: str):
    share = get_package_share_directory('pp_moveit_config')
    with open(os.path.join(share, 'config', 'moveit_controllers.yaml'), encoding='utf-8') as f:
        controllers = yaml.safe_load(f)
    manager = controllers['moveit_simple_controller_manager']
    if arm_controller != 'arm_controller':
        manager[arm_controller] = manager.pop('arm_controller')
        manager['controller_names'] = [arm_controller, 'gripper_controller']
    fd, controllers_file = tempfile.mkstemp(prefix='moveit_controllers_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(controllers, f)

    return (
        MoveItConfigsBuilder('pp_robot', package_name='pp_moveit_config')
        .robot_description(file_path=os.path.join(
            get_package_share_directory('pp_description'), 'urdf', 'pp_robot.urdf.xacro'))
        .robot_description_semantic(file_path='config/pp_robot.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path=controllers_file)
        .planning_pipelines(pipelines=['ompl'], default_planning_pipeline='ompl')
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True)
        .to_moveit_configs()
    )


def launch_setup(context):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    config = moveit_configs(LaunchConfiguration('arm_controller').perform(context))
    move_group = Node(
        package='moveit_ros_move_group', executable='move_group', output='screen',
        parameters=[config.to_dict(), {'use_sim_time': use_sim_time}])
    rviz = Node(
        package='rviz2', executable='rviz2', output='log',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=['-d', os.path.join(
            get_package_share_directory('pp_moveit_config'), 'config', 'moveit.rviz')],
        parameters=[config.robot_description, config.robot_description_semantic,
                    config.robot_description_kinematics, config.planning_pipelines,
                    config.joint_limits, {'use_sim_time': use_sim_time}])
    return [move_group, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('arm_controller', default_value='arm_controller',
                              description="'arm_controller' (JTC) or 'ctc_controller'"),
        OpaqueFunction(function=launch_setup),
    ])
