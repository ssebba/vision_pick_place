"""Complete pick-and-place demo: Gazebo cell, MoveIt, perception and behavior tree.

  ros2 launch pp_bringup pick_place.launch.py                     # JTC, RViz
  ros2 launch pp_bringup pick_place.launch.py arm_controller:=ctc_controller seed:=4

With `results_file` set, the task manager appends one CSV row per object and the launch
shuts down when the task is over (used by scripts/run_trials.sh). With `state_bag` set, the
state of the arm controller (reference, feedback, error at 100 Hz) is recorded to that bag.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
import yaml


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    bringup = get_package_share_directory('pp_bringup')
    arm_controller = arg('arm_controller')
    benchmark = bool(arg('results_file'))

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'sim.launch.py')),
        launch_arguments={'headless': arg('headless'), 'seed': arg('seed'),
                          'num_objects': arg('num_objects'),
                          'arm_controller': arm_controller,
                          'record_dir': arg('record_dir')}.items())
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('pp_moveit_config'), 'launch', 'move_group.launch.py')),
        launch_arguments={'use_rviz': arg('use_rviz'), 'arm_controller': arm_controller}.items())

    detector = LifecycleNode(
        package='pp_perception', executable='object_detector', name='object_detector',
        namespace='', output='screen', autostart=True,
        parameters=[{'use_sim_time': True}])

    # MoveGroupInterface loads its own robot model: it needs the IK solver settings and the
    # acceleration limits (time parametrization of the Cartesian paths); the rest comes
    # from move_group.
    moveit_config = os.path.join(get_package_share_directory('pp_moveit_config'), 'config')
    with open(os.path.join(moveit_config, 'kinematics.yaml'), encoding='utf-8') as f:
        kinematics = {'robot_description_kinematics': yaml.safe_load(f)}
    with open(os.path.join(moveit_config, 'joint_limits.yaml'), encoding='utf-8') as f:
        joint_limits = {'robot_description_planning': yaml.safe_load(f)}
    task_manager = LifecycleNode(
        package='pp_task', executable='task_manager', name='task_manager',
        namespace='', output='screen', autostart=True,
        parameters=[kinematics, joint_limits, {
            'use_sim_time': True,
            'results_file': arg('results_file'),
            'run_id': arg('run_id') or f'seed{arg("seed")}_{arm_controller}',
            'shutdown_when_done': benchmark,
            'wait_for_actions': [f'{arm_controller}/follow_joint_trajectory',
                                 'gripper_controller/follow_joint_trajectory'],
            'start_delay': 2.0,
        }])
    actions = [sim, move_group, detector, task_manager]
    if arg('state_bag'):
        actions.append(ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '--output', arg('state_bag'),
                 '--topics', f'/{arm_controller}/controller_state'], output='log'))
    if benchmark:
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=task_manager,
            on_exit=[EmitEvent(event=Shutdown(reason='task finished'))])))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('seed', default_value='1'),
        DeclareLaunchArgument('num_objects', default_value='3'),
        DeclareLaunchArgument('arm_controller', default_value='arm_controller',
                              description="'arm_controller' (JTC) or 'ctc_controller'"),
        DeclareLaunchArgument('results_file', default_value='',
                              description='CSV with one row per object (benchmark mode)'),
        DeclareLaunchArgument('run_id', default_value=''),
        DeclareLaunchArgument('state_bag', default_value='',
                              description='Record the arm controller state to this bag'),
        DeclareLaunchArgument('record_dir', default_value='',
                              description='Save the frames of an extra camera here (GIF)'),
        OpaqueFunction(function=launch_setup),
    ])
