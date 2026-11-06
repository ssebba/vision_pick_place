"""Gazebo work cell: UR5e + gripper + RGB-D camera, ros2_control, random objects on the table.

The world is generated at launch from `seed` (object shapes, poses), so benchmark trials are
reproducible and different from each other.

  ros2 launch pp_bringup sim.launch.py headless:=false seed:=3
"""

import math
import os
import random
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler, Shutdown)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml

WORLD_NAME = 'pick_place'
ARM_CONTROLLERS = ['arm_controller', 'ctc_controller', 'impedance_controller']

# (shape, size) of the objects, cycled: boxes are 4-6 cm wide (the gripper opens 8.5 cm).
OBJECT_TYPES = [
    ('box', (0.045, 0.07, 0.05)),
    ('cylinder', (0.025, 0.08)),
    ('box', (0.05, 0.05, 0.06)),
    ('cylinder', (0.03, 0.06)),
    ('box', (0.04, 0.08, 0.04)),
]
COLORS = ['0.85 0.2 0.15 1', '0.15 0.45 0.85 1', '0.2 0.7 0.3 1', '0.9 0.7 0.1 1',
          '0.6 0.3 0.8 1']


def sample_poses(count, rng):
    """Non-overlapping (x, y, yaw) in the workspace in front of the robot."""
    poses = []
    while len(poses) < count:
        x, y = rng.uniform(0.40, 0.62), rng.uniform(-0.20, 0.20)
        if all(math.hypot(x - px, y - py) > 0.12 for px, py, _ in poses):
            poses.append((x, y, rng.uniform(-math.pi / 2, math.pi / 2)))
    return poses


def object_model(index, x, y, yaw):
    shape, size = OBJECT_TYPES[index % len(OBJECT_TYPES)]
    mass = 0.08
    if shape == 'box':
        sx, sy, sz = size
        geometry = f'<box><size>{sx} {sy} {sz}</size></box>'
        height = sz
        ixx, iyy, izz = (mass * (sy**2 + sz**2) / 12, mass * (sx**2 + sz**2) / 12,
                         mass * (sx**2 + sy**2) / 12)
    else:
        r, h = size
        geometry = f'<cylinder><radius>{r}</radius><length>{h}</length></cylinder>'
        height = h
        ixx = iyy = mass * (3 * r**2 + h**2) / 12
        izz = mass * r**2 / 2
    color = COLORS[index % len(COLORS)]
    return f'''
    <model name="object_{index}">
      <pose>{x:.4f} {y:.4f} {height / 2 + 0.002:.4f} 0 0 {yaw:.4f}</pose>
      <link name="link">
        <inertial>
          <mass>{mass}</mass>
          <inertia><ixx>{ixx:.3e}</ixx><iyy>{iyy:.3e}</iyy><izz>{izz:.3e}</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="collision">
          <geometry>{geometry}</geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry>{geometry}</geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
        </visual>
      </link>
      <!-- Ground truth for the benchmark: the model pose, named (child_frame_id) unlike the
           poses of the scene broadcaster once converted by ros_gz_bridge. -->
      <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
        <publish_model_pose>true</publish_model_pose>
        <publish_link_pose>false</publish_link_pose>
        <publish_collision_pose>false</publish_collision_pose>
        <publish_visual_pose>false</publish_visual_pose>
        <use_pose_vector_msg>true</use_pose_vector_msg>
        <update_frequency>20</update_frequency>
      </plugin>
    </model>'''


def recording_camera(directory: str) -> str:
    """Static camera with an oblique view of the cell, saving every frame (README GIF)."""
    return f'''
    <model name="recording_camera">
      <static>true</static>
      <pose>1.50 1.05 1.05 0 0.466 -2.386</pose>
      <link name="link">
        <sensor name="recorder" type="camera">
          <always_on>true</always_on>
          <update_rate>15</update_rate>
          <topic>/recording_camera/image</topic>
          <camera>
            <horizontal_fov>1.05</horizontal_fov>
            <image><width>960</width><height>540</height><format>R8G8B8</format></image>
            <clip><near>0.05</near><far>10.0</far></clip>
            <save enabled="true"><path>{directory}</path></save>
          </camera>
        </sensor>
      </link>
    </model>'''


def make_world(num_objects: int, seed: int, record_dir: str = '') -> str:
    rng = random.Random(seed)
    models = ''.join(object_model(i, *p) for i, p in enumerate(sample_poses(num_objects, rng)))
    if record_dir:
        models += recording_camera(record_dir)
    sdf = f'''<?xml version="1.0"?>
<sdf version="1.9">
  <world name="{WORLD_NAME}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <!-- External wrenches on demand (/world/pick_place/wrench), e.g. to measure the
         stiffness of the Cartesian impedance controller. -->
    <plugin filename="gz-sim-apply-link-wrench-system" name="gz::sim::systems::ApplyLinkWrench"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <pose>0 0 -0.8 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal></plane></geometry></collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material><ambient>0.6 0.6 0.6 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material>
        </visual>
      </link>
    </model>{models}
  </world>
</sdf>
'''
    fd, path = tempfile.mkstemp(prefix=f'pick_place_{seed}_', suffix='.sdf')
    with os.fdopen(fd, 'w') as f:
        f.write(sdf)
    return path


def make_bridge_config(num_objects: int, record: bool = False) -> str:
    entries = [
        {'ros_topic_name': '/camera/points', 'gz_topic_name': '/camera/points',
         'ros_type_name': 'sensor_msgs/msg/PointCloud2', 'gz_type_name': 'gz.msgs.PointCloudPacked',
         'direction': 'GZ_TO_ROS'},
        {'ros_topic_name': '/camera/image', 'gz_topic_name': '/camera/image',
         'ros_type_name': 'sensor_msgs/msg/Image', 'gz_type_name': 'gz.msgs.Image',
         'direction': 'GZ_TO_ROS'},
    ]
    if record:
        # A camera renders only while something subscribes to it, saving frames included: the
        # bridge is that subscriber.
        entries.append({
            'ros_topic_name': '/recording_camera/image', 'gz_topic_name': '/recording_camera/image',
            'ros_type_name': 'sensor_msgs/msg/Image', 'gz_type_name': 'gz.msgs.Image',
            'direction': 'GZ_TO_ROS'})
    for i in range(num_objects):
        # Ground truth (benchmark only): every object publishes its pose on the same topic.
        entries.append({
            'ros_topic_name': '/sim/model_poses', 'gz_topic_name': f'/model/object_{i}/pose',
            'ros_type_name': 'tf2_msgs/msg/TFMessage', 'gz_type_name': 'gz.msgs.Pose_V',
            'direction': 'GZ_TO_ROS'})
        for action in ('attach', 'detach'):
            entries.append({
                'ros_topic_name': f'/object_{i}/{action}', 'gz_topic_name': f'/object_{i}/{action}',
                'ros_type_name': 'std_msgs/msg/Empty', 'gz_type_name': 'gz.msgs.Empty',
                'direction': 'ROS_TO_GZ'})
    fd, path = tempfile.mkstemp(prefix='pp_bridge_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(entries, f)
    return path


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    num_objects = int(arg('num_objects'))
    world = make_world(num_objects, int(arg('seed')), arg('record_dir'))
    description_share = get_package_share_directory('pp_description')
    robot_description = ParameterValue(Command([
        'xacro ', os.path.join(description_share, 'urdf', 'pp_robot.urdf.xacro'),
        ' num_objects:=', str(num_objects),
        ' controllers_file:=', os.path.join(description_share, 'config', 'ros2_controllers.yaml'),
    ]), value_type=str)

    # Gazebo server and GUI as separate processes, without the shell that ros_gz_sim's
    # gz_sim.launch.py puts in front of them: the shell does not forward SIGINT, and a combined
    # server + GUI process waits for its GUI child. Either way a launch that shuts itself down
    # (task finished, failed node) left the server running, and the next launch found "another
    # world of the same name".
    headless = arg('headless').lower() == 'true'
    gz_env = {'GZ_SIM_SYSTEM_PLUGIN_PATH': os.pathsep.join(filter(None, [
        os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH'), os.environ.get('LD_LIBRARY_PATH')]))}
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s'] + (['--headless-rendering'] if headless else []) + [world],
        name='gazebo', output='screen', on_exit=Shutdown(), additional_env=gz_env)
    gazebo_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-g', '--gui-config',
             os.path.join(get_package_share_directory('pp_bringup'), 'config', 'gazebo_gui.config')],
        name='gazebo_gui', output='log', additional_env=gz_env)

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='log',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])
    spawn = Node(
        package='ros_gz_sim', executable='create', output='log',
        arguments=['-topic', 'robot_description', '-name', 'pp_robot', '-allow_renaming', 'false'])
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='log',
        parameters=[{'config_file': make_bridge_config(num_objects, bool(arg('record_dir'))),
                     'use_sim_time': True}])
    # /clock on a bridge of its own: the controller manager gives its controllers the ROS time,
    # and a bridge busy converting point clouds delivered the clock late and in bursts.
    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge', output='log',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])

    controllers_file = os.path.join(description_share, 'config', 'ros2_controllers.yaml')

    def spawner(*controllers, inactive=False):
        # The controller manager inside Gazebo does not forward its parameter file to the
        # controller nodes: each spawner loads it into the controllers it starts.
        args = list(controllers) + ['--controller-manager', '/controller_manager',
                                    '--param-file', controllers_file]
        if inactive:
            args.append('--inactive')
        return Node(package='controller_manager', executable='spawner', arguments=args,
                    output='log')

    # One arm controller at a time: they claim the same joints (position vs effort interfaces).
    arm_controller = arg('arm_controller')
    others = [c for c in ARM_CONTROLLERS if c != arm_controller]
    active_controllers = spawner('joint_state_broadcaster', arm_controller, 'gripper_controller')
    inactive_controllers = spawner(*others, inactive=True)

    # The detachable joints are created attached when the robot appears: release every object
    # right away, before an arm without active controller can sag and drag them along. The
    # command is repeated: gz-transport drops it if the plugin is not subscribed yet, and an
    # object left hanging from the wrist is a load the model-based controllers do not know.
    detach_all = [ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--times', '5', '--rate', '1', '-w', '1',
             f'/object_{i}/detach', 'std_msgs/msg/Empty', '{}'], output='log')
        for i in range(num_objects)]

    return [gazebo] + ([] if headless else [gazebo_gui]) + [
        robot_state_publisher, spawn, bridge, clock_bridge,
        RegisterEventHandler(OnProcessExit(
            target_action=spawn, on_exit=[active_controllers] + detach_all)),
        RegisterEventHandler(OnProcessExit(
            target_action=active_controllers, on_exit=[inactive_controllers])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true',
                              description='Run the Gazebo server only'),
        DeclareLaunchArgument('num_objects', default_value='3'),
        DeclareLaunchArgument('seed', default_value='1',
                              description='Seed of the object shapes and poses'),
        DeclareLaunchArgument('arm_controller', default_value='arm_controller',
                              description=f'Active arm controller: {ARM_CONTROLLERS}'),
        DeclareLaunchArgument('record_dir', default_value='',
                              description='If set, an extra camera saves its frames here'),
        OpaqueFunction(function=launch_setup),
    ])
