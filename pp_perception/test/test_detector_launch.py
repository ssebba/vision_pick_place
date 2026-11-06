"""Integration test of the installed object_detector executable, started by a launch file.

A static transform places a camera above the table and the test process publishes a
synthetic tabletop cloud (table, one box, one cylinder) in the camera frame. The detector,
brought to ACTIVE by the launch file and configured through launch parameters, must publish
both objects in the world frame with the right positions, sizes and classes.
"""

import math
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import numpy as np
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from vision_msgs.msg import Detection3DArray

CLOUD_TOPIC = '/test/points'
STEP = 0.004
# Camera 0.9 m above the table, looking down: rotation of 180 deg about x.
CAMERA_XYZ = np.array([0.5, 0.0, 0.9])
WORLD_FROM_CAMERA = np.diag([1.0, -1.0, -1.0])

BOX_CENTER, BOX_YAW, BOX_SIZE = (0.55, 0.10), 0.3, (0.05, 0.07, 0.05)
CYLINDER_CENTER, CYLINDER_RADIUS, CYLINDER_HEIGHT = (0.42, -0.15), 0.025, 0.08


def table_points():
    xs, ys = np.meshgrid(np.arange(0.25, 0.85, STEP), np.arange(-0.35, 0.35, STEP))
    return np.column_stack([xs.ravel(), ys.ravel(), np.zeros(xs.size)])


def box_points():
    sx, sy, sz = BOX_SIZE
    us = np.arange(-sx / 2, sx / 2 + 1e-9, STEP)
    vs = np.arange(-sy / 2, sy / 2 + 1e-9, STEP)
    zs = np.arange(STEP, sz, STEP)
    local = [(u, v, sz) for u in us for v in vs]
    local += [(u, v, z) for z in zs for u in us for v in (-sy / 2, sy / 2)]
    local += [(u, v, z) for z in zs for v in vs for u in (-sx / 2, sx / 2)]
    c, s = math.cos(BOX_YAW), math.sin(BOX_YAW)
    rotation = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return np.array(local) @ rotation.T + np.array([*BOX_CENTER, 0.0])


def cylinder_points():
    def ring(radius, z):
        count = max(1, int(2 * math.pi * radius / STEP))
        return [(radius * math.cos(a), radius * math.sin(a), z)
                for a in np.linspace(0.0, 2 * math.pi, count, endpoint=False)]

    local = []
    for radius in np.arange(0.0, CYLINDER_RADIUS + 1e-9, STEP):
        local += ring(radius, CYLINDER_HEIGHT)
    for z in np.arange(STEP, CYLINDER_HEIGHT, STEP):
        local += ring(CYLINDER_RADIUS, z)
    return np.array(local) + np.array([*CYLINDER_CENTER, 0.0])


def cloud_in_camera_frame():
    world = np.vstack([table_points(), box_points(), cylinder_points()])
    world += np.random.default_rng(7).normal(0.0, 0.001, world.shape)
    # p_world = t + R p_camera, so p_camera = R^T (p_world - t), here on row vectors.
    return ((world - CAMERA_XYZ) @ WORLD_FROM_CAMERA).astype(np.float32)


@pytest.mark.launch_test
def generate_test_description():
    detector = launch_ros.actions.LifecycleNode(
        package='pp_perception', executable='object_detector', name='object_detector',
        namespace='', output='screen', autostart=True,
        parameters=[{'input_topic': CLOUD_TOPIC, 'target_frame': 'world', 'min_period': 0.0}])
    camera_tf = launch_ros.actions.Node(
        package='tf2_ros', executable='static_transform_publisher', output='log',
        arguments=['--x', str(CAMERA_XYZ[0]), '--y', str(CAMERA_XYZ[1]),
                   '--z', str(CAMERA_XYZ[2]), '--roll', str(math.pi),
                   '--frame-id', 'world', '--child-frame-id', 'camera_optical_frame'])
    return launch.LaunchDescription([detector, camera_tf, launch_testing.actions.ReadyToTest()])


class TestObjectDetector(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=[])
        cls.node = rclpy.create_node('object_detector_test')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_detects_box_and_cylinder(self):
        received = []
        self.node.create_subscription(
            Detection3DArray, '/object_detector/detections', received.append, 10)
        publisher = self.node.create_publisher(PointCloud2, CLOUD_TOPIC, qos_profile_sensor_data)
        cloud = point_cloud2.create_cloud_xyz32(
            Header(frame_id='camera_optical_frame'), cloud_in_camera_frame())

        # Publish until the detector (started and activated by the launch) answers.
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline and not received:
            cloud.header.stamp = self.node.get_clock().now().to_msg()
            publisher.publish(cloud)
            rclpy.spin_once(self.node, timeout_sec=0.5)
        self.assertTrue(received, 'no detections received')

        msg = received[-1]
        self.assertEqual(msg.header.frame_id, 'world')
        by_class = {d.results[0].hypothesis.class_id: d.bbox for d in msg.detections}
        self.assertEqual(sorted(by_class), ['box', 'cylinder'])

        box = by_class['box']
        self.assertAlmostEqual(box.center.position.x, BOX_CENTER[0], delta=0.01)
        self.assertAlmostEqual(box.center.position.y, BOX_CENTER[1], delta=0.01)
        self.assertAlmostEqual(box.size.x, min(BOX_SIZE[:2]), delta=0.01)
        self.assertAlmostEqual(box.size.y, max(BOX_SIZE[:2]), delta=0.01)
        self.assertAlmostEqual(box.size.z, BOX_SIZE[2], delta=0.01)

        cylinder = by_class['cylinder']
        self.assertAlmostEqual(cylinder.center.position.x, CYLINDER_CENTER[0], delta=0.01)
        self.assertAlmostEqual(cylinder.center.position.y, CYLINDER_CENTER[1], delta=0.01)
        self.assertAlmostEqual(cylinder.size.x, 2 * CYLINDER_RADIUS, delta=0.01)
        self.assertAlmostEqual(cylinder.size.z, CYLINDER_HEIGHT, delta=0.01)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
