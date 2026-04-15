import ament_index_python
import launch_testing.actions
import os
import pytest
import rclpy
import unittest

from aruco_detector_skill_msgs.action import ArucoDetectorSkill
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from rclpy.action import ActionClient
from rclpy.node import Node


# Run the skill server before running the tests
@pytest.mark.launch_test
def generate_test_description():

    # This is necessary to get unbuffered output from the process under test
    proc_env = os.environ.copy()
    proc_env['PYTHONUNBUFFERED'] = '1'

    print('Inside testing -----')
    pkg_path = ament_index_python.get_package_prefix('aruco_detector_skill_server')
    pkg_path = pkg_path.replace('install', 'build', 1)
    return LaunchDescription([
        ExecuteProcess(
            cmd=[pkg_path + '/aruco_detector_skill_server_node'],
            env=proc_env,
            output='screen'
        ),
        # Start tests right away - no need to wait for anything
        launch_testing.actions.ReadyToTest()
    ])


class ArucoDetectorSkillTestClient(Node):

    # Class constructor - initialize node
    def __init__(self):
        super().__init__('aruco_detector_skill_test_client')
        self._action_client = ActionClient(self, ArucoDetectorSkill, 'ArucoDetectorSkill')

    # Send goal to server
    def send_goal(self, goal, callback):

        # Send goal to server
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal)
        self.get_logger().info('Sent goal to server')

        # Add callbacks
        self._send_goal_future.add_done_callback(callback)

    def shutdown(self):
        rclpy.shutdown()


class TestArucoDetectorSkillServer(unittest.TestCase):

    def __goal_response_callback_success(self, future):

        goal_handle = future.result()
        self.assertTrue(goal_handle.accepted)
        self.client.get_logger().info('Goal was accepted by the server')

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.__get_result_callback_success)

    def __get_result_callback_success(self, future):

        self.client.get_logger().info('Received result from the server')

        result = future.result().result
        self.assertEqual(result.percentage, 100)
        self.assertEqual(result.skill_status, 'ArucoDetector Skill: Succeeded')

        # Ends test
        self.client.shutdown()

    def test_skill_execution_success(self):
        rclpy.init(args=None)

        # Prepare goal
        goal = ArucoDetectorSkill.Goal()

        self.client = ArucoDetectorSkillTestClient()
        self.client.send_goal(goal, self.__goal_response_callback_success)
        rclpy.spin(self.client)
