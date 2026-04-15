from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get config file path
    config_file = os.path.join(
        get_package_share_directory('aruco_detector_skill_server'),
        'config',
        'server_config.yaml'
    )

    # Create node pointing to YAML file
    aruco_detector_skill_server_node = Node(
        package='aruco_detector_skill_server',
        executable='aruco_detector_skill_server_node',
        name='aruco_detector_skill_server',
        parameters=['/server_config.yaml', '/pipeline_config.yaml'  ],
        output='screen',
    )

    return LaunchDescription([
        aruco_detector_skill_server_node
    ])