import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('aruco_detector_skill_server')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Rosbag) clock if true'
    )

    declare_server_config_arg = DeclareLaunchArgument(
        'server_config_file',
        default_value=os.path.join(pkg_dir, 'config', 'server_config.yaml'),
        description='Path to the server configuration YAML file'
    )


    aruco_detector_skill_server_node = Node(
        package='aruco_detector_skill_server',
        executable='aruco_detector_skill_server_node',
        name='aruco_detector_skill_server',
        output='screen',
        parameters=[
            LaunchConfiguration('server_config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_server_config_arg,
        aruco_detector_skill_server_node
    ])
