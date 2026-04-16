from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():

    use_sim_time_arg = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Rosbag) clock if true'
    )

    server_config = PathJoinSubstitution([
        FindPackageShare('aruco_detector_skill_server'),
        'config',
        'server_config.yaml'
    ])

    # Create node pointing to YAML file
    aruco_detector_skill_server_node = Node(
        package='aruco_detector_skill_server',
        executable='aruco_detector_skill_server_node',
        name='aruco_detector_skill_server',
        output='screen',
        parameters=[server_config]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        aruco_detector_skill_server_node
    ])
