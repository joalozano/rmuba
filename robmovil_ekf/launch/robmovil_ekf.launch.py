from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    debug = LaunchConfiguration('debug')
    detector = LaunchConfiguration('detector')

    return LaunchDescription([
        DeclareLaunchArgument('debug', default_value='true'),
        DeclareLaunchArgument('detector', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),

        Node(
            package='landmark_detector',
            executable='landmark_detector',
            name='landmark_detector',
            output='screen',
            parameters=[{'publish_robot_frame': 'base_link_ekf'},
                        {"use_sim_time": True}], 
            condition=IfCondition(detector)
        ),

        Node(
            package='landmark_detector',
            executable='landmark_detector',
            name='landmark_detector_gt',
            output='screen',
            parameters=[{'publish_robot_frame': 'base_link_gt'},
                        {"use_sim_time": True}], 
            remappings=[
                ('/landmarks_pointcloud', '/landmarks_pointcloud/groundtruth'),
                ('/landmarks', '/landmarks/groundtruth'),
            ],
            condition=IfCondition(detector)
        ),

        Node(
            package='modelo_odometrico',
            executable='omni_odometry_node',
            name='omni_odometry',
            output='screen',
            parameters=[{"use_sim_time": True}]
        ),

        # EKF node
        Node(
            package='robmovil_ekf',
            executable='localizer',
            name='localizer',
            output='screen',
            parameters=[{'only_prediction': False},
                        {'min_landmark_size': 1},
                        {"ekf_frame": "odom"},
                        {"use_sim_time": True}]
        )
    ])
