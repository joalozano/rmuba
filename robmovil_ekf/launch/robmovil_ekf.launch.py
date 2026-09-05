from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='landmark_detector',
            executable='landmark_detector',
            name='landmark_detector',
            output='screen',
            parameters=[{"use_sim_time": True}]
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
                        {"use_sim_time": True}]
        )
    ])
