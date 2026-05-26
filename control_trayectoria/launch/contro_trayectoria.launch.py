from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package="modelo_diferencial",
            executable="omni_odometry_node",
            name="omni_odometry",
            output="screen",
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package="control_trayectoria",
            executable="trajectory_follower_cl",
            name="trajectory_follower_cl",
            output="screen",
        ),

        Node(
            package="control_trayectoria",
            executable="trajectory_generator",
            name="trajectory_generator",
            output="screen",
        ),
    ])
