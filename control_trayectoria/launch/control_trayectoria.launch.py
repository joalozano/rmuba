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
            parameters=[{'use_sim_time': True}],
        ),

        Node(
            package="control_trayectoria",
            executable="trajectory_generator",
            name="trajectory_generator",
            output="screen",
            parameters=[
                {'use_sim_time': True},
                {"stepping": float(0.1)},
                {'spline_waypoints': [
                    0., 0., 0., 0.,
                    10., 0., 4., 1.57,
                    20., 4., 4., 3.14,
                    30., 4., 0., 4.71,
                    40., 0., 0., 0.
                ]}
            ],  
        ),
    ])
