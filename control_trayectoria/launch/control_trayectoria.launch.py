from launch import LaunchDescription
from launch_ros.actions import Node

ekf=True

def agregar_nodos_ekf():
    if (ekf):
        return [
        Node(
            package='landmark_detector',
            executable='landmark_detector',
            name='landmark_detector',
            output='screen',
            parameters=[{'publish_robot_frame': 'base_link'},
                        {"use_sim_time": True}]
        ),
        Node(
            package='robmovil_ekf',
            executable='localizer',
            name='localizer',
            output='screen',
            parameters=[{'only_prediction': False},
                        {'min_landmark_size': 3},
                        {"base_link_frame": "base_link"},
                        {"ekf_frame": "odom"},
                        {"use_sim_time": True}],
        )
    ]
    else:
        return []

def generate_launch_description():
    return LaunchDescription(agregar_nodos_ekf() + [
        Node(
            package="modelo_odometrico",
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
            parameters=[{'use_sim_time': True},
                        {'use_ekf': ekf}],
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
                    15., 0., -4., -1.57,
                    30., -4., -4., 3.14,
                    45., -4., 0., 1.57,
                    60., 0., 0., 0.
                ]}
            ],  
        ),
    ])
