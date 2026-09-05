from launch import LaunchDescription
from launch_ros.actions import Node

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
            package='robmovil_ekf',
            executable='localizer',
            name='localizer',
            output='screen',
            parameters=[{'only_prediction': False},
                        {'min_landmark_size': 3},
                        {"use_sim_time": True}]
        ),

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
                        {'base_link_frame': "base_link_ekf"}]
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
                    0., -2., 2., 1.57,
                    5., 2., 2., 0.,
                    10., 2., -2., -1.57,
                    15., -2., -2., 3.14,
                    20., -2., 2., 1.57
                ]}
            ]  
        )
    ])
        # se puede controlar si se usa localización por ekf o odometría usando link_frame: base_link (odom) ó base_link_ekf (ekf)
