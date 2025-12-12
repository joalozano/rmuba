from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package="modelo_diferencial",
            executable="pioneer_odometry_node",
            name="pioneer_odometry",
            output="screen",
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package="lazo_cerrado",
            executable="trajectory_follower_cl",
            name="trajectory_follower_cl",
            output="screen",
            parameters=[
                {'use_sim_time': True},
                {"goal_selection": "PURSUIT_BASED"}, #FIXED_GOAL, TIME_BASED, PURSUIT_BASED
                {"fixed_goal_x": float(2.0)},
                {"fixed_goal_y": float(2.0)},
                {"fixed_goal_a": float(-0.785)}, # -1/2 * PI
            ],
        ),

        Node(
            package="lazo_abierto",
            executable="trajectory_generator",
            name="trajectory_generator",
            output="screen",
            parameters=[
                {'use_sim_time': True},
                {'trajectory_type': 'spline'}, #sin or spline
                {"stepping": float(0.1)},
                {"total_time": float(10.0)},
                {"amplitude": float(1.0)},
                {"cycles": float(1.0)},
                {'spline_waypoints': [
                    0., 0., 0., 0.,
                    10., 5., 0., 1.57,
                    20., 5., 5., 3.14,
                    30., 0., 5., 4.71,
                    40., 0., 0., 0.
                ]}
            ],  
        ),
	# Note: each waypoint must have 4 values: time(sec), position_x(m), position_y(m), orientation(rad)
        Node(
            package="lazo_cerrado",
            executable="logger",
            name="logger",
            output="screen",
            parameters=[{'use_sim_time': True}]
        ),
    ])
