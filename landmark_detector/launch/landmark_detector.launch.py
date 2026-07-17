from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # Omni odometry node
        Node(
            package="modelo_odometrico",
            executable="omni_odometry_node",  # check executable name in ROS 2
            name="omni_odometry",
            output="screen",
            parameters=[{"use_sim_time": True}]
        ),

        #Landmark detector
        Node(
            package="landmark_detector",
            executable="landmark_detector",
            name="landmark_detector",
            output="screen",
            parameters=[{"use_sim_time": True}]
        ),
    ])
