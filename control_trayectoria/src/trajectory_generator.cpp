#include <rclcpp/rclcpp.hpp>

#include <robmovil_msgs/msg/trajectory.hpp>
#include <robmovil_msgs/msg/trajectory_point.hpp>

#include <nav_msgs/msg/path.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

void addSegment(
    robmovil_msgs::msg::Trajectory& traj,
    nav_msgs::msg::Path& path,
    double x0,
    double y0,
    double x1,
    double y1,
    double spacing,
    double& accumulated_time)
{
    double dx = x1 - x0;
    double dy = y1 - y0;

    double length = hypot(dx, dy);

    int steps =
        std::max(1, (int)(length / spacing));

    for (int i = 0; i <= steps; i++)
    {
        double alpha =
            (double)i / (double)steps;

        double x =
            x0 + alpha * dx;

        double y =
            y0 + alpha * dy;

        // orientación opuesta al centro

        double theta =
            atan2(y, x);

        robmovil_msgs::msg::TrajectoryPoint point;

        point.time_from_start =
            rclcpp::Duration::from_seconds(
                accumulated_time);

        point.transform.translation.x = x;
        point.transform.translation.y = y;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta);

        point.transform.rotation =
            tf2::toMsg(q);

        traj.points.push_back(point);

        geometry_msgs::msg::PoseStamped pose;

        pose.header.frame_id = "odom";

        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.orientation =
            tf2::toMsg(q);

        path.poses.push_back(pose);

        accumulated_time += 0.1;
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable();
    qos.transient_local();

    auto node =
        rclcpp::Node::make_shared(
            "trajectory_generator");

    auto traj_pub =
        node->create_publisher<
            robmovil_msgs::msg::Trajectory>(
                "/robot/trajectory",
                qos);

    auto path_pub =
        node->create_publisher<nav_msgs::msg::Path>(
            "/target_path",
            10);

    robmovil_msgs::msg::Trajectory traj_msg;
    traj_msg.header.frame_id = "odom";

    nav_msgs::msg::Path path;
    path.header.frame_id = "odom";

    double t = 0.0;

    double spacing = 0.1;
	
    addSegment(traj_msg, path,
        2,-2,
       -2,-2,
        spacing, t);

    addSegment(traj_msg, path,
       -2, 2,
        2, 2,
        spacing, t);

    addSegment(traj_msg, path,
       -2,-2,
       -2, 2,
        spacing, t);

    addSegment(traj_msg, path,
        2, 2,
        2,-2,
        spacing, t);


    traj_pub->publish(traj_msg);

    path_pub->publish(path);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
