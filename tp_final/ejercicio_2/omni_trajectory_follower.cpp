#include "omni_trajectory_follower.h"

OmniTrajectoryFollower::OmniTrajectoryFollower()
    : Node("omni_trajectory_follower")
{
    rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
    qos_profile.reliable();
    qos_profile.transient_local();

    cmd_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>(
            "/robot/cmd_vel",
            rclcpp::QoS(10));

    trajectory_sub_ =
        this->create_subscription<robmovil_msgs::msg::Trajectory>(
            "/robot/trajectory",
            qos_profile,
            std::bind(
                &OmniTrajectoryFollower::handleNewTrajectory,
                this,
                std::placeholders::_1));
}

void OmniTrajectoryFollower::handleNewTrajectory(
    const robmovil_msgs::msg::Trajectory& trajectory_msg)
{
    RCLCPP_INFO(this->get_logger(), "New trajectory received");

    do {
        t0_ = this->now();
    } while (t0_.seconds() == 0.0);

    current_trajectory_ = trajectory_msg;

    if (timer_)
        timer_->cancel();

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(
            &OmniTrajectoryFollower::timerCallback,
            this));
}

void OmniTrajectoryFollower::timerCallback()
{
    rclcpp::Time t = this->now();

    double vx, vy, w;

    if (!control(t, vx, vy, w))
    {
        RCLCPP_INFO(this->get_logger(), "Trajectory finished");

        timer_->cancel();

        geometry_msgs::msg::Twist cmd;
        cmd_pub_->publish(cmd);

        return;
    }

    geometry_msgs::msg::Twist cmd;

    cmd.linear.x = vx;
    cmd.linear.y = vy;
    cmd.angular.z = w;

    cmd_pub_->publish(cmd);
}

bool OmniTrajectoryFollower::nextPointIndex(
    const rclcpp::Time& time,
    size_t& next_point_idx) const
{
    size_t idx = 0;

    for (const auto& point : current_trajectory_.points)
    {
        if (time < t0_ + point.time_from_start)
        {
            next_point_idx = idx;
            return true;
        }

        idx++;
    }

    return false;
}