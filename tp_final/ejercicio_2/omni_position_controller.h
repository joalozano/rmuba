#pragma once

#include "omni_trajectory_follower.h"

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/utils.h>
#include <angles/angles.h>

class OmniPositionController : public OmniTrajectoryFollower
{
public:

    OmniPositionController();

    bool control(
        const rclcpp::Time& t,
        double& vx,
        double& vy,
        double& w) override;

private:

    void getCurrentPoseFromOdometry(
        const nav_msgs::msg::Odometry& odom);

    bool getPursuitBasedGoal(
        double& gx,
        double& gy,
        double& ga);

    double x_;
    double y_;
    double theta_;

    int current_wp_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
};