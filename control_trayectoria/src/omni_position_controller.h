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

protected:

    virtual void handleNewTrajectory(
        const robmovil_msgs::msg::Trajectory& trajectory_msg) override;

private:

    void getCurrentPoseFromOdometry(
        const nav_msgs::msg::Odometry& odom);

    //bool getPursuitBasedGoal(const rclcpp::Time& t,
    bool getPursuitBasedGoal(
        double& gx,
        double& gy,
        double& ga);

	/*tf2_ros::Buffer tfBuffer_;
	tf2_ros::TransformListener transform_listener_;
	rclcpp::Publisher<geometry_msgs>*/

    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;

    int current_wp_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
};
