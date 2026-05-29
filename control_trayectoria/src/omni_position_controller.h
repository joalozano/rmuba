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

	bool control(const rclcpp::Time& t, double& vx, double& vy, double& w);

private:

	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

	int current_wp_ = 0;
	double x_ = 0.0;
	double y_ = 0.0;
	double theta_ = 0.0;

	void getCurrentPoseFromOdometry(const nav_msgs::msg::Odometry& odom);

	// Implementaremos únicamente pursuit-based trajectories para el TP
	bool getPursuitBasedGoal(double& gx, double& gy, double& ga);

	// Publica los waypoints para poder visualizarlos en RViz2
	void publishCurrentGoal(const rclcpp::Time& t, const double& goal_x, const double& goal_y, const double& goal_theta);
};
