#pragma once

#include "omni_trajectory_follower.h"

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <angles/angles.h>

class OmniPositionController : public OmniTrajectoryFollower
{
public:

	OmniPositionController();

protected:
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
	void publishCurrentGoal(const rclcpp::Time& t, const double& goal_x, const double& goal_y, const double& goal_theta)
	{
		geometry_msgs::msg::PoseStamped goal;

		goal.header.frame_id = "odom";
		goal.header.stamp = t;
		goal.pose.position.x = goal_x;
		goal.pose.position.y = goal_y;
		tf2::Quaternion q;
		q.setRPY(0, 0, goal_theta);
		goal.pose.orientation = tf2::toMsg(q);
		goal_pub_->publish(goal);
	}
};
