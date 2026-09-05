#pragma once

#include "omni_trajectory_follower.h"

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <angles/angles.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

// casi el mismo módulo que el usado durante la cátedra. Ahora la pose es derivada de un robot_frame, agnóstico a cómo se obtiene la estimación (si es por odometría o por ekf)

class OmniPositionController : public OmniTrajectoryFollower
{
public:

	OmniPositionController();

protected:
	bool control(const rclcpp::Time& t, double& vx, double& vy, double& w);

private:

	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;

	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener tf_listener_;
	rclcpp::TimerBase::SharedPtr position_timer;
	std::string base_frame_; // frame del robot (base_link para odom o base_link_ekf para ekf)

	int current_wp_ = 0;
	double x_ = 0.0;
	double y_ = 0.0;
	double theta_ = 0.0;

	// obtenemos la pose de alguna transformación al frame del robot deseado
	void getCurrentPose();

	// Implementaremos únicamente pursuit-based trajectories para el TP
	bool getPursuitBasedGoal(double& gx, double& gy, double& ga);

	// Publica los waypoints para poder visualizarlos en RViz2
	void publishCurrentGoal(const rclcpp::Time& t, const double& goal_x, const double& goal_y, const double& goal_theta)
	{
		geometry_msgs::msg::PoseStamped goal;

		goal.header.frame_id = "map";
		goal.header.stamp = t;
		goal.pose.position.x = goal_x;
		goal.pose.position.y = goal_y;
		tf2::Quaternion q;
		q.setRPY(0, 0, goal_theta);
		goal.pose.orientation = tf2::toMsg(q);
		goal_pub_->publish(goal);
	}
};
