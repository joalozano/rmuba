#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <robmovil_msgs/msg/trajectory.hpp>

class OmniTrajectoryFollower : public rclcpp::Node
{
public:

	OmniTrajectoryFollower();

	void stop_timer() { timer_->cancel(); }

protected:

	const rclcpp::Time& getInitialTime() const { return t0_; }

	const robmovil_msgs::msg::Trajectory& getTrajectory() const { return current_trajectory_; }

	void handleNewTrajectory(const robmovil_msgs::msg::Trajectory& trajectory_msg);
	
	virtual bool control(const rclcpp::Time& t, double& vx, double& vy, double& w) = 0;

private:

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
	rclcpp::Subscription<robmovil_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;

	// tiempo inicial de trayectoria
	rclcpp::Time t0_;

	// trayectoria actual
	robmovil_msgs::msg::Trajectory current_trajectory_;
	
	// funciones auxiliares
	void timerCallback();
};
