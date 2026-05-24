#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <robmovil_msgs/msg/trajectory.hpp>

class OmniTrajectoryFollower : public rclcpp::Node
{
public:

    OmniTrajectoryFollower();

    virtual bool control(
        const rclcpp::Time& t,
        double& vx,
        double& vy,
        double& w) = 0;

    bool nextPointIndex(
        const rclcpp::Time& time,
        size_t& next_point_idx) const;

    const robmovil_msgs::msg::Trajectory& getTrajectory() const
    {
        return current_trajectory_;
    }

    const rclcpp::Time& getInitialTime() const
    {
        return t0_;
    }

    void stop_timer()
    {
        if(timer_)
            timer_->cancel();
    }

protected:

    virtual void handleNewTrajectory(
        const robmovil_msgs::msg::Trajectory& trajectory_msg);

    void timerCallback();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    rclcpp::Subscription<robmovil_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;

    robmovil_msgs::msg::Trajectory current_trajectory_;

    rclcpp::Time t0_;

    rclcpp::TimerBase::SharedPtr timer_;
};