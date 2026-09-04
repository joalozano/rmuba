#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <robmovil_msgs/msg/multi_encoder_ticks.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/utils.h>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

// Constantes pertinentes al robot holonómico:
#define WHEEL_COUNT 4
// WHEEL_X/Y corresponden a la distancia entre las ruedas en X y en Y, medidas en metros
#define WHEEL_X 0.350
#define WHEEL_Y 0.350
// auto-explicativo, radio medido en metros
#define WHEEL_RADIUS 0.050
#define ENCODER_TICKS 500.0
// la suma de l_x y l_y, definidos en la sección 2 del paper provisto y usado en muchas fórmulas
#define L ((WHEEL_X + WHEEL_Y)/2.0)

#define rads_per_tick ((2 * M_PI) / ENCODER_TICKS)

namespace robmovil
{

class OmniOdometry : public rclcpp::Node
{
  public:

	OmniOdometry();

	void on_velocity_cmd(const geometry_msgs::msg::Twist::SharedPtr twist);

	void on_encoder_ticks(const robmovil_msgs::msg::MultiEncoderTicks::SharedPtr encoder);

	// función para comparar /robot/ground_truth con /robot/odometry
	void gt_callback(const nav_msgs::msg::Odometry::SharedPtr gt);

  private:

	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
	rclcpp::Subscription<robmovil_msgs::msg::MultiEncoderTicks>::SharedPtr encoder_sub_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_sub_;

	// publishers para la velocidad de cada rueda mecanum, usando la misma convención de numeración que ticks[WHEEL_COUNT]
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_vel_[WHEEL_COUNT];

	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;

	double x_, y_, theta_;

	bool ticks_initialized_;
	int32_t last_ticks_[WHEEL_COUNT];
	rclcpp::Time last_ticks_time;

	//boost::shared_ptr<tf::TransformBroadcaster> tf_broadcaster;
	std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}
