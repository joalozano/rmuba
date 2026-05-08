#include "omni_odometry.h"
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <angles/angles.h> // para tener normalize_angle

// matrices definidas en las ecuaciones 19 y 21 del paper
double T_for[WHEEL_COUNT][3] = {{1, -1, -L},
		     {1, 1, L},
		     {1, 1, -L},
		     {1, -1, L}};
double T_inv[3][WHEEL_COUNT] = {{1, 1, 1, 1}, 
		     {-1, 1, 1, -1},
		     {-1, 1, -1, 1}};

using namespace robmovil;

OmniOdometry::OmniOdometry() : Node("nodeOdometry"), x_(0), y_(0), theta_(0), ticks_initialized_(false)
{
	// Nos suscribimos a los comandos de velocidad en el tópico "/robot/cmd_vel" de tipo geometry_msgs::Twist
	twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
		"/robot/cmd_vel", rclcpp::QoS(10),
		std::bind(&OmniOdometry::on_velocity_cmd, this, std::placeholders::_1));

	encoder_sub_ =	this->create_subscription<robmovil_msgs::msg::MultiEncoderTicks>(
			"/robot/encoders", rclcpp::QoS(10),
			std::bind(&OmniOdometry::on_encoder_ticks, this, std::placeholders::_1));

	pub_vel_[0] = this->create_publisher<std_msgs::msg::Float64>(
			"/robot/front_left_wheel/cmd_vel", rclcpp::QoS(10));
	pub_vel_[1] = this->create_publisher<std_msgs::msg::Float64>(
			"/robot/front_right_wheel/cmd_vel", rclcpp::QoS(10));
	pub_vel_[2] = this->create_publisher<std_msgs::msg::Float64>(
			"/robot/rear_left_wheel/cmd_vel", rclcpp::QoS(10));
	pub_vel_[3] = this->create_publisher<std_msgs::msg::Float64>(
			"/robot/rear_right_wheel/cmd_vel", rclcpp::QoS(10));
	
	pub_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>(
			"/robot/odometry", rclcpp::QoS(10));
	
	tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void OmniOdometry::on_velocity_cmd(const geometry_msgs::msg::Twist::SharedPtr twist)
{
	// definiremos las velocidades w_i acorde al sistema de ecuaciones 20 del paper
	double vx = twist->linear.x,
	       vy = twist->linear.y,
	       w  = twist->angular.z;

	// publicamos cada velocidad para cada rueda
	// T_for es la matriz de transformación para cinemática directa/forward
	std_msgs::msg::Float64 msg_salida;
	for (int i = 0; i < WHEEL_COUNT; i++) {
		msg_salida.data = (vx * T_for[i][0] + vy * T_for[i][1] + w * T_for[i][2]);
		msg_salida.data /= WHEEL_RADIUS;

		pub_vel_[i]->publish(msg_salida);
	}
}

void OmniOdometry::on_encoder_ticks(const robmovil_msgs::msg::MultiEncoderTicks::SharedPtr encoder)
{
	// La primera vez que llega un mensaje de encoders
	// inicializo las variables de estado.
	if (!ticks_initialized_) {
		ticks_initialized_ = true;
		for (int i = 0; i < WHEEL_COUNT; i++)
			last_ticks_[i] = encoder->ticks[i];
		last_ticks_time = encoder->header.stamp;
		return;
	}

	/* Utilizar este delta de tiempo entre momentos */
	rclcpp::Time current_time(encoder->header.stamp);

	double vx = 0, vy = 0, w = 0,
	       wheel_angular_vel;

	for (int i = 0; i < WHEEL_COUNT; i++) {
		// convertimos la diferencia de ticks de encoder a radianes y dividimos por el tiempo transcurrido, para obtener la velocidad angular de la rueda
		wheel_angular_vel = ticks_to_rad_per_s * (encoder->ticks[i] - last_ticks_[i]);
		// calculamos las distintas velocidades del robot usando la matriz de
		// transformación de cinemática inversa definida en la ecuación 21
		vx += wheel_angular_vel * T_inv[0][i];
		vy += wheel_angular_vel * T_inv[1][i];
		w  += wheel_angular_vel * T_inv[2][i];
	}
	// vx = (radio/4.0) * T * vector_w, y así para vy y w
	vx /= 4.0;
	vy /= 4.0;
	w /= (4.0 * L);
	
	// considerando que tenemos vx y vy en el marco del robot, transformamos al marco global usando la transformación definida en la diapo 30 de la clase 06
	x_ += (vx * cos(theta_) - vy * sin(theta_));
	y_ += (vx * sin(theta_) + vy * cos(theta_));
	theta_ += w;
	theta_ = angles::normalize_angle(theta_);

	// Construir el mensaje odometry utilizando el esqueleto siguiente:
	nav_msgs::msg::Odometry msg;

	msg.header.stamp = encoder->header.stamp;
	msg.header.frame_id = "odom";
	msg.child_frame_id = "base_link";

	msg.pose.pose.position.x = x_;
	msg.pose.pose.position.y = y_;
	msg.pose.pose.position.z = 0;

	tf2::Quaternion q;
	q.setRPY(0, 0, theta_);	// roll, pitch, yaw
	msg.pose.pose.orientation = tf2::toMsg(q);

	msg.twist.twist.linear.x = vx;
	msg.twist.twist.linear.y = vy;
	msg.twist.twist.linear.z = 0;

	msg.twist.twist.angular.x = 0;
	msg.twist.twist.angular.y = 0;
	msg.twist.twist.angular.z = w;

	pub_odometry_->publish( msg );

	// Actualizo las variables de estado

	for (int i = 0; i < WHEEL_COUNT; i++)
		last_ticks_[i] = encoder->ticks[i];
	last_ticks_time = current_time;

	/* Mando tambien un transform usando TF */

	geometry_msgs::msg::TransformStamped t;
	t.header.stamp = current_time;
	t.header.frame_id = "odom";
	t.child_frame_id = "base_link";

	t.transform.translation.x = msg.pose.pose.position.x;
	t.transform.translation.y = msg.pose.pose.position.y;
	t.transform.translation.z = msg.pose.pose.position.z;
	t.transform.rotation = msg.pose.pose.orientation;

	tf_broadcaster_->sendTransform(t);

}
