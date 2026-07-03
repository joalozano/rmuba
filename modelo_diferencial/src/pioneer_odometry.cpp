#include "pioneer_odometry.h"
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

using namespace robmovil;

#define WHEEL_BASELINE 0.331
#define WHEEL_RADIUS 0.0975
#define ENCODER_TICKS 500.0

// velocidad = (R_x + R_th*b)/r
inline double vel_lado(double v_lineal, double v_angular)
{return ((v_lineal + (v_angular*(WHEEL_BASELINE/2)))/WHEEL_RADIUS);}
// dist = pi·D·(delta/N)·G, G = 1 en este caso
inline double dist_lado(double delta_lado)
{return (M_PI*(2*WHEEL_RADIUS)*(delta_lado)/ENCODER_TICKS);}

PioneerOdometry::PioneerOdometry() : Node("nodeOdometry"), x_(0), y_(0), theta_(0), ticks_initialized_(false)
{
  // Nos suscribimos a los comandos de velocidad en el tópico "/robot/cmd_vel" de tipo geometry_msgs::Twist
  twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(10), std::bind(&PioneerOdometry::on_velocity_cmd, this, std::placeholders::_1));

  vel_pub_left_ = this->create_publisher<std_msgs::msg::Float64>("/robot/left_wheel/cmd_vel", rclcpp::QoS(10));
  vel_pub_right_ = this->create_publisher<std_msgs::msg::Float64>("/robot/right_wheel/cmd_vel", rclcpp::QoS(10));

  encoder_sub_ =  this->create_subscription<robmovil_msgs::msg::EncoderTicks>("/robot/encoders", rclcpp::QoS(10), std::bind(&PioneerOdometry::on_encoder_ticks, this, std::placeholders::_1));
  
  pub_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("/robot/odometry", rclcpp::QoS(10));
  
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void PioneerOdometry::on_velocity_cmd(const geometry_msgs::msg::Twist::SharedPtr twist)
{
  /** Completar los mensajes de velocidad vLeft y vRight*/
     
  double vLeft = vel_lado(twist->linear.x, -twist->angular.z),
         vRight = vel_lado(twist->linear.x, twist->angular.z);

  // publish left velocity
  {
    std_msgs::msg::Float64 msg;
    msg.data = vLeft;

    vel_pub_left_->publish(msg);
  }

  // publish right velocity
  {
    std_msgs::msg::Float64 msg;
    msg.data = vRight;

    vel_pub_right_->publish(msg);
  }
}

void PioneerOdometry::on_encoder_ticks(const robmovil_msgs::msg::EncoderTicks::SharedPtr encoder)
{
  // La primera vez que llega un mensaje de encoders
  // inicializo las variables de estado.
  if (!ticks_initialized_) {
    ticks_initialized_ = true;
    last_ticks_left_ = encoder->ticks_left.data;
    last_ticks_right_ = encoder->ticks_right.data;
    last_ticks_time = encoder->header.stamp;
    return;
  }

  int32_t delta_ticks_left = encoder->ticks_left.data - last_ticks_left_;
  int32_t delta_ticks_right = encoder->ticks_right.data - last_ticks_right_;

  // calcular el desplazamiento relativo: delta_x, delta_y, delta_theta

  /* Utilizar este delta de tiempo entre momentos */
  rclcpp::Time current_time(encoder->header.stamp);
  double delta_t = (current_time - last_ticks_time).seconds();

  if (delta_t <= 0)
	  return;

  double dist_izq, dist_der, dist, delta_th, delta_x, delta_y;
  dist_izq = dist_lado(delta_ticks_left);
  dist_der = dist_lado(delta_ticks_right);
  dist = (dist_izq + dist_der)/2;
  delta_th = (dist_der - dist_izq)/(WHEEL_BASELINE);

  /** Utilizar variables globales x_, y_, theta_ definidas en el .h */
  delta_x = dist * cos(theta_ + delta_th/2);
  delta_y = dist * sin(theta_ + delta_th/2);
  x_ += delta_x;
  y_ += delta_y;
  theta_ += delta_th;
  theta_ = atan2(sin(theta_), cos(theta_)); // normalizamos el ángulo

  // Construir el mensaje odometry utilizando el esqueleto siguiente:
  nav_msgs::msg::Odometry msg;

  msg.header.stamp = encoder->header.stamp;
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_link";

  msg.pose.pose.position.x = x_;
  msg.pose.pose.position.y = y_;
  tf2::Quaternion q;
  q.setRPY(0, 0, theta_);  // roll, pitch, yaw
  msg.pose.pose.orientation = tf2::toMsg(q);

  msg.twist.twist.linear.x = dist/delta_t; // acá publicaría la velocidad, suponiendo que es necesario
  msg.twist.twist.angular.z = delta_th/delta_t;

  pub_odometry_->publish( msg );

  // Actualizo las variables de estado

  last_ticks_left_ = encoder->ticks_left.data;
  last_ticks_right_ = encoder->ticks_right.data;
  last_ticks_time = current_time;

  /* Mando tambien un transform usando TF */

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = encoder->header.stamp;
  t.header.frame_id = "odom";
  t.child_frame_id = "base_link";
  t.transform.translation.x = msg.pose.pose.position.x;
  t.transform.translation.y = msg.pose.pose.position.y;
  t.transform.rotation = msg.pose.pose.orientation;

  tf_broadcaster_->sendTransform(t);
}
