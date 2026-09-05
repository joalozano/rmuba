#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include "localizer.h"

robmovil_ekf::Localizer::Localizer() :
	Node("localizer_node"), u(3), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
	u(1) = u(2) = u(3) = 0;
	tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

	// Get node parameters
	this->declare_parameter("only_prediction", false);
	this->declare_parameter("min_landmark_size", 2);
	this->get_parameter("only_prediction", only_prediction);
	this->get_parameter("min_landmark_size", min_landmark_size);
	
	landmark_sub = this->create_subscription<robmovil_msgs::msg::LandmarkArray>("/landmarks", rclcpp::QoS(10), std::bind(&Localizer::on_landmark_array, this, std::placeholders::_1));
	odo_sub = this->create_subscription<nav_msgs::msg::Odometry>("/robot/odometry", rclcpp::QoS(10), std::bind(&Localizer::on_odometry, this, std::placeholders::_1));
	// a diferencia de durante la cursada, el mapa de postes ya está armado, y no debe crearse en el primer sensado
	post_sub = this->create_subscription<geometry_msgs::msg::PoseArray>("/posts", rclcpp::QoS(10), std::bind(&Localizer::on_posts, this, std::placeholders::_1));

	pose_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/pose", rclcpp::QoS(10));
	
	while ((this->now().nanoseconds() == 0));
	t = this->now();
	prediction_timer = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Localizer::prediction_event, this));
}

/**
 * Prediccion hecha periódicamente, en caso de que no haya habido sensado por cierto tiempo
 */

void robmovil_ekf::Localizer::prediction_event() //const rclcpp::TimerEvent& event)
{
	//RCLCPP_INFO(this->get_logger(), "Prediction Event");
	
	prediction(this->now());
	
	/* publicar estimado actual */
	publish_estimate(this->now());
}

void robmovil_ekf::Localizer::prediction(const rclcpp::Time& now)
{
	//RCLCPP_INFO(this->get_logger(), "Prediction");

	/* avanzar el tiempo del filtro desde el tiempo anterior al actual */
	advance_time(now);
	//RCLCPP_INFO(this->get_logger(), "t is now: %.3f", t.seconds());
	
	/* hacer prediccion */
	ekf.timeUpdateStep(u);

	LocalizerEKF::Vector x = ekf.getX();
	LocalizerEKF::Matrix P = ekf.calculateP();
	
	//RCLCPP_INFO_STREAM(this->get_logger(), "Predicted estimate: " << x);
	//RCLCPP_INFO_STREAM(this->get_logger(), "Predicted covariance: " << P);
}

void robmovil_ekf::Localizer::on_landmark_array(const robmovil_msgs::msg::LandmarkArray::SharedPtr msg)
{
	//RCLCPP_INFO(this->get_logger(), "Only prediction: %d", only_prediction); // Prints 0 or 1
	if (only_prediction || (int)msg->landmarks.size() < min_landmark_size || !map_received_)
	  return;
	
	//RCLCPP_INFO(this->get_logger(), "on_landmark_array: %3.f", t.seconds());
	
	/* Se apaga el timer que predice periodicamente dado que
	 * se predice y actualiza dadas las mediciones recibidas */
	prediction_timer->cancel();

	// Prediccion avanzando el tiempo al momento del sensado
	prediction(msg->header.stamp);

	/* hacer update(s) */
	for (size_t i = 0; i < msg->landmarks.size(); i++)
	{
	  LocalizerEKF::Vector z(2);
	  
	  z(1) = msg->landmarks[i].range;
	  z(2) = msg->landmarks[i].bearing;
	  
	  //RCLCPP_INFO_STREAM(this->get_logger(), "Measurement update with: " << z);
	  
	  if (ekf.set_measure(z)) {
	    ekf.measureUpdateStep(z);
	    //RCLCPP_INFO_STREAM(this->get_logger(), "Measure updated estimate: " << ekf.getX());
	    //RCLCPP_INFO_STREAM(this->get_logger(), "Measure updated covariance: " << ekf.calculateP());
	  }
	}

	/* publicar estimado actual */
	publish_estimate(msg->header.stamp);

	/* prendo el timer de prediccion, por si no llega un sensado a tiempo y asi todavia estaria prediciendo hasta que vuelva a haber un sensado */
	prediction_timer->reset();
}

void robmovil_ekf::Localizer::on_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	u(1) = msg->twist.twist.linear.x;
	u(2) = msg->twist.twist.linear.y;
	u(3) = msg->twist.twist.angular.z;
	//RCLCPP_INFO(this->get_logger(), "Received linear velocity X: %f", u(1));
}

void robmovil_ekf::Localizer::on_posts(const geometry_msgs::msg::PoseArray::SharedPtr posts)
{
	ekf.set_map(posts);
	map_received_ = true;
	//RCLCPP_INFO(this->get_logger(), "Mapa de %zu postes cargado (transformado a base_link)", posts->poses.size());

}

void robmovil_ekf::Localizer::advance_time(const rclcpp::Time& now)
{
	double delta_t = (now - t).seconds();

	if (delta_t < 0) {
	  RCLCPP_WARN_STREAM(this->get_logger(), "Negative delta_t, ignorado: " << delta_t);
	  return;
	}

	ekf.set_delta_t(delta_t);
	t = now;
	//RCLCPP_INFO(this->get_logger(), "Time advanced by: %f", delta_t);
}

void robmovil_ekf::Localizer::publish_estimate(const rclcpp::Time& now)
{
	LocalizerEKF::Vector x = ekf.getX();
	LocalizerEKF::Matrix P = ekf.calculateP();

	geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
	pose_msg.header.stamp = now;
	pose_msg.header.frame_id = "map";

	/* build covariance */
	pose_msg.pose.covariance.fill(0.0);

	/* XY/XY covariance */
	for (int i = 0; i < 2; i++)
	  for (int j = 0; j < 2; j++)
	    pose_msg.pose.covariance[i * 6 + j] = P(i + 1, j + 1);

	/* XY/theta covariance */
	pose_msg.pose.covariance[0 * 6 + 5] = P(1,3);
	pose_msg.pose.covariance[1 * 6 + 5] = P(2,3);

	/* theta/XY covariance */
	pose_msg.pose.covariance[5 * 6 + 0] = P(3,1);
	pose_msg.pose.covariance[5 * 6 + 1] = P(3,2);

	/* theta/theta covariance */
	pose_msg.pose.covariance[5 * 6 + 5] = P(3,3);

	/* pongo en -1 los elementos de z, y los dos angulos sin usar, dado que estoy trabajando en 2D */
	pose_msg.pose.covariance[2 * 6 + 2] = -1;
	pose_msg.pose.covariance[3 * 6 + 3] = -1;
	pose_msg.pose.covariance[4 * 6 + 4] = -1;

	/* set pose, acá se transforma del marco map a map_frame_, que generalmente será odom */
	tf2::Transform map_to_base_link;
	map_to_base_link.setOrigin(tf2::Vector3(x(1), x(2), 0));
	map_to_base_link.setRotation(tf2::Quaternion(tf2::Vector3(0,0,1),x(3)));
	tf2::toMsg(map_to_base_link, pose_msg.pose.pose);

	/* publish */
	pose_pub->publish(pose_msg);
	// publish TF transform: map -> base_link_ekf (directo, segun pide el enunciado)
	geometry_msgs::msg::TransformStamped map_to_base_link_msg;
	map_to_base_link_msg.header.stamp = now;
	map_to_base_link_msg.header.frame_id = "map";
	map_to_base_link_msg.child_frame_id = "base_link_ekf";
	map_to_base_link_msg.transform = tf2::toMsg(map_to_base_link);

	tf_broadcaster_->sendTransform(map_to_base_link_msg);
}
