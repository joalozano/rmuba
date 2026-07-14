#include "omni_position_controller.h"

#define KX 1.0
#define KY 0.9
#define KTH 1.1

#define LOOKAHEAD 0.3

#define TOL 0.02
#define ATOL 0.08


OmniPositionController::OmniPositionController() :
	OmniTrajectoryFollower(), x_(0.0), y_(0.0), theta_(0), current_wp_(0)
{
	rclcpp::QoS qos_profile(rclcpp::KeepLast(50));
	qos_profile.reliable();
	qos_profile.durability_volatile();

	goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", rclcpp::QoS(10));
	odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/robot/odometry", rclcpp::QoS(10), std::bind(&OmniPositionController::getCurrentPoseFromOdometry, this, std::placeholders::_1));
}

void OmniPositionController::getCurrentPoseFromOdometry(const nav_msgs::msg::Odometry& odom)
{
	x_ = odom.pose.pose.position.x;
	y_ = odom.pose.pose.position.y;

	theta_ = tf2::getYaw(odom.pose.pose.orientation);
}

bool OmniPositionController::getPursuitBasedGoal(double& gx, double& gy, double& ga)
{
	const auto& traj = getTrajectory();
	size_t wp_count = traj.points.size();
	if (wp_count == 0)
		return false;
	
	//RCLCPP_INFO(this->get_logger(), "wp_count: %lu", wp_count);
	
	double ca = tf2::getYaw(traj.points[current_wp_].transform.rotation);
	// validamos que ya estamos apuntando a donde se espera antes de buscar el próximo waypoint en el plano x,y
	if (fabs(angles::normalize_angle(ca - theta_)) < ATOL) {
		for (size_t i = current_wp_; i < wp_count; i++)
		{
			double px = traj.points[i].transform.translation.x;
			double py = traj.points[i].transform.translation.y;

			double d = hypot(px - x_, py - y_);

			if (d >= LOOKAHEAD)
			{
				current_wp_ = i;
				break;
			}
		}
	}

	gx = traj.points[current_wp_].transform.translation.x;
	gy = traj.points[current_wp_].transform.translation.y;
	ga = tf2::getYaw(traj.points[current_wp_].transform.rotation);
	
	double dist_angular = fabs(angles::normalize_angle(ga - theta_));
	double dist_linear = hypot(gx - x_, gy - y_);

	return dist_linear >= TOL || dist_angular >= ATOL;
}

bool OmniPositionController::control(const rclcpp::Time& t, double& vx, double& vy, double& w)
{
	double gx, gy, ga;
	// inicializamos la siguiente pose objetivo, o cortamos con el seguimiento si ya se terminó la trayectoria
	if (!getPursuitBasedGoal(gx, gy, ga))
		return false;
	
	// publicamos la pose objetivo a RViz2
	publishCurrentGoal(t, gx, gy, ga);

	// calculamos el error inicial, y luego transformamos al marco del goal, acorde a la diapositiva 13 de la clase 8
	double dx = gx - x_;
	double dy = gy - y_;

	double etheta_r = angles::normalize_angle(ga - theta_);

	// error en frame del robot
	double ex_r = cos(theta_) * dx + sin(theta_) * dy;
	double ey_r = -sin(theta_) * dx + cos(theta_) * dy;

	// controlamos las velocidades acorde al error y unas variables de ajuste K
	if (hypot(ex_r, ey_r) < TOL) {
		vx = 0.0; // estamos cerca en el plano x,y de la posición deseada, dejamos al robot quieto y rotamos
		vy = 0.0;
	} else {
		vx = KX * ex_r;
		vy = KY * ey_r;
	}
	w  = KTH * etheta_r;

	// imprimimos las velocidades en un formato csv-friendly, con una espera de 100ms
	// idem con posiciones esperadas y actuales
	RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 333,
			"%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", t.seconds(), gx, gy, ga, x_, y_, theta_,vx,vy,w);


	return true;
}
