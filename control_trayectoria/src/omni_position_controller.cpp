#include "omni_position_controller.h"

#define KX 1.0
#define KY 0.9
#define KTH 1.1

#define LOOKAHEAD 0.3

#define TOL 0.02
#define ATOL 0.08


OmniPositionController::OmniPositionController() :
	OmniTrajectoryFollower(), x_(0.0), y_(0.0), theta_(0), current_wp_(0),
	tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
	rclcpp::QoS qos_profile(rclcpp::KeepLast(50));
	qos_profile.reliable();
	qos_profile.durability_volatile();
	this->declare_parameter("base_link_frame", std::string("base_link")); // por default, usar odometría
	this->get_parameter("base_link_frame", base_frame_);

	goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

	while ((this->now().nanoseconds() == 0));
	position_timer = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&OmniPositionController::getCurrentPose, this));
}

void OmniPositionController::getCurrentPose()
{
	try {
		auto transform = tf_buffer_.lookupTransform("map", base_frame_, tf2::TimePointZero);
		// para depurar: obtenemos la posición gt del robot y la comparamos con nuestra estimación
		auto gt = tf_buffer_.lookupTransform("map", "base_link_gt", tf2::TimePointZero);
		x_ = transform.transform.translation.x;
		y_ = transform.transform.translation.y;
		theta_ = tf2::getYaw(transform.transform.rotation);
		RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 333,
		"%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
		this->get_clock()->now().seconds(), gt.transform.translation.x, gt.transform.translation.y, tf2::getYaw(gt.transform.rotation), x_, y_, theta_);

	} catch (tf2::TransformException &ex) {
		RCLCPP_WARN(this->get_logger(), "No se pudo obtener posición.");
	}
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
	// buscamos el próximo punto más cercano que está a distancia lookahead, considerando que el arreglo tiene la trayectoria ordenada por tiempo y, por ende, por posiciones cercanas.
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

	// para debugging, comparamos posición a la posición deseada
	/*RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 333,
			"t: %.2f, gx: %.2f, gy: %.2f, gth: %.2f, x_: %.2f, y_: %.2f, th_: %.2f", t.seconds(), gx, gy, ga, x_, y_, theta_);*/
	// ahora en un formato csv-friendly, incluyendo las velocidades
	/*RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 333,
			"%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", t.seconds(), gx, gy, ga, x_, y_, theta_,vx,vy,w);*/


	return true;
}
