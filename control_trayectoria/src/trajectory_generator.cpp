#include <rclcpp/rclcpp.hpp>

#include <robmovil_msgs/msg/trajectory.hpp>
#include <robmovil_msgs/msg/trajectory_point.hpp>

#include <nav_msgs/msg/path.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


void build_spline_trajectory(double, std::vector<std::vector<double>>&, robmovil_msgs::msg::Trajectory&, nav_msgs::msg::Path&);

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::QoS qos(rclcpp::KeepLast(1));
	qos.reliable();
	qos.transient_local();

	auto node = rclcpp::Node::make_shared("trajectory_generator");
	auto traj_pub = node->create_publisher<robmovil_msgs::msg::Trajectory>("/robot/trajectory", qos);
	auto path_pub = node->create_publisher<nav_msgs::msg::Path>("/ground_truth/target_path", 10);

	robmovil_msgs::msg::Trajectory traj_msg;
	traj_msg.header.stamp = node->now();
	traj_msg.header.frame_id = "odom";

	nav_msgs::msg::Path path_msg;
	path_msg.header.stamp = node->now();
	path_msg.header.frame_id = "odom";
	
	// generamos la trayectoria con splines a partir de puntos fijados por el usuario en launch.py
	// los datos almacenados son t, x, y, theta, de forma que por cada waypoint debe haber cuatro valores en spline_waypoints_data
	std::vector<double> spline_waypoints_data = node->declare_parameter<std::vector<double>>("spline_waypoints", {});
	
	// si no se definió algún waypoint o si falta algún dato para algún waypoint, arrojamos un error.
	if (spline_waypoints_data.size() == 0 || spline_waypoints_data.size() % 4 != 0) {
		RCLCPP_ERROR(node->get_logger(),"Trajectory waypoints data is not multiple of 4, it expects: time, position_x, position_y, orientation");
		return 1;
	}

	// guardamos los waypoints como una matriz de n_puntos x 4, donde wp[0] es t, wp[1] es x, etc
	std::vector<std::vector<double>> spline_waypoints;
	for (int i = 0; i < spline_waypoints_data.size(); i += 4) {
		std::vector<double> wp_data;
		for (int j = i; j < i + 4; j++) {
			double value = static_cast<double>(spline_waypoints_data[j]);
			wp_data.push_back(value);
		}

		spline_waypoints.push_back(wp_data);
	}

	// stepping nos permitirá agregar puntos en la trayectoria con mayor granularidad, acorde al "tamaño de pasos" en tiempo
	double stepping = node->declare_parameter("stepping", 0.1);
	
	build_spline_trajectory(stepping, spline_waypoints, traj_msg, path_msg);

	traj_pub->publish(traj_msg);
	path_pub->publish(path_msg);

	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}

// generamos trayectoria (y camino para RViz2) usando un spline cúbico, a partir de puntos definidos (para el TP: un cuadrado de cuatro metros de lado)
void build_spline_trajectory(double stepping, std::vector<std::vector<double>>& wpoints, robmovil_msgs::msg::Trajectory& trajectory_msg, nav_msgs::msg::Path& path_msg)
{
	int wpoint_count = wpoints.size();

	for (int i = 0; i < wpoint_count - 1; i++) {
		double ti = wpoints[i][0],
		       tf = wpoints[i + 1][0],
		       dt = tf - ti;
		double xi = wpoints[i][1],
		       yi = wpoints[i][2],
		       ai = wpoints[i][3];
		double xf = wpoints[i + 1][1],
		       yf = wpoints[i + 1][2],
		       af = wpoints[i + 1][3];
		/*
		// si hay un waypoint mal puesto (tiempo de llegada menor o igual al punto siguiente), cortamos la generación de la trayectoria; hace falta comunicar al usuario que hay un error
		if ((tf - ti) <= 0)
			break;
		*/
		// parámetros de curvatura: velocidades esperadas, calculadas de la forma más simple posible.
		double vxi = (xf - xi) / (tf - ti),
		       vyi = (yf - yi) / (tf - ti),
		       vai = (af - ai) / (tf - ti), // definidos vagamente porque para la trayectoria que queremos, el robot siempre girará a derecha; alternativamente, se puede hacer lo mismo que con vx y vy
		       vxf, vyf, vaf;
		// calculamos las velocidades finales usando las velocidades iniciales del próximo punto
		if (i < wpoint_count - 2) {
			vxf = (wpoints[i + 2][1] - xf) / (tf - ti);
			vyf = (wpoints[i + 2][2] - yf) / (tf - ti);
			vyf = (wpoints[i + 2][3] - af) / (tf - ti);
		} else {
			vxf = 0.; // últimos dos puntos, la velocidad final será quedarse quieto
			vyf = 0.;
			vaf = 0.;
		}

		// coeficientes para calcular x(t)
		double a0 = xi,
		       a1 = vxi,
		       a2 = (3/(dt*dt))*(xf - xi) - (2/dt)*vxi - vxf/dt,
		       a3 = (-2/(dt*dt*dt))*(xf - xi) + (vxf - vxi)/(dt*dt);
		// idem para y(t)
		double b0 = yi,
		       b1 = vyi,
		       b2 = (3/(dt*dt))*(yf - yi) - (2/dt)*vyi - vyf/dt,
		       b3 = (-2/(dt*dt*dt))*(yf - yi) + (vyf - vyi)/(dt*dt);
		// idem para theta(t), aunque sería posible calcular la dirección opuesta al origen con atan2(y - y_world,x - x_world)
		double c0 = ai,
		       c1 = vai,
		       c2 = (3/(dt*dt))*(af - ai) - (2/dt)*vai - vaf/dt,
		       c3 = (-2/(dt*dt*dt))*(af - ai) + (vaf - vai)/(dt*dt);
		
		// calculamos todos los puntos intermedios en la trayectoria, proveyendo posibles velocidades esperadas en cada punto
		for (double t = ti; t <= tf; t += stepping) {
			double t_offset = t - ti;
			// posiciones esperadas
			double x = a0 + a1 * (t_offset / dt) + a2 * ((t_offset * t_offset) / (dt * dt)) + a3 * ((t_offset * t_offset * t_offset) / (dt * dt * dt)),
			       y = b0 + b1 * (t_offset / dt) + b2 * ((t_offset * t_offset) / (dt * dt)) + b3 * ((t_offset * t_offset * t_offset) / (dt * dt * dt)),
			       a = c0 + c1 * (t_offset / dt) + c2 * ((t_offset * t_offset) / (dt * dt)) + c3 * ((t_offset * t_offset * t_offset) / (dt * dt * dt));
			
			// velocidades esperadas/derivadas primeras
			// sería útil incluir estos valores al calcular los comandos de velocidades en la función de control en omni_position_controller.cpp
			double vx = a1 / dt + 2. * a2 * (t_offset / (dt * dt)) + 3. * a3 * ((t_offset * t_offset) / (dt * dt * dt)),
			       vy = b1 / dt + 2. * b2 * (t_offset / (dt * dt)) + 3. * b3 * ((t_offset * t_offset) / (dt * dt * dt)),
			       va = c1 / dt + 2. * c2 * (t_offset / (dt * dt)) + 3. * c3 * ((t_offset * t_offset) / (dt * dt * dt));
			
			// se crean los waypoints de la trayectoria
			robmovil_msgs::msg::TrajectoryPoint point_msg;
			point_msg.time_from_start = rclcpp::Duration::from_seconds(t);
			
			point_msg.transform.translation.x = x;
			point_msg.transform.translation.y = y;
			tf2::Quaternion q;
			q.setRPY(0, 0, a);
			point_msg.transform.rotation = tf2::toMsg(q);

			point_msg.velocity.linear.x  = vx;
			point_msg.velocity.linear.y  = vy;
			point_msg.velocity.angular.z = va;

			trajectory_msg.points.push_back( point_msg );

			// construcción de una lista de poses, para visualizarlo en RViz2
			geometry_msgs::msg::PoseStamped stamped_pose_msg;
				  
			stamped_pose_msg.header.stamp = path_msg.header.stamp;
			stamped_pose_msg.header.frame_id = path_msg.header.frame_id;

			stamped_pose_msg.pose.position.x = x;
			stamped_pose_msg.pose.position.y = y;
			stamped_pose_msg.pose.orientation = tf2::toMsg(q);

			path_msg.poses.push_back(stamped_pose_msg);  
		}
	}
}
