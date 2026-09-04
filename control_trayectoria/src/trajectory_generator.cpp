#include <rclcpp/rclcpp.hpp>

#include <robmovil_msgs/msg/trajectory.hpp>
#include <robmovil_msgs/msg/trajectory_point.hpp>

#include <nav_msgs/msg/path.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <angles/angles.h>

// este módulo es una modificación del módulo usado dentro de la cursada. Se eliminaron las trayectorias time-based y las sinoidales, para proveer únicamente trayectorias basadas en splines

void build_spline_trajectory(double, std::vector<std::vector<double>>&, robmovil_msgs::msg::Trajectory&, nav_msgs::msg::Path&);
inline void compute_waypoint_velocities(std::vector<std::vector<double>>& wpoints, int wpoint_count);

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
	if (wpoint_count <= 1)
		return;
	
	// parámetros de curvatura: velocidades esperadas en el punto actual y el siguiente. obtenidos experimentalmente; una alternativa sería obtener (var1 - var0) / delta_t
	constexpr double n0 = 0.08,
			  		 n1 = 0.03;

	for (int i = 0; i < wpoint_count - 1; i++) {
		double ti = wpoints[i][0],
		       tf = wpoints[i + 1][0],
		       dt = tf - ti;

		// si hay un waypoint mal puesto (tiempo de llegada menor o igual al punto siguiente), cortamos la generación de la trayectoria; hace falta comunicar al usuario que hay un error
		if ((tf - ti) <= 0)
			continue;

		// posiciones (x,y,angulo) iniciales y finales en un tramo específico
		double xi = wpoints[i][1],
		       yi = wpoints[i][2],
		       ai = wpoints[i][3];
		double xf = wpoints[i + 1][1],
		       yf = wpoints[i + 1][2],
		       af = wpoints[i + 1][3];
		double dangular = angles::normalize_angle(af - ai); // normalizamos la distancia entre ángulos, así se mantiene el rango en [-pi,pi]


		// coeficientes para calcular x(t), que implementa lo visto en la clase de generación de trayectorias basada en splines para un robot holonómico
		double a0 = xi,
		       a1 = n0,
		       a2 = (3/(dt*dt))*(xf - xi) - (2/dt)*n0 - n1/dt,
		       a3 = (-2/(dt*dt*dt))*(xf - xi) + (n1 + n0)/(dt*dt);
		// idem para y(t)
		double b0 = yi,
		       b1 = n0,
		       b2 = (3/(dt*dt))*(yf - yi) - (2/dt)*n0 - n1/dt,
		       b3 = (-2/(dt*dt*dt))*(yf - yi) + (n1 + n0)/(dt*dt);
		// idem para theta(t)
		double c0 = ai,
		       c1 = n0,
		       c2 = (3/(dt*dt))*(dangular) - (2/dt)*n0 - n1/dt,
		       c3 = (-2/(dt*dt*dt))*(dangular) + (n1 + n0)/(dt*dt);
		
		// calculamos todos los puntos intermedios en la trayectoria, proveyendo posibles velocidades esperadas en cada punto
		for (double t = ti; t < tf; t += stepping) {
			double t_offset = t - ti;
			// posiciones esperadas
			double x = a0 + a1 * t_offset + a2 * (t_offset * t_offset) + a3 * (t_offset * t_offset * t_offset),
			       y = b0 + b1 * t_offset + b2 * (t_offset * t_offset) + b3 * (t_offset * t_offset * t_offset),
			       a = c0 + c1 * t_offset + c2 * (t_offset * t_offset) + c3 * (t_offset * t_offset * t_offset);
			
			// velocidades esperadas/derivadas primeras
			// sería útil incluir estos valores al calcular los comandos de velocidades en la función de control en omni_position_controller.cpp
			double vx = a1 + 2. * a2 * t_offset + 3. * a3 * (t_offset * t_offset),
			       vy = b1 + 2. * b2 * t_offset + 3. * b3 * (t_offset * t_offset),
			       va = c1 + 2. * c2 * t_offset + 3. * c3 * (t_offset * t_offset);
			
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
