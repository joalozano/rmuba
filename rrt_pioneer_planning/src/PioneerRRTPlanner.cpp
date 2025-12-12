#include "PioneerRRTPlanner.h"

#include <angles/angles.h>
#include <queue>
#include <map>
#include <vector>
#include <random>

typedef robmovil_planning::PioneerRRTPlanner::SpaceConfiguration SpaceConfiguration;

robmovil_planning::PioneerRRTPlanner::PioneerRRTPlanner()
: RRTPlanner(0, 0)
{ 
	goal_bias_ = this->declare_parameter<double>("goal_bias", 0.6);
	int it_tmp = this->declare_parameter<int>("max_iterations", 200000);
	max_iterations_ = it_tmp >= 0 ? it_tmp : 20000;
	Vx_step_ = this->declare_parameter<double>("linear_velocity_stepping", 0.05);
	Wz_step_ = this->declare_parameter<double>("angular_velocity_stepping", 0.025);
}

SpaceConfiguration robmovil_planning::PioneerRRTPlanner::defineStartConfig()
{
	/* Se toma la variable global de la pose incial y se la traduce a SpaceConfiguration */
	//return SpaceConfiguration( { starting_pose_.getOrigin().getX(), starting_pose_.getOrigin().getY(), tf::getYaw(starting_pose_.getRotation()) } );
	return SpaceConfiguration( { starting_pose_.pose.position.x, starting_pose_.pose.position.y, tf2::getYaw(starting_pose_.pose.orientation) } );
}

SpaceConfiguration robmovil_planning::PioneerRRTPlanner::defineGoalConfig()
{
	/* Se toma la variable global de la pose del goal y se la traduce a SpaceConfiguration */
	//return SpaceConfiguration( { goal_pose_.getOrigin().getX(), goal_pose_.getOrigin().getY(), tf::getYaw(goal_pose_.getRotation()) } );
	return SpaceConfiguration( { goal_pose_.pose.position.x, goal_pose_.pose.position.y, tf2::getYaw(goal_pose_.pose.orientation) } );
}

#define DIST_C 0.5
SpaceConfiguration robmovil_planning::PioneerRRTPlanner::generateRandomConfig()
{	
	
		/* COMPLETAR: Deben retornar una configuracion aleatoria dentro del espacio de busqueda.
		 * 
		 * ATENCION: - Tener encuenta el valor de la variable global goal_bias_ 
		 *					 - Pueden utilizar la funcion randBetween(a,b) para la generacion de numeros aleatorios 
		 *					 - Utilizar las funciones getOriginOfCell() y la informacion de la grilla para establecer el espacio de busqueda:
		 *								grid_->info.width, grid_->info.height, grid_->info.resolution */	
		 
		double x, y, th;
	 
	if (randBetween(0.0,1.0) > goal_bias_) {
		double x_m, y_m;
		if (getOriginOfCell(grid_->info.width - 1, grid_->info.height - 1, x_m, y_m)) {
			x = randBetween(0.0, x_m);
			y = randBetween(0.0, y_m);
			th = randBetween(0.0, 2.0*M_PI);
		}
	} else { // TMP definicion de punto cercano al goal
		SpaceConfiguration goal = defineGoalConfig();
		x = randBetween(goal.get(0) - DIST_C*grid_->info.resolution, goal.get(0) + DIST_C*grid_->info.resolution);
		y = randBetween(goal.get(1) - DIST_C*grid_->info.resolution, goal.get(1) + DIST_C*grid_->info.resolution);
		th = angles::normalize_angle(randBetween(goal.get(2) - DIST_C, goal.get(2) + DIST_C));
	}
		
		SpaceConfiguration rand( {x, y, th} );

		return rand;
}

#define dist_ang(a,b) std::abs( angles::shortest_angular_distance(a.get(2), b.get(2)) )
double robmovil_planning::PioneerRRTPlanner::distancesBetween(const SpaceConfiguration& c1, const SpaceConfiguration& c2)
{
	/* COMPLETAR: Funcion auxiliar recomendada para evaluar la distancia entre configuraciones
	 * 
	 * ATENCION: Utilizar abs( angles::shortest_angular_distance(c1.get(2), c2.get(2)) )
	 *					 para medir la distancia entre las orientaciones */
	
	//double dist_ori = dist_ang(c1, c2);
	double diff_x = c1.get(0) - c2.get(0),
		 diff_y = c1.get(1) - c2.get(1);
	double dist_euc = std::sqrt(diff_x*diff_x + diff_y*diff_y);
	//return dist_euc + dist_ori; // alternativa integrando ambas?
	return dist_euc;
}

// número de posibles velocidades lineales y angulares, para saber la cantidad máxima de hijos de un nodo
#define NR_V_X 5
#define NR_W_Z 5
// mini núcleo
#define try_and_replace(newconfig, min_distance, config) \
	double d = distancesBetween(newconfig, rand_config_); \
	if (d < min_distance) { \
		min_distance = d; \
		config = newconfig; \
	}
SpaceConfiguration robmovil_planning::PioneerRRTPlanner::nearest()
{
	/* COMPLETAR: Retornar configuracion mas cercana a la aleatoria (rand_config_). DEBE TENER HIJOS LIBRES
	 * 
	 * ATENCION: - Deberan recorrer la variable global graph_ la cual contiene los nodos del arbol
	 *						 generado hasta el momento como CLAVES DEL DICCIONARIO
	 *					 - Se recomienda establecer una relacion de distancia entre configuraciones en distancesBetween() 
	 *						 y utilizar esa funcion como auxiliar */
	
	SpaceConfiguration nearest;
	double min_distance = std::numeric_limits<double>::max(); 
	
	for(const auto& config : graph_ ) {
		if (config.second.size() < NR_W_Z * NR_V_X) {
			try_and_replace(config.first, min_distance, nearest);
		}
	}
	
	/*if (min_distance == std::numeric_limits<double>::max()) // verificamos si no encontramos alguna configuración libre y posible
		nearest = defineStartConfig; // opcional, actualmente devolvemos la configuración 0,0,0 */

	return nearest;
}

#define TOL 1.0
SpaceConfiguration robmovil_planning::PioneerRRTPlanner::steer()
{	
	/* COMPLETAR: Retornar una nueva configuracion a partir de la mas cercana near_config_.
	 *						La nueva configuracion debe ser ademas la mas cercana a rand_config_ de entre las posibles.
	 * 
	 * ATENCION: - Utilizar las variables globales Vx_step_ , Wz_step_ para la aplicaciones de las velocidades
	 *					 - Pensar en la conversion de coordenadas polares a cartesianas al establecer la nueva configuracion
	 *					 - Utilizar angles::normalize_angle() */
	
	// Conjunto de steers ya ocupados en la configuracion near_config_
	const std::list<SpaceConfiguration> occupied_steerings = graph_[near_config_];
	std::vector<SpaceConfiguration> free_steerings;
	
	if (occupied_steerings.size() < NR_V_X * NR_W_Z) {
		 double v_c[NR_V_X] = {-0.5*Vx_step_, 0, Vx_step_, 1.75*Vx_step_,2.75*Vx_step_}, // posibles velocidades lineales y angulares que usaremos para calcular posiciones candidatas
				w_c[NR_W_Z] = {-1.1*Wz_step_, -0.75*Wz_step_, 0, 0.75*Wz_step_, 1.1*Wz_step_};
		/* RECOMENDACION: Establecer configuraciones posibles en free_steerings y calcular la mas cercana a rand_config_ */
		
		 for (int i = 0; i < NR_V_X; i++) {
			for (int j = 0; j < NR_W_Z; j++) {
				double x_p, y_p, th_p;
				x_p = near_config_.get(0) + std::cos(near_config_.get(2))*v_c[i];
				y_p = near_config_.get(1) + std::sin(near_config_.get(2))*v_c[i];
				th_p = angles::normalize_angle(near_config_.get(2) + w_c[j]);
				SpaceConfiguration s_posible({x_p, y_p, th_p});
				bool duplicado = false;
				for (const auto& s : occupied_steerings) { // considerar si hace falta validar que no duplico configuraciones
					if (distancesBetween(s_posible, s) <= TOL) {
						duplicado = true;
						break;
					}
				}
				if (!duplicado)
					free_steerings.push_back(s_posible);
			}
		 }
	}
	
	SpaceConfiguration steer;
	if (!free_steerings.empty()) {
		double min_distance = std::numeric_limits<double>::max();
		for (const auto& s : free_steerings) {
			try_and_replace(s, min_distance, steer);
		}
	} else { // no hay direcciones libres, damos como default near_config_
		/*double x_st = near_config_.get(0),
			   y_st = near_config_.get(1)
			   th_st = near_config_.get(2);*/
		steer = near_config_;
	}
	
	return steer;
}

bool robmovil_planning::PioneerRRTPlanner::isFree()
{
	/* COMPLETAR: Utilizar la variable global new_config_ para establecer si existe un area segura alrededor de esta */

	double x = new_config_.get(0), y = new_config_.get(1),
		   res = grid_->info.resolution,
		   x1,y1;
	unsigned int k, l;
	bool valido;
	for (int i = -1; i < 2; i++) {
		for (int j = -1; j < 2; j++) {
			x1 = x + i * res;
			y1 = y + j * res;
			valido = getCellOfPosition(x1, y1, k, l);
			if (!valido || isCellOccupy(k, l))
				return false;
		}
	}
	
	return true;
}

bool robmovil_planning::PioneerRRTPlanner::isGoalAchieve()
{
	
	/* COMPLETAR: Comprobar si new_config_ se encuentra lo suficientemente cerca del goal.
	 * 
	 * ATENCION: Utilizar abs( angles::shortest_angular_distance(c1.get(2), c2.get(2)) )
	 *					 para medir la distancia entre las orientaciones */
	SpaceConfiguration goal = defineGoalConfig();
	return distancesBetween(new_config_,goal) <= 0.1 && dist_ang(new_config_,goal) <= 1.5*M_PI;
}



/* DESDE AQUI YA NO HACE FALTA COMPLETAR */



bool robmovil_planning::PioneerRRTPlanner::isValid()
{ return true; }

void robmovil_planning::PioneerRRTPlanner::notifyTrajectory(robmovil_msgs::msg::Trajectory& result_trajectory,
																														const SpaceConfiguration& start, const SpaceConfiguration& goal, 
																														std::map<SpaceConfiguration, SpaceConfiguration>& came_from) const
{
	std::vector<SpaceConfiguration> path;
	SpaceConfiguration current = goal;
	
	path.push_back(current);
	
	while(current != start)
	{
		current = came_from[current];
		path.push_back(current);
	}
	
	result_trajectory.header.stamp = this->now();
	result_trajectory.header.frame_id = "map";
	
	rclcpp::Duration t_from_start = rclcpp::Duration::from_seconds(0.0);
	rclcpp::Duration delta_t = rclcpp::Duration::from_seconds(1.0);

	/* Se recorre de atras para adelante */
	for (auto it = path.rbegin(); it != path.rend(); ++it) {		
		double config_x = it->get(0);
		double config_y = it->get(1);
		double config_theta = it->get(2);
		
		tf2::Transform wp_odom_ref;
		wp_odom_ref.setOrigin(tf2::Vector3(config_x, config_y, 0.0));
		tf2::Quaternion delta_q;
		delta_q.setRPY(0,0,config_theta);
		wp_odom_ref.setRotation(delta_q);
		
		//wp_odom_ref = map_to_odom_.inverse() * wp_odom_ref;
		tf2::Transform tf_map_to_odom;
		tf2::fromMsg(map_to_odom_.transform, tf_map_to_odom);

		wp_odom_ref = tf_map_to_odom.inverse() * wp_odom_ref;
		
		// Se crean los waypoints de la trayectoria
		robmovil_msgs::msg::TrajectoryPoint point_msg;
		
		//transformTFToMsg(wp_odom_ref, point_msg.transform);
		point_msg.transform = tf2::toMsg(wp_odom_ref);
		
		if(it != path.rend()-1) {
			double config_dx = (it+1)->get(0) - config_x;
			double config_dy = (it+1)->get(1) - config_y;
			double config_dtheta = angles::shortest_angular_distance(config_theta, (it+1)->get(2));
			point_msg.velocity.linear.x = sqrt(pow(config_dx,2) + pow(config_dy,2));
			point_msg.velocity.angular.z = config_dtheta;
		}else{
			point_msg.velocity.linear.x = 0;
			point_msg.velocity.angular.z = 0;
		}
		
		point_msg.time_from_start = t_from_start;
		
		result_trajectory.points.push_back( point_msg );
		
		t_from_start = t_from_start + delta_t;
	}
}
