#include <lazo_abierto/FeedForwardController.h>

FeedForwardController::FeedForwardController() : TrajectoryFollower()
{}

double lineal_interp(const rclcpp::Time& t0, const rclcpp::Time& t1, double v0, double v1, const rclcpp::Time& t)
{
  /** COMPLETAR: esta funcion debe interpolar entre las velocidades requeridas.
   * entre v0 y v1 dependiendo el tiempo trancurrido entre ambos
   * considerar que t siempre se encuentra en t0 y t1. */
  double v_resultante = v0; // arrancamos con la velocidad inicial y luego le sumaremos la interpolación
  v_resultante += ((t - t0).seconds()/(t1 - t0).seconds())*(v1 - v0); // (tj - ti/tf - ti)*(xf - xi)
  return v_resultante;
}

bool FeedForwardController::control(const rclcpp::Time& t, double& v, double& w)
{
  size_t next_point_idx;

  /* Cuando la trayectoria se termina se devuelve false */
  if( not nextPointIndex(t, next_point_idx ) )
    return false;

  RCLCPP_INFO(this->get_logger(), "processing index: %zu", next_point_idx);

  const auto& traj = getTrajectory();
  /* se obtienen los puntos de la trayectoria mas proximos en tiempo (el punto anteriormente transitado y el proximo a alcanzar) */
  robmovil_msgs::msg::TrajectoryPoint prev_point;
  if (next_point_idx == 0)
	  prev_point = traj.points[ next_point_idx ];
  else
	  prev_point = traj.points[ next_point_idx-1 ];
  const robmovil_msgs::msg::TrajectoryPoint& next_point = traj.points[ next_point_idx ];

  /* tiempos requeridos para cada uno de los puntos (se debe alcanzar el siguiente punto en el tiempo t1) */
  const rclcpp::Time& t0 = getInitialTime() + prev_point.time_from_start;
  const rclcpp::Time& t1 = getInitialTime() + next_point.time_from_start;

  if (t < t0 || t >= t1 || t0 == t1)
	  return true; // un punto fue imposible de alcanzar, seguimos con la trayectoria

  /* velocidades lineales en X notificadas por la trayectoria en ambos puntos. */
  double vx0 = prev_point.velocity.linear.x;
  double vx1 = next_point.velocity.linear.x;

  /* velocidades lineales en Y notificadas por la trayectoria en ambos puntos. */
  double vy0 = prev_point.velocity.linear.y;
  double vy1 = next_point.velocity.linear.y;

  /* velocidades angulares notificadas por la trayectoria en ambos puntos*/
  double va0 = prev_point.velocity.angular.z;
  double va1 = next_point.velocity.angular.z;

  RCLCPP_INFO(this->get_logger(), "inter: t0=%ld t1=%ld vx0=%.3f vx1=%.3f va0=%.3f va1=%.3f t=%ld",
            t0.nanoseconds(), t1.nanoseconds(), vx0, vx1, va0, va1, t.nanoseconds());

  RCLCPP_INFO(this->get_logger(), "trajectory size: %zu", traj.points.size());

  /* realizar una interpolacion entre las velocidades requeridas */
  double vx = lineal_interp(t0, t1, vx0, vx1, t); // calculo de la velocidad lineal en X
  double vy = lineal_interp(t0, t1, vy0, vy1, t); // calculo de la velocidad lineal en Y
  double va = lineal_interp(t0, t1, va0, va1, t); // calculo de la velocidad angular

  /** COMPLETAR: Evaluar las velocidades lineales y angulares resultantes para publicar
   * como comandos de velocidad. */
  v = sqrt(vx*vx + vy*vy); // hace falta tener cuidado con el signo: qué pasa si es necesario retroceder?
  w = va;
    
  return true;
}
