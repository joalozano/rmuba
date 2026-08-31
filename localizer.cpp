#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include "localizer.h"

robmovil_ekf::Localizer::Localizer() :
  Node("localizer_node"), u(3), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  u(1) = u(2) = u(3) = 0;
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // Get node parameters
  this->declare_parameter("base_link_frame", std::string("base_link_ekf"));
  this->declare_parameter("ekf_frame", std::string("odom"));
  this->declare_parameter("laser_frame", std::string("laser"));
  this->declare_parameter("only_prediction", false);
  this->declare_parameter("min_landmark_size", 2);

  this->get_parameter("base_link_frame", base_frame_);
  this->get_parameter("ekf_frame", map_frame_);
  this->get_parameter("laser_frame", laser_frame_);
  this->get_parameter("only_prediction", only_prediction);
  this->get_parameter("min_landmark_size", min_landmark_size);
  
  landmark_sub = this->create_subscription<robmovil_msgs::msg::LandmarkArray>(
      "/landmarks", rclcpp::QoS(10),
      std::bind(&Localizer::on_landmark_array, this, std::placeholders::_1));
  odo_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "/robot/odometry", rclcpp::QoS(10),
      std::bind(&Localizer::on_odometry, this, std::placeholders::_1));

  /* NOTA: antes se pedia QoS reliable + transient_local para /posts. Si el publisher del
   * simulador es volatile (lo mas comun, salvo que la escena lo configure explicitamente
   * como transient_local), esa combinacion es INCOMPATIBLE en ROS2 -- un publisher volatile
   * no matchea con un subscriber que pide transient_local -- y la suscripcion nunca conecta,
   * sin tirar excepcion (solo un warning de QoS en el log). Si eso pasaba: map_received_
   * quedaba en false para siempre, on_landmark_array() cortaba apenas entraba, measureUpdateStep()
   * nunca se ejecutaba, y la covarianza solo podia crecer -- exactamente el sintoma reportado.
   * Usamos QoS por default (volatile), que es compatible tanto si el publisher es volatile
   * como si es transient_local. */
  post_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/posts", rclcpp::QoS(10), std::bind(&Localizer::on_posts, this, std::placeholders::_1));

  pose_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/pose", rclcpp::QoS(10));
  
  while ((this->now().nanoseconds() == 0));
  t = this->now();

  /* NOTA: se probo primero con rclcpp::create_timer(this, this->get_clock(), ...) para
   * atar el timer al reloj del nodo (respetando use_sim_time). En la practica esto
   * tiraba error / se comportaba mal, muy probablemente por el salto de tiempo simulado
   * de 0 al primer valor real (o reinicios de la simulacion), que rclcpp interpreta como
   * un "jump" del clock y puede desestabilizar un timer atado a ese clock source.
   *
   * Se usa entonces create_wall_timer solo para el DISPARO periodico. Esto no rompe la
   * consistencia temporal: prediction_event()/advance_time()/publish_estimate() siguen
   * usando this->now(), que sigue respetando use_sim_time y calculando delta_t en tiempo
   * simulado, igual que los timestamps de /robot/odometry y /landmarks. */
  prediction_timer = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&Localizer::prediction_event, this));
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

  // LocalizerEKF::Vector x = ekf.getX();
  // LocalizerEKF::Matrix P = ekf.calculateP();
  
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
    
    if (ekf.set_measure(z))
      ekf.measureUpdateStep(z);
      
      //RCLCPP_INFO_STREAM(this->get_logger(), "Measure updated estimate: " << ekf.getX());
      //RCLCPP_INFO_STREAM(this->get_logger(), "Measure updated covariance: " << ekf.calculateP());
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
	// /posts viene en frame "map". El EKF trabaja en "odom" (mismo origen
  // que la odometría de rueda). Hace falta transformar los postes.
  std::string map_frame = posts->header.frame_id.empty() ? "map" : posts->header.frame_id;

  geometry_msgs::msg::TransformStamped map_to_odom;
  try
  {
    map_to_odom = tf_buffer_.lookupTransform(
        "odom", map_frame, posts->header.stamp, tf2::durationFromSec(1.0));
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_WARN(this->get_logger(),
                "No pude obtener map->odom en el stamp de /posts (%s), reintento con la ultima disponible",
                ex.what());
    try
    {
      map_to_odom = tf_buffer_.lookupTransform("odom", map_frame, tf2::TimePointZero);
    }
    catch (tf2::TransformException& ex2)
    {
      RCLCPP_ERROR(this->get_logger(), "No se pudo transformar el mapa de postes: %s", ex2.what());
      return;
    }
  }

  auto posts_in_odom = std::make_shared<geometry_msgs::msg::PoseArray>();
  posts_in_odom->header.frame_id = "odom";
  posts_in_odom->header.stamp = posts->header.stamp;
  posts_in_odom->poses.reserve(posts->poses.size());

  for (const auto& pose : posts->poses)
  {
    geometry_msgs::msg::Pose transformed;
    tf2::doTransform(pose, transformed, map_to_odom);
    posts_in_odom->poses.push_back(transformed);
  }

  ekf.set_map(posts_in_odom);
  map_received_ = true;
  RCLCPP_INFO(this->get_logger(), "Mapa de %zu postes cargado (transformado a odom)", posts_in_odom->poses.size());

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

  /* Pose estimada por el EKF, en el frame en el que integra la odometria (parametro
   * ekf_frame, "odom" por defecto -- mismo origen que la odometria de rueda) */
  tf2::Transform ekf_frame_to_base_link;
  ekf_frame_to_base_link.setOrigin(tf2::Vector3(x(1), x(2), 0));
  ekf_frame_to_base_link.setRotation(tf2::Quaternion(tf2::Vector3(0,0,1), x(3)));

  /* El enunciado pide publicar la transformacion map -> base_link_ekf directamente
   * (punto 4.c), no odom -> base_link_ekf. La componemos con la map -> ekf_frame que ya
   * publica el simulador (map -> odom), en vez de dejar que un consumidor externo tenga
   * que encadenar ambas tf por su cuenta.
   *
   * NOTA: la covarianza publicada abajo queda expresada tal cual la calcula el EKF (en el
   * frame ekf_frame), sin rotarla por la orientacion de map->ekf_frame. Si esa transformacion
   * tuviera una rotacion distinta de cero, el elipsoide de covarianza en RViz quedaria
   * desalineado respecto del frame map -- para este TP alcanza, pero si hace falta mas
   * precision habria que rotar tambien el bloque de covarianza XY. */
  tf2::Transform map_to_base_link = ekf_frame_to_base_link;

  try
  {
    geometry_msgs::msg::TransformStamped map_to_ekf_frame_msg =
        tf_buffer_.lookupTransform("map", map_frame_, tf2::TimePointZero);
    tf2::Transform map_to_ekf_frame;
    tf2::fromMsg(map_to_ekf_frame_msg.transform, map_to_ekf_frame);
    map_to_base_link = map_to_ekf_frame * ekf_frame_to_base_link;
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_WARN(this->get_logger(),
                "No pude obtener map -> %s para publicar map -> base_link_ekf (%s); "
                "publico la pose sin componer (asumiendo map == %s por ahora)",
                map_frame_.c_str(), ex.what(), map_frame_.c_str());
  }

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

  /* set pose (ya compuesta a map -> base_link_ekf) */
  tf2::toMsg(map_to_base_link, pose_msg.pose.pose);

  /* publish */
  pose_pub->publish(pose_msg);

  // publish TF transform: map -> base_link_ekf (directo, segun pide el enunciado)
  geometry_msgs::msg::TransformStamped map_to_base_link_msg;
  map_to_base_link_msg.header.frame_id = "map";
  map_to_base_link_msg.header.stamp = now;
  map_to_base_link_msg.child_frame_id = base_frame_;
  map_to_base_link_msg.transform = tf2::toMsg(map_to_base_link);

  tf_broadcaster_->sendTransform(map_to_base_link_msg);
}
