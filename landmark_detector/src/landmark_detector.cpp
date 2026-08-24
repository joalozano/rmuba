#include <vector>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <robmovil_msgs/msg/landmark_array.hpp>
#include <robmovil_msgs/msg/landmark.h>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "landmark_detector.h"

#define LANDMARK_DIAMETER 0.1 // metros (0.1 = 10cm)

robmovil_ekf::LandmarkDetector::LandmarkDetector() :
    Node("landmark_detector_node"), tf_buffer(this->get_clock()), tf_listener(tf_buffer), transform_received(false)
{
  laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>( "/robot/front_laser/scan", rclcpp::QoS(10), std::bind(&LandmarkDetector::on_laser_scan, this, std::placeholders::_1));
  landmark_pub = this->create_publisher<robmovil_msgs::msg::LandmarkArray>("/landmarks", rclcpp::QoS(10));
  pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud>("/landmarks_pointcloud", rclcpp::QoS(10));

  
  this->declare_parameter("robot_frame", std::string("base_link"));
  this->declare_parameter("publish_robot_frame", std::string("base_link"));
  this->declare_parameter("laser_frame", std::string("front_laser"));

  this->get_parameter("robot_frame", robot_frame);
  this->get_parameter("publish_robot_frame", publish_robot_frame);
  this->get_parameter("laser_frame", laser_frame);

  //RCLCPP_INFO(this->get_logger(), "Publishing to frame  %s", publish_robot_frame.c_str());
}

float dist2(tf2::Vector3 *p1, tf2::Vector3 *p2) {
	float delta_x = p1->getX() - p2->getX(),
		  delta_y = p1->getY() - p2->getY();
	return sqrt(delta_x * delta_x + delta_y * delta_y);
}

void robmovil_ekf::LandmarkDetector::on_laser_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if(!update_laser_tf(msg->header.stamp)){
    RCLCPP_WARN(this->get_logger(), "%s -> %s transform not yet received, not publishing landmarks", laser_frame.c_str(), robot_frame.c_str());
    return;
  }

  /* COMPLETAR: Convertir range,bearing a puntos cartesianos x,y,0. (ojo: descarta puntos en el "infinito")
   * Descartando aquellas mediciones por fuera de los rangos validos */
  std::vector<tf2::Vector3> cartesian;
  
  for (int i = 0; i < msg->ranges.size(); i++)
  {
    /* Utilizar la informacion del mensaje para filtrar y convertir */
    float range = msg->ranges[i];
    float range_min = msg->range_min;
    float range_max = msg->range_max;
    
	if (range >= range_max || range <= range_min)
		continue;

    float angle_min = msg->angle_min;
    float angle_increment = msg->angle_increment;
	float angle = angle_min + angle_increment * i;
	
    /* COMPLETAR: p debe definirse con informacion valida y 
     * en coordenadas cartesianas */
    tf2::Vector3 p(range*cos(angle),
				  range*sin(angle),
				  0);
    
    /* convierto el punto en relacion al marco de referencia del laser al marco del robot */
    p = laser_transform * p;
    cartesian.push_back(p);
  }

  /* Mensaje del arreglo de landmarks detectados */
  robmovil_msgs::msg::LandmarkArray landmark_array;
  landmark_array.header.stamp = msg->header.stamp;
  landmark_array.header.frame_id = publish_robot_frame;
  
  /* VECTORES AUXILIARES: Pueden utilizar landmark_points para ir acumulando
   * mediciones cercanas */
  std::vector<tf2::Vector3> landmark_points;
  
  // centroides estimados de los postes en coordenadas cartesianas
  std::vector<tf2::Vector3> centroids;
  
  if (not cartesian.empty()) {
  	landmark_points.push_back(cartesian[0]);
  }

  for (int i = 1; i < cartesian.size(); i++)
  {
    /* COMPLETAR: Acumular, de manera secuencial, mediciones cercanas (distancia euclidea) */
	#define D_TOL 0.1
	if (((cartesian[i] - landmark_points.back()).length() <= D_TOL)) {
		landmark_points.push_back(cartesian[i]);
      		continue;
	}
	
    /* Al terminarse las mediciones provenientes al landmark que se venia detectando,
     * se calcula la pose del landmark como el centroide de las mediciones */

    //RCLCPP_INFO(this->get_logger(), "landmark con %zu puntos", landmark_points.size());
    
    /* COMPLETAR: calcular el centroide de los puntos acumulados */

    tf2::Vector3 centroid(0, 0, 0);// = cartesian[min_cart] + cartesian[min_cart].normalize() * (LANDMARK_DIAMETER/2);
	
	float min_dist = FLT_MAX, dist;
	for (int j = 0; j < landmark_points.size(); j++) {
		dist = landmark_points[j].length();
		if (min_dist > dist) {
			min_dist = dist;
			centroid = landmark_points[j];
		}
	}
	tf2::Vector3 centroidCopy = centroid;
	centroidCopy.normalize();
	centroid += centroidCopy * (LANDMARK_DIAMETER/2);
	//centroid = laser_transform * centroid;

    //RCLCPP_INFO(this->get_logger(), "landmark detectado (cartesianas): %f %f %f", centroid.getX(), centroid.getY(), centroid.getZ());
    centroids.push_back(centroid);

    /* Convertir el centroide a coordenadas polares, construyendo el mensaje requerido */
    robmovil_msgs::msg::Landmark landmark;
    float x = centroid.getX(), y = centroid.getY();

    float r = sqrt(x*x + y*y); // distancia desde el robot al centroide
    landmark.range = r;
    
    float a = atan2(y,x); // angulo de la recta que conecta al robot con el centroide
    landmark.bearing = a;

    /* Fin Completar */

    /* se agrega el landmark en coordenadas polares */
    landmark_array.landmarks.push_back(landmark);

    /* empiezo a procesar un nuevo landmark */
    	landmark_points.clear();
	landmark_points.push_back(cartesian[i]);
  }
  
  if (!landmark_points.empty()) { // procesamos el último landmark
    tf2::Vector3 centroid(0, 0, 0);
	
	float min_dist = FLT_MAX, dist;
	for (int i = 0; i < landmark_points.size(); i++) { // buscamos el más cercano de entre los puntos del landmark
		dist = landmark_points[i].length();
		if (min_dist > dist) {
			min_dist = dist;
			centroid = landmark_points[i];
		}
	}
	tf2::Vector3 centroidCopy = centroid;
	centroidCopy.normalize();
	centroid += centroidCopy * (LANDMARK_DIAMETER/2);
	//centroid = laser_transform * centroid;

    centroids.push_back(centroid);

    /* Convertir el centroide a coordenadas polares, construyendo el mensaje requerido */
    robmovil_msgs::msg::Landmark landmark;
    float x = centroid.getX(), y = centroid.getY();

    float r = sqrt(x*x + y*y); // distancia desde el robot al centroide
    landmark.range = r;
    
    float a = atan2(y,x); // angulo de la recta que conecta al robot con el centroide
    landmark.bearing = a;

    /* se agrega el landmark en coordenadas polares */
    landmark_array.landmarks.push_back(landmark);
    landmark_points.clear();
  }
  /* Publicamos el mensaje de los landmarks encontrados */
  if (!landmark_array.landmarks.empty()){
    landmark_pub->publish(landmark_array);
    publish_pointcloud(landmark_array.header, centroids);
  }
}

bool robmovil_ekf::LandmarkDetector::update_laser_tf(const rclcpp::Time& required_time)
{
  try
  {
    geometry_msgs::msg::TransformStamped transformStamped = tf_buffer.lookupTransform(robot_frame, laser_frame, required_time, tf2::durationFromSec(0.1));
    tf2::fromMsg(transformStamped.transform, laser_transform);
    return true;
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN(this->get_logger(), "No se pudo transformar %s a %s: %s",
              robot_frame.c_str(), laser_frame.c_str(), ex.what());
    return false;
  }
}

void robmovil_ekf::LandmarkDetector::publish_pointcloud(const std_msgs::msg::Header& header, const std::vector<tf2::Vector3>& landmark_positions)
{
  sensor_msgs::msg::PointCloud pointcloud;
  pointcloud.header.stamp = header.stamp;
  pointcloud.header.frame_id = header.frame_id;

  for (int i = 0; i < landmark_positions.size(); i++)
  {
    geometry_msgs::msg::Point32 point;
    point.x = landmark_positions[i].getX();
    point.y = landmark_positions[i].getY();
    point.z = landmark_positions[i].getZ();
    pointcloud.points.push_back(point);
  }
  pointcloud_pub->publish(pointcloud);
}
