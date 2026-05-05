#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "robmovil_msgs/msg/multi_encoder_ticks.hpp"

#define M_PI 3.14159265358979323846

double r;     // radio rueda
double lx;    // mitad largo
double ly;    // mitad ancho

class OmniOdometry : public rclcpp::Node
{
public:
    OmniOdometry();

private:

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<robmovil_msgs::msg::MultiEncoderTicks>::SharedPtr enc_sub_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fl_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fr_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rl_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rr_pub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Estado del robot
    double x_, y_, theta_;

    // Parámetros
    double r_, lx_, ly_;

    // Encoders
    bool initialized_;
    std::vector<int> last_ticks_;
    rclcpp::Time last_time_;

    // Callbacks
    void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void encCallback(const robmovil_msgs::msg::MultiEncoderTicks::SharedPtr msg);
};

// CONSTRUCTOR

OmniOdometry::OmniOdometry() : Node("omni_odometry"),
    x_(0), y_(0), theta_(0), initialized_(false)
{
    // Subs
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/robot/cmd_vel", 10,
        std::bind(&OmniOdometry::cmdCallback, this, std::placeholders::_1));

    enc_sub_ = this->create_subscription<robmovil_msgs::msg::MultiEncoderTicks>(
        "/robot/encoders", 10,
        std::bind(&OmniOdometry::encCallback, this, std::placeholders::_1));

    // Pubs ruedas
    fl_pub_ = this->create_publisher<std_msgs::msg::Float64>("/robot/front_left_wheel/cmd_vel", 10);
    fr_pub_ = this->create_publisher<std_msgs::msg::Float64>("/robot/front_right_wheel/cmd_vel", 10);
    rl_pub_ = this->create_publisher<std_msgs::msg::Float64>("/robot/rear_left_wheel/cmd_vel", 10);
    rr_pub_ = this->create_publisher<std_msgs::msg::Float64>("/robot/rear_right_wheel/cmd_vel", 10);

    // Odom
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/robot/odometry", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Parámetros (ejemplo, después ajustás)
    r_ = 0.05;
    lx_ = 0.175;
    ly_ = 0.175;
}


// CINEMÁTICA INVERSA

void OmniOdometry::cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    double vx = msg->linear.x;
    double vy = msg->linear.y;
    double w  = msg->angular.z;

    double L = lx_ + ly_;

    double w1 = (1/r_) * (vx - vy - L*w);
    double w2 = (1/r_) * (vx + vy + L*w);
    double w3 = (1/r_) * (vx + vy - L*w);
    double w4 = (1/r_) * (vx - vy + L*w);

    std_msgs::msg::Float64 m;

    m.data = w1; fl_pub_->publish(m);
    m.data = w2; fr_pub_->publish(m);
    m.data = w3; rl_pub_->publish(m);
    m.data = w4; rr_pub_->publish(m);

    // Debug
    RCLCPP_INFO(this->get_logger(),
        "CMD vx=%.2f vy=%.2f w=%.2f -> w1=%.2f w2=%.2f w3=%.2f w4=%.2f",
        vx, vy, w, w1, w2, w3, w4);
}


// CINEMÁTICA DIRECTA + ODOMETRÍA

void OmniOdometry::encCallback(const robmovil_msgs::msg::MultiEncoderTicks::SharedPtr msg)
{
    if (!initialized_) {
        initialized_ = true;
        last_ticks_ = msg->ticks;
        last_time_ = msg->header.stamp;
        return;
    }

    // Δticks
    std::vector<int> delta(4);
    for (int i = 0; i < 4; i++) {
        delta[i] = msg->ticks[i] - last_ticks_[i];
    }

    // Δt
    rclcpp::Time current_time(msg->header.stamp);
    double dt = (current_time - last_time_).seconds();

    if (dt <= 0.0) return;

    // ticks → velocidad angular
    double N = 500.0;
    std::vector<double> w(4);

    for (int i = 0; i < 4; i++) {
        w[i] = (2 * M_PI * delta[i]) / (N * dt);
    }

    double L = lx_ + ly_;

    // cinemática directa
    double vx = (r_/4.0) * (w[0] + w[1] + w[2] + w[3]);
    double vy = (r_/4.0) * (-w[0] + w[1] + w[2] - w[3]);
    double omega = (r_/(4.0*L)) * (-w[0] + w[1] - w[2] + w[3]);

    // integración
    x_ += (vx * cos(theta_) - vy * sin(theta_)) * dt;
    y_ += (vx * sin(theta_) + vy * cos(theta_)) * dt;
    theta_ += omega * dt;

    theta_ = atan2(sin(theta_), cos(theta_));

    // mensaje odometría
    nav_msgs::msg::Odometry odom;

    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom.pose.pose.orientation = tf2::toMsg(q);

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = omega;

    odom_pub_->publish(odom);

    // TF
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = current_time;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";

    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0;
    t.transform.rotation = odom.pose.pose.orientation;

    tf_broadcaster_->sendTransform(t);

    // Debug
    RCLCPP_INFO(this->get_logger(),
        "ODOM vx=%.2f vy=%.2f w=%.2f | x=%.2f y=%.2f th=%.2f",
        vx, vy, omega, x_, y_, theta_);

    // actualizar estado
    last_ticks_ = msg->ticks;
    last_time_ = current_time;
}

// MAIN

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OmniOdometry>());
    rclcpp::shutdown();
    return 0;
}