#include "omni_position_controller.h"

#define KX 0.8
#define KY 0.8
#define KTH 1.0

#define LOOKAHEAD 0.5

#define MAX_VEL 0.5
#define MAX_W 1.0

#define POS_TOL 0.05


OmniPositionController::OmniPositionController()
    : OmniTrajectoryFollower(),
      x_(0.0),
      y_(0.0),
      theta_(0.0),
      current_wp_(0)
{
    odom_sub_ =
        this->create_subscription<nav_msgs::msg::Odometry>(
            "/robot/odometry",
            rclcpp::QoS(10),
            std::bind(
                &OmniPositionController::getCurrentPoseFromOdometry,
                this,
                std::placeholders::_1));

    goal_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/goal_pose",
            rclcpp::QoS(10));
}

void OmniPositionController::handleNewTrajectory(
    const robmovil_msgs::msg::Trajectory& trajectory_msg)
{
    current_wp_ = 0;

    OmniTrajectoryFollower::handleNewTrajectory(
        trajectory_msg);
}

void OmniPositionController::getCurrentPoseFromOdometry(
    const nav_msgs::msg::Odometry& odom)
{
    x_ = odom.pose.pose.position.x;
    y_ = odom.pose.pose.position.y;

    theta_ = tf2::getYaw(odom.pose.pose.orientation);
}

bool OmniPositionController::getPursuitBasedGoal(
    double& gx,
    double& gy,
    double& ga)
{
    const auto& traj = getTrajectory();

    if (traj.points.empty())
        return false;

    double best_dist = 1e9;

    for (size_t i = current_wp_; i < traj.points.size(); i++)
    {
        double px = traj.points[i].transform.translation.x;
        double py = traj.points[i].transform.translation.y;

        double d =
            hypot(px - x_, py - y_);

        if (d < best_dist)
        {
            best_dist = d;
            current_wp_ = i;
        }
    }

    for (size_t i = current_wp_; i < traj.points.size(); i++)
    {
        double px = traj.points[i].transform.translation.x;
        double py = traj.points[i].transform.translation.y;

        double d =
            hypot(px - x_, py - y_);

        if (d >= LOOKAHEAD)
        {
            current_wp_ = i;
            break;
        }
    }

    if (current_wp_ >= (int)traj.points.size())
        return false;

    gx = traj.points[current_wp_].transform.translation.x;
    gy = traj.points[current_wp_].transform.translation.y;

    ga = tf2::getYaw(
        traj.points[current_wp_].transform.rotation);

    return true;
}

bool OmniPositionController::control(
    const rclcpp::Time& t,
    double& vx,
    double& vy,
    double& w)
{
    double gx, gy, ga;

    if (!getPursuitBasedGoal(gx, gy, ga))
        return false;

    double ex = gx - x_;
    double ey = gy - y_;

    double etheta =
        angles::normalize_angle(ga - theta_);

    // error en frame robot

    double ex_r =
        cos(theta_) * ex +
        sin(theta_) * ey;

    double ey_r =
       -sin(theta_) * ex +
        cos(theta_) * ey;

    // control P

    vx = KX * ex_r;
    vy = KY * ey_r;
    w  = KTH * etheta;

    // desaceleración progresiva cerca del objetivo

    double dist = hypot(ex, ey);

    double scale = 1.0;

    if (dist < 0.3)
    {
        scale = dist / 0.3;
    }

    vx *= scale;
    vy *= scale;
    w  *= scale;

    // saturaciones

    vx = std::clamp(vx, -MAX_VEL, MAX_VEL);
    vy = std::clamp(vy, -MAX_VEL, MAX_VEL);
    w  = std::clamp(w , -MAX_W, MAX_W);

    // publish goal

    geometry_msgs::msg::PoseStamped goal;

    goal.header.stamp = this->now();
    goal.header.frame_id = "odom";

    goal.pose.position.x = gx;
    goal.pose.position.y = gy;

    tf2::Quaternion q;
    q.setRPY(0, 0, ga);

    goal.pose.orientation =
        tf2::toMsg(q);

    goal_pub_->publish(goal);

    // finish condition

    double dist = hypot(ex, ey);

    if (current_wp_ ==
            (int)getTrajectory().points.size()-1 &&
        dist < POS_TOL &&
        fabs(etheta) < 0.05)
    {
        return false;
    }

    return true;
}