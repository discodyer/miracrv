#include "libracer/ardurover.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std::chrono_literals;

namespace libracer {

ArduRoverDriver::ArduRoverDriver(const rclcpp::NodeOptions& options)
    : BaseDriver("miracrv_driver") 
{
    // Parameters
    this->declare_parameter("target_system", 1);
    this->declare_parameter("target_component", 1);

    // MAVROS subscribers
    ekf3_sub_ = create_subscription<mavros_msgs::msg::EKf3Status>(
        "/ekf3/status", 10,
        [this](const mavros_msgs::msg::EKf3Status::SharedPtr msg) {
            handle_ekf3_status(msg);
        });

    global_pos_sub_ = create_subscription<mavros_msgs::msg::GlobalPosition>(
        "/global_position/raw", 10,
        [this](const mavros_msgs::msg::GlobalPosition::SharedPtr msg) {
            handle_global_position(msg);
        });

    // Publishers and Subscribers
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/miracrv/odom", 10);
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/miracrv/cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            send_velocity_command(*msg);
        });
}

void ArduRoverDriver::send_velocity_command(const geometry_msgs::msg::Twist& cmd_vel) {
    // Implement MAVLink message conversion and sending logic
    auto set_position_target = std::make_shared<mavros_msgs::msg::PositionTarget>();
    
    set_position_target->header.stamp = now();
    set_position_target->coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_BODY_NED;
    set_position_target->type_mask = 0x0CFC; // Enable velocity controls
    set_position_target->velocity.x = cmd_vel.linear.x;
    set_position_target->velocity.y = cmd_vel.linear.y;
    set_position_target->velocity.z = cmd_vel.angular.z;

    // Use MAVROS service or publisher to send the command
}

void ArduRoverDriver::update_odometry(const nav_msgs::msg::Odometry& odom) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_pub_->publish(odom);
}

void ArduRoverDriver::handle_ekf3_status(const mavros_msgs::msg::EKf3Status::SharedPtr msg) {
    ekf3_ready = msg->flags & 0x01; // Check EKF3 health flag
}

void ArduRoverDriver::handle_global_position(const mavros_msgs::msg::GlobalPosition::SharedPtr msg) {
    if(!ekf3_ready) return;

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    
    // Convert latitude/longitude to local coordinates (需要实现坐标系转换)
    odom.pose.pose.position.x = msg->latitude;
    odom.pose.pose.position.y = msg->longitude;
    odom.pose.pose.position.z = msg->altitude;

    // Convert heading to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, msg->heading * M_PI/180.0);
    odom.pose.pose.orientation = tf2::toMsg(q);

    update_odometry(odom);
}

} // namespace libracer