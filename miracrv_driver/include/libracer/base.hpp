#ifndef BASE_HPP
#define BASE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace libracer {

class BaseDriver : public rclcpp::Node {
public:
    BaseDriver(const std::string& node_name) : Node(node_name) {}
    virtual void send_velocity_command(const geometry_msgs::msg::Twist& cmd_vel) = 0;
    virtual void update_odometry(const nav_msgs::msg::Odometry& odom) = 0;

protected:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    
    uint8_t system_id = 1;
    uint8_t component_id = 1;
};

} // namespace libracer

#endif // BASE_HPP