#ifndef ARDUROVER_HPP
#define ARDUROVER_HPP

#include "libracer/base.hpp"
#include <mavros_msgs/msg/ekf3_status.hpp>
#include <mavros_msgs/msg/global_position.hpp>

namespace libracer
{

    class ArduRoverDriver : public BaseDriver
    {
    public:
        explicit ArduRoverDriver(const rclcpp::NodeOptions &options);

        void send_velocity_command(const geometry_msgs::msg::Twist &cmd_vel) override;
        void update_odometry(const nav_msgs::msg::Odometry &odom) override;

    private:
        void handle_ekf3_status(const mavros_msgs::msg::EKf3Status::SharedPtr msg);
        void handle_global_position(const mavros_msgs::msg::GlobalPosition::SharedPtr msg);

        rclcpp::Subscription<mavros_msgs::msg::EKf3Status>::SharedPtr ekf3_sub_;
        rclcpp::Subscription<mavros_msgs::msg::GlobalPosition>::SharedPtr global_pos_sub_;

        bool ekf3_ready = false;
        std::mutex odom_mutex_;
    };

} // namespace libracer

#endif // ARDUROVER_HPP