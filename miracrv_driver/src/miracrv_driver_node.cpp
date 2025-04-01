#include "libracer/ardurover.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto options = rclcpp::NodeOptions();
    auto driver = std::make_shared<libracer::ArduRoverDriver>(options);
    
    rclcpp::spin(driver);
    rclcpp::shutdown();
    return 0;
}