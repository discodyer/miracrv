#include <rclcpp/rclcpp.hpp>
#include "libracer/arduRoverDriver.hpp"

class ArduRoverNode : public rclcpp::Node
{
public:
    ArduRoverNode() : Node("ardu_rover_node")
    {
        // 声明参数
        this->declare_parameter("auto_arm", false);
        this->declare_parameter("default_mode", "GUIDED");
        this->declare_parameter("gps_origin_lat", 0.0);
        this->declare_parameter("gps_origin_lon", 0.0);
        
        // 创建驱动实例
        driver_ = std::make_shared<libracer::ArduRoverDriver>(shared_from_this());
        
        // 初始化驱动
        if (!driver_->initialize()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize ArduRover driver");
            rclcpp::shutdown();
            return;
        }
        
        // 创建定时器进行状态检查
        stateTimer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&ArduRoverNode::stateTimerCallback, this));
        
        // 设置GPS原点（如果提供）
        double lat = this->get_parameter("gps_origin_lat").as_double();
        double lon = this->get_parameter("gps_origin_lon").as_double();
        if (std::abs(lat) > 0.0 && std::abs(lon) > 0.0) {
            driver_->setGlobalOrigin(lat, lon);
        }
        
        RCLCPP_INFO(this->get_logger(), "ArduRover node started successfully");
    }

private:
    void stateTimerCallback()
    {
        if (!driver_->isConnected()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "Not connected to MAVROS");
            return;
        }
        
        // 自动切换到默认模式
        auto currentState = driver_->getCurrentState();
        std::string defaultMode = this->get_parameter("default_mode").as_string();
        if (currentState.mode != defaultMode && !modeSet_) {
            if (driver_->setMode(defaultMode)) {
                modeSet_ = true;
            }
        }
        
        // 自动解锁（如果启用）
        bool autoArm = this->get_parameter("auto_arm").as_bool();
        if (autoArm && !driver_->isArmed() && modeSet_) {
            driver_->arm();
        }
    }
    
private:
    std::shared_ptr<libracer::ArduRoverDriver> driver_;
    rclcpp::TimerBase::SharedPtr stateTimer_;
    bool modeSet_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<ArduRoverNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception in main: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}