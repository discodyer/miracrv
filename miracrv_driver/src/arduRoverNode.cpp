#include <rclcpp/rclcpp.hpp>
#include "libracer/arduRoverDriver.hpp"

class ArduRoverNode : public rclcpp::Node
{
public:
    ArduRoverNode() : Node("ardu_rover_node")
    {
        // 声明所有参数（包括嵌套参数）
        declareParameters();

        RCLCPP_INFO(this->get_logger(), "ArduRover node constructed");
    }
        
    // 添加初始化方法
    void init()
    {
        // 创建定时器专用的回调组
        timerCallbackGroup_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // 创建驱动实例
        driver_ = std::make_shared<libracer::ArduRoverDriver>(shared_from_this());
        
        // 初始化驱动
        if (!driver_->initialize()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize ArduRover driver");
            rclcpp::shutdown();
            return;
        }

        // 设置GPS原点（如果启用）
        if (this->get_parameter("gps_origin.enabled").as_bool()) {
            double lat = this->get_parameter("gps_origin.latitude").as_double();
            double lon = this->get_parameter("gps_origin.longitude").as_double();
            // double alt = this->get_parameter("gps_origin.altitude").as_double();
            if (std::abs(lat) > 0.0 && std::abs(lon) > 0.0) {
            driver_->setGlobalOrigin(lat, lon);
            }
        }
        
        // 创建定时器进行状态检查
        stateTimer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&ArduRoverNode::stateTimerCallback, this),
            timerCallbackGroup_);

        // TF坐标变换设置
        if (this->get_parameter("tf.publish_tf").as_bool()) {
            driver_->setBaseFrameId(this->get_parameter("tf.base_frame_id").as_string());
            driver_->setOdomFrameId(this->get_parameter("tf.odom_frame_id").as_string());
        }

        // 保存速度限制供后续使用
        maxLinearX_ = this->get_parameter("velocity_limits.max_linear_x").as_double();
        maxLinearY_ = this->get_parameter("velocity_limits.max_linear_y").as_double();
        maxAngularZ_ = this->get_parameter("velocity_limits.max_angular_z").as_double();
        
        // 如果启用watchdog，创建watchdog定时器
        // if (this->get_parameter("safety.enable_watchdog").as_bool()) {
        //     double timeout = this->get_parameter("safety.watchdog_timeout").as_double();
        //     watchdogTimer_ = this->create_wall_timer(
        //         std::chrono::milliseconds(static_cast<int>(timeout * 1000)),
        //         std::bind(&ArduRoverNode::watchdogCallback, this));
        // }
        
        RCLCPP_INFO(this->get_logger(), "ArduRover node started with configuration from: %s",
                    this->get_parameter("config_file").as_string().c_str());
    }

private:
    void declareParameters()
    {
        // 基本参数
        this->declare_parameter("auto_arm", false);
        this->declare_parameter("default_mode", "GUIDED");
        this->declare_parameter("config_file", "none");
        
        // GPS原点参数
        this->declare_parameter("gps_origin.enabled", true);
        this->declare_parameter("gps_origin.latitude", 0.0);
        this->declare_parameter("gps_origin.longitude", 0.0);
        // this->declare_parameter("gps_origin.altitude", 0.0);
        
        // 速度限制参数
        this->declare_parameter("velocity_limits.max_linear_x", 5.0);
        this->declare_parameter("velocity_limits.max_linear_y", 2.0);
        this->declare_parameter("velocity_limits.max_angular_z", 1.0);
        
        // 超时参数
        this->declare_parameter("timeouts.connection", 5.0);
        this->declare_parameter("timeouts.service_call", 1.0);
        
        // 话题参数
        this->declare_parameter("topics.cmd_vel", "/miracrv/cmd_vel");
        this->declare_parameter("topics.odom", "/miracrv/odom");
        
        // TF参数
        this->declare_parameter("tf.publish_tf", true);
        this->declare_parameter("tf.base_frame_id", "base_link");
        this->declare_parameter("tf.odom_frame_id", "odom");
        
        // 安全参数
        // this->declare_parameter("safety.enable_watchdog", true);
        // this->declare_parameter("safety.watchdog_timeout", 2.0);
        // this->declare_parameter("safety.enable_geofence", false);
        // this->declare_parameter("safety.geofence_radius", 50.0);
    }

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
        if (currentState.mode != defaultMode) {
            if (driver_->setMode(defaultMode)) {
                modeSet_ = true;
            }
        }else{
            modeSet_ = true;
        }
        
        // 自动解锁（如果启用）
        bool autoArm = this->get_parameter("auto_arm").as_bool();
        if (autoArm && !driver_->isArmed() && modeSet_) {
            driver_->arm();
        }
    }

    // void watchdogCallback()
    // {
    //     // 实现watchdog逻辑
    //     auto now = this->now();
    //     if ((now - lastCmdTime_).seconds() > 
    //         this->get_parameter("safety.watchdog_timeout").as_double()) {
    //         // 发送停止命令
    //         RCLCPP_WARN(this->get_logger(), "Watchdog timeout - stopping vehicle");
    //         // driver_->setBodyVelocity(0, 0, 0);
    //     }
    // }
    
private:
    // 定时器专用回调组
    rclcpp::CallbackGroup::SharedPtr timerCallbackGroup_;

    std::shared_ptr<libracer::ArduRoverDriver> driver_;
    rclcpp::TimerBase::SharedPtr stateTimer_;
    // rclcpp::TimerBase::SharedPtr watchdogTimer_;
    bool modeSet_ = false;

    // 速度限制
    double maxLinearX_;
    double maxLinearY_;
    double maxAngularZ_;
    
    // Watchdog
    // rclcpp::Time lastCmdTime_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<ArduRoverNode>();
        node->init();

        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception in main: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}