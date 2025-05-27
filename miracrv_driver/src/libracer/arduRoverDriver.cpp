#include "libracer/arduRoverDriver.hpp"
#include <mavros_msgs/msg/command_code.hpp>
#include <chrono>

namespace libracer
{

ArduRoverDriver::ArduRoverDriver(rclcpp::Node::SharedPtr node)
    : BaseDriver(node, "/miracrv/cmd_vel", "/mavros/local_position/odom")
    , autoArm_(false)
    , defaultMode_("GUIDED")
{
    RCLCPP_INFO(getLogger(), "ArduRoverDriver constructor called");
}

bool ArduRoverDriver::initialize()
{
    // 调用基类初始化
    if (!BaseDriver::initialize()) {
        return false;
    }
    
    try {
        // 创建MAVROS相关发布者
        positionTargetPub_ = node_->create_publisher<mavros_msgs::msg::PositionTarget>(
            "mavros/setpoint_raw/local", 10);
        
        globalOriginPub_ = node_->create_publisher<geographic_msgs::msg::GeoPointStamped>(
            "mavros/global_position/set_gp_origin", 10);
        
        // 创建MAVROS状态订阅者
        stateSub_ = node_->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 10,
            std::bind(&ArduRoverDriver::stateCallback, this, std::placeholders::_1));
        
        // 创建服务客户端
        armingClient_ = node_->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
        setModeClient_ = node_->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        commandClient_ = node_->create_client<mavros_msgs::srv::CommandLong>("mavros/cmd/command");
        
        // 等待MAVROS连接
        if (!waitForConnection(5.0)) {
            RCLCPP_WARN(getLogger(), "Failed to connect to MAVROS within timeout");
        }
        
        RCLCPP_INFO(getLogger(), "ArduRoverDriver initialized successfully");
        return true;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(getLogger(), "Failed to initialize ArduRoverDriver: %s", e.what());
        return false;
    }
}

void ArduRoverDriver::cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    // 将TwistStamped转换为车体坐标系速度控制
    setBodyVelocity(
        msg->twist.linear.x,
        msg->twist.linear.y,
        msg->twist.angular.z
    );
}

void ArduRoverDriver::setBodyVelocity(double vx, double vy, double yawRate)
{
    auto rawTarget = mavros_msgs::msg::PositionTarget();
    
    // 使用车体坐标系
    rawTarget.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_BODY_OFFSET_NED;
    
    // 设置忽略的字段（只使用速度控制）
    rawTarget.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_PX |
                         mavros_msgs::msg::PositionTarget::IGNORE_PY |
                         mavros_msgs::msg::PositionTarget::IGNORE_PZ |
                         mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                         mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                         mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                         mavros_msgs::msg::PositionTarget::IGNORE_VZ;  // 忽略Z轴速度（地面车辆）
    
    // 处理偏航率
    if (std::fabs(yawRate) < 1e-3) {
        rawTarget.type_mask |= mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
        rawTarget.yaw = 0;
    } else {
        rawTarget.type_mask |= mavros_msgs::msg::PositionTarget::IGNORE_YAW;
    }
    
    // 设置速度值
    rawTarget.velocity.x = vx;
    rawTarget.velocity.y = vy;
    rawTarget.velocity.z = 0.0;  // 地面车辆不需要Z轴速度
    rawTarget.yaw_rate = yawRate;
    
    // 添加时间戳
    rawTarget.header.stamp = now();
    
    // 发布消息
    positionTargetPub_->publish(rawTarget);
}

void ArduRoverDriver::setGlobalOrigin(double latitude, double longitude, double altitude)
{
    // 构建 GeoPointStamped 消息
    auto gpOrigin = geographic_msgs::msg::GeoPointStamped();
    gpOrigin.position.latitude = latitude;
    gpOrigin.position.longitude = longitude;
    gpOrigin.position.altitude = altitude;
    gpOrigin.header.stamp = now();
    
    globalOriginPub_->publish(gpOrigin);
    RCLCPP_INFO(getLogger(), "Setting GPS origin: lat=%.6f, lon=%.6f, alt=%.2f", 
                latitude, longitude, altitude);
}

bool ArduRoverDriver::arm()
{
    // 检查服务是否可用
    if (!armingClient_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(getLogger(), "Arming service not available");
        return false;
    }
    
    // 创建请求
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = true;
    
    // 发送请求
    auto future = armingClient_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        auto result = future.get();
        if (result->success) {
            RCLCPP_INFO(getLogger(), "Vehicle armed successfully");
            return true;
        } else {
            RCLCPP_ERROR(getLogger(), "Failed to arm vehicle: %s", result->result ? "true" : "false");
            return false;
        }
    }
    
    RCLCPP_ERROR(getLogger(), "Arming request timed out");
    return false;
}

bool ArduRoverDriver::disarm()
{
    // 检查服务是否可用
    if (!armingClient_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(getLogger(), "Arming service not available");
        return false;
    }
    
    // 创建请求
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = false;
    
    // 发送请求
    auto future = armingClient_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        auto result = future.get();
        if (result->success) {
            RCLCPP_INFO(getLogger(), "Vehicle disarmed successfully");
            return true;
        }
    }
    
    RCLCPP_ERROR(getLogger(), "Disarming request failed");
    return false;
}

bool ArduRoverDriver::setMoveSpeed(double speed)
{
    // 检查服务是否可用
    if (!commandClient_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(getLogger(), "Command service not available");
        return false;
    }
    
    // 创建请求
    auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    request->command = mavros_msgs::msg::CommandCode::DO_CHANGE_SPEED;
    request->param1 = 0;  // ignored by ArduPilot
    request->param2 = speed;
    
    // 发送请求
    auto future = commandClient_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        auto result = future.get();
        if (result->success) {
            RCLCPP_INFO(getLogger(), "Move speed set to: %.2f m/s", speed);
            return true;
        }
    }
    
    RCLCPP_ERROR(getLogger(), "Failed to set move speed");
    return false;
}

bool ArduRoverDriver::setMode(const std::string& mode)
{
    // 检查服务是否可用
    if (!setModeClient_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(getLogger(), "Set mode service not available");
        return false;
    }
    
    // 创建请求
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = mode;
    
    // 发送请求
    auto future = setModeClient_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        auto result = future.get();
        if (result->mode_sent) {
            RCLCPP_INFO(getLogger(), "Mode changed to: %s", mode.c_str());
            return true;
        }
    }
    
    RCLCPP_ERROR(getLogger(), "Failed to set mode to: %s", mode.c_str());
    return false;
}

void ArduRoverDriver::stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
{
    currentState_ = *msg;
}

bool ArduRoverDriver::waitForConnection(double timeout)
{
    auto startTime = node_->now();
    rclcpp::Rate rate(10);  // 10 Hz
    
    while (rclcpp::ok()) {
        if (isConnected()) {
            RCLCPP_INFO(getLogger(), "Connected to MAVROS");
            return true;
        }
        
        if ((node_->now() - startTime).seconds() > timeout) {
            return false;
        }
        
        rate.sleep();
    }
    
    return false;
}

} // namespace libracer