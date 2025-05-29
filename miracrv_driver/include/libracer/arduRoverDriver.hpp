#ifndef LIBRACER_ARDU_ROVER_DRIVER_HPP
#define LIBRACER_ARDU_ROVER_DRIVER_HPP

#include "libracer/baseDriver.hpp"
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <geographic_msgs/msg/geo_point_stamped.hpp>

namespace libracer
{

/**
 * @brief ArduPilot Rover驱动类，继承自BaseDriver
 */
class ArduRoverDriver : public BaseDriver
{
public:
    /**
     * @brief 构造函数
     * @param node ROS2节点指针
     */
    explicit ArduRoverDriver(rclcpp::Node::SharedPtr node);

    /**
     * @brief 析构函数
     */
    virtual ~ArduRoverDriver() = default;

    /**
     * @brief 初始化驱动
     * @return 初始化是否成功
     */
    virtual bool initialize() override;

    /**
     * @brief 设置全局坐标原点
     * @param latitude 纬度
     * @param longitude 经度
     * @param altitude 高度（默认为0）
     */
    void setGlobalOrigin(double latitude, double longitude, double altitude = 0.0);

    /**
     * @brief 解锁电机
     * @return 解锁是否成功
     */
    bool arm();

    /**
     * @brief 上锁电机
     * @return 上锁是否成功
     */
    bool disarm();

    /**
     * @brief 设置移动速度
     * @param speed 速度值（米/秒）
     * @return 设置是否成功
     */
    bool setMoveSpeed(double speed);

    /**
     * @brief 设置移动速度限制
     * @param maxLinearX 速度值（米/秒）
     * @param maxLinearY 速度值（米/秒）
     * @param maxAngularZ 旋转速度（弧度/秒）
     */
    void setMoveSpeedLimit(double maxLinearX, double maxLinearY, double maxAngularZ);

    /**
     * @brief 设置飞行模式
     * @param mode 模式名称（如"GUIDED", "MANUAL"等）
     * @return 设置是否成功
     */
    bool setMode(const std::string& mode);

    /**
     * @brief 获取当前状态
     * @return 当前MAVROS状态
     */
    mavros_msgs::msg::State getCurrentState() const { return currentState_; }

    /**
     * @brief 检查是否已连接
     * @return 是否已连接到飞控
     */
    bool isConnected() const { return currentState_.connected; }

    /**
     * @brief 检查是否已解锁
     * @return 电机是否已解锁
     */
    bool isArmed() const { return currentState_.armed; }

protected:
    /**
     * @brief 速度控制回调函数实现
     * @param msg 速度控制消息
     */
    virtual void cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) override;

    /**
     * @brief 设置车体坐标系下的速度
     * @param vx 前进速度（米/秒）
     * @param vy 横向速度（米/秒）
     * @param yawRate 偏航角速度（弧度/秒）
     */
    void setBodyVelocity(double vx, double vy, double yawRate);

    /**
     * @brief 停止移动
     */
    void setBreak();

    /**
     * @brief 设置车体坐标系下的偏航角速度
     * @param yawRate 偏航角速度（弧度/秒）
     */
    void setAngularRate(double yawRate);

private:
    /**
     * @brief MAVROS状态回调函数
     * @param msg 状态消息
     */
    void stateCallback(const mavros_msgs::msg::State::SharedPtr msg);

    /**
     * @brief 等待MAVROS连接
     * @param timeout 超时时间（秒）
     * @return 是否成功连接
     */
    bool waitForConnection(double timeout = 5.0);

private:
    // MAVROS相关发布者
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr positionTargetPub_;
    rclcpp::Publisher<geographic_msgs::msg::GeoPointStamped>::SharedPtr globalOriginPub_;
    
    // MAVROS相关订阅者
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr stateSub_;
    
    // MAVROS服务客户端
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr armingClient_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr setModeClient_;
    rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr commandClient_;
    
    // 当前状态
    mavros_msgs::msg::State currentState_;
    
    // 配置参数
    bool autoArm_;
    std::string defaultMode_;

    // 速度限制
    double maxLinearX_;
    double maxLinearY_;
    double maxAngularZ_;

};

} // namespace libracer

#endif // LIBRACER_ARDU_ROVER_DRIVER_HPP