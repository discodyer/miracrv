#ifndef LIBRACER_BASE_DRIVER_HPP
#define LIBRACER_BASE_DRIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <memory>
#include <string>

namespace libracer
{

/**
 * @brief 底盘驱动基类，定义了小车底盘最基本的功能接口
 */
class BaseDriver
{
public:
    /**
     * @brief 构造函数
     * @param node ROS2节点指针
     * @param cmdVelTopic 速度控制话题名称
     * @param odomTopic 里程计话题名称
     */
    BaseDriver(rclcpp::Node::SharedPtr node, 
               const std::string& cmdVelTopic = "/cmd_vel",
               const std::string& odomTopic = "/odom");

    /**
     * @brief 虚析构函数
     */
    virtual ~BaseDriver() = default;

    /**
     * @brief 初始化驱动
     * @return 初始化是否成功
     */
    virtual bool initialize();

    /**
     * @brief 设置是否发布TF变换
     * @param enable 是否启用TF发布
     */
    void setPublishTf(bool enable) { publishTf_ = enable; }

    /**
     * @brief 设置底盘坐标系名称
     * @param frameId 坐标系名称
     */
    void setBaseFrameId(const std::string& frameId) { baseFrameId_ = frameId; }

    /**
     * @brief 设置里程计坐标系名称
     * @param frameId 坐标系名称
     */
    void setOdomFrameId(const std::string& frameId) { odomFrameId_ = frameId; }

protected:
    /**
     * @brief 速度控制回调函数（纯虚函数，子类必须实现）
     * @param msg 速度控制消息
     */
    virtual void cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) = 0;

    /**
     * @brief 发布里程计信息
     * @param x X位置（米）
     * @param y Y位置（米）
     * @param theta 航向角（弧度）
     * @param vx X方向速度（米/秒）
     * @param vy Y方向速度（米/秒）
     * @param vtheta 角速度（弧度/秒）
     */
    void publishOdometry(double x, double y, double theta, 
                        double vx, double vy, double vtheta);

    /**
     * @brief 获取日志器
     * @return ROS2日志器
     */
    rclcpp::Logger getLogger() const { return node_->get_logger(); }

    /**
     * @brief 获取当前时间
     * @return ROS2时间
     */
    rclcpp::Time now() const { return node_->now(); }

protected:
    // ROS2节点
    rclcpp::Node::SharedPtr node_;
    
    // 发布者和订阅者
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmdVelSub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub_;
    
    // TF广播器
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
    
    // 话题名称
    std::string cmdVelTopic_;
    std::string odomTopic_;
    
    // 坐标系名称
    std::string baseFrameId_;
    std::string odomFrameId_;
    
    // 配置选项
    bool publishTf_;
};

} // namespace libracer

#endif // LIBRACER_BASE_DRIVER_HPP