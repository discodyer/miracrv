#include "libracer/baseDriver.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace libracer
{

BaseDriver::BaseDriver(rclcpp::Node::SharedPtr node, 
                       const std::string& cmdVelTopic,
                       const std::string& odomTopic)
    : node_(node)
    , cmdVelTopic_(cmdVelTopic)
    , odomTopic_(odomTopic)
    , baseFrameId_("base_link")
    , odomFrameId_("odom")
    , publishTf_(true)
{
    RCLCPP_INFO(getLogger(), "BaseDriver constructor called");
}

bool BaseDriver::initialize()
{
    try {
        // 创建速度命令订阅者
        cmdVelSub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
            cmdVelTopic_, 10,
            std::bind(&BaseDriver::cmdVelCallback, this, std::placeholders::_1));
        
        // 创建里程计发布者
        odomPub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odomTopic_, 10);
        
        // 创建TF广播器
        tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
        
        RCLCPP_INFO(getLogger(), "BaseDriver initialized successfully");
        RCLCPP_INFO(getLogger(), "Subscribing cmd_vel to: %s", cmdVelTopic_.c_str());
        RCLCPP_INFO(getLogger(), "Publishing odometry to: %s", odomTopic_.c_str());
        
        return true;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(getLogger(), "Failed to initialize BaseDriver: %s", e.what());
        return false;
    }
}

void BaseDriver::publishOdometry(double x, double y, double theta, 
                                double vx, double vy, double vtheta)
{
    auto currentTime = now();
    
    // 创建里程计消息
    auto odomMsg = nav_msgs::msg::Odometry();
    odomMsg.header.stamp = currentTime;
    odomMsg.header.frame_id = odomFrameId_;
    odomMsg.child_frame_id = baseFrameId_;
    
    // 设置位置
    odomMsg.pose.pose.position.x = x;
    odomMsg.pose.pose.position.y = y;
    odomMsg.pose.pose.position.z = 0.0;
    
    // 设置姿态（从欧拉角转换为四元数）
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    odomMsg.pose.pose.orientation = tf2::toMsg(q);
    
    // 设置速度
    odomMsg.twist.twist.linear.x = vx;
    odomMsg.twist.twist.linear.y = vy;
    odomMsg.twist.twist.linear.z = 0.0;
    odomMsg.twist.twist.angular.x = 0.0;
    odomMsg.twist.twist.angular.y = 0.0;
    odomMsg.twist.twist.angular.z = vtheta;
    
    // 发布里程计消息
    odomPub_->publish(odomMsg);
    
    // 发布TF变换
    if (publishTf_) {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = currentTime;
        transformStamped.header.frame_id = odomFrameId_;
        transformStamped.child_frame_id = baseFrameId_;
        
        transformStamped.transform.translation.x = x;
        transformStamped.transform.translation.y = y;
        transformStamped.transform.translation.z = 0.0;
        transformStamped.transform.rotation = tf2::toMsg(q);
        
        tfBroadcaster_->sendTransform(transformStamped);
    }
}

} // namespace libracer