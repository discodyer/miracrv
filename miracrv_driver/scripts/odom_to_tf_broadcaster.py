#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTFBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_to_tf_broadcaster')
        
        # 创建tf广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 创建QoS配置，设置为BEST_EFFORT
        qos_profile = QoSProfile(
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 订阅里程计话题
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/miracz7/Odom',
            self.odom_callback,
            qos_profile
        )
        
        self.get_logger().info('Odom to TF broadcaster node started')
        
    def odom_callback(self, msg):
        """处理里程计消息并广播tf变换"""
        
        # 创建TransformStamped消息
        transform = TransformStamped()
        
        # 设置时间戳
        transform.header.stamp = msg.header.stamp
        
        # 设置父坐标系（通常是odom）
        transform.header.frame_id = 'odom'
        
        # 设置子坐标系（从里程计消息的child_frame_id获取，通常是base_link）
        # transform.child_frame_id = msg.child_frame_id
        transform.child_frame_id = 'base_footprint'
        
        # 设置平移变换
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        
        # 设置旋转变换（四元数）
        transform.transform.rotation.x = msg.pose.pose.orientation.x
        transform.transform.rotation.y = msg.pose.pose.orientation.y
        transform.transform.rotation.z = msg.pose.pose.orientation.z
        transform.transform.rotation.w = msg.pose.pose.orientation.w
        
        # 广播变换
        self.tf_broadcaster.sendTransform(transform)
        
        # 可选：打印调试信息
        self.get_logger().debug(
            f'Broadcasting transform from {transform.header.frame_id} to {transform.child_frame_id}'
        )


def main(args=None):
    rclpy.init(args=args)
    
    node = OdomToTFBroadcaster()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()