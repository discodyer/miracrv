#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
import math
import tf_transformations
from geometry_msgs.msg import Quaternion

class MapToBaseLinkTFNode(Node):
    def __init__(self):
        super().__init__('map_to_base_link_tf_publisher')
        # TF2相关初始化
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # 创建发布器
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose',
            10
        )
        
        # 添加标志位，跟踪是否已经检测到过有效的坐标变换
        self.transform_detected = False
        
        # 设置定时器定期执行回调
        self.timer = self.create_timer(
            0.05,  # 20Hz
            self.timer_callback
        )
        self.get_logger().info("Start the map to base_link transformation publishing node")
        self.get_logger().info("Publishing zero odometry until valid transform is detected...")

    def publish_zero_pose(self):
        """发布全零的里程计信息"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        # 位置全为0
        pose_msg.pose.position.x = 0.0
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = 0.0
        
        # 姿态为单位四元数（无旋转）
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.707106771713121
        pose_msg.pose.orientation.w = 0.707106790659974
        
        self.pose_pub.publish(pose_msg)

    def timer_callback(self):
        try:
            # 获取最新的变换
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # 如果是第一次检测到有效变换，记录并输出日志
            if not self.transform_detected:
                self.transform_detected = True
                self.get_logger().info("Valid transform detected! Switching to real odometry data.")
            
            # 构造PoseStamped消息
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'

            roll_lidar = 0       # lidar的roll角
            pitch_lidar = 0      # lidar的pitch角
            yaw_lidar = 0        # lidar的yaw角
            gamma_world = -1.5707963      # 旋转偏移角（单位：弧度）

            # 填充位置和姿态 - 修复数学表达式
            pose_msg.pose.position.x = math.cos(gamma_world) * transform.transform.translation.x + math.sin(gamma_world) * transform.transform.translation.y
            pose_msg.pose.position.y = math.sin(gamma_world) * transform.transform.translation.x * -1 + math.cos(gamma_world) * transform.transform.translation.y
            pose_msg.pose.position.z = transform.transform.translation.z

            quat_lidar = transform.transform.rotation
            quat_lidar_tf = [quat_lidar.x, quat_lidar.y, quat_lidar.z, quat_lidar.w]

            # 创建body本身的旋转四元数
            quat_body = tf_transformations.quaternion_from_euler(roll_lidar, pitch_lidar, yaw_lidar)

            # 绕Z轴的修正旋转
            quat_rot_z = tf_transformations.quaternion_from_euler(0, 0, -gamma_world)

            # 四元数乘法（注意顺序：rot_z * lidar * body）
            quat_combined = tf_transformations.quaternion_multiply(
                tf_transformations.quaternion_multiply(quat_rot_z, quat_lidar_tf),
                quat_body
            )

            # 归一化四元数
            norm = math.sqrt(sum(q**2 for q in quat_combined))
            quat_normalized = [q / norm for q in quat_combined]

            quat_msg = Quaternion()
            quat_msg.x = quat_normalized[0]
            quat_msg.y = quat_normalized[1]
            quat_msg.z = quat_normalized[2]
            quat_msg.w = quat_normalized[3]

            pose_msg.pose.orientation = quat_msg
            # 发布消息
            self.pose_pub.publish(pose_msg)
            
        except TransformException as ex:
            # 如果还没有检测到过有效变换，发送零值里程计
            if not self.transform_detected:
                self.publish_zero_pose()
            else:
                # 已经检测到过有效变换，但当前获取失败，输出警告但不发送任何数据
                self.get_logger().warning(
                    f'Transform was lost: {ex}',
                    throttle_duration_sec=5.0  # 限流，避免大量警告
                )

def main(args=None):
    rclpy.init(args=args)
    node = MapToBaseLinkTFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':  # 修复语法错误
    main()