#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf_transformations import quaternion_from_euler
from rclpy.parameter import Parameter
import psutil
import os
import argparse

cpu_core_msg=''
cpu_core_error_msg=''

class MultiStaticTransformBroadcaster(Node):
    def __init__(self):
        super().__init__('multi_static_tf_broadcaster')

        global cpu_core_msg,cpu_core_error_msg
        if(cpu_core_msg != ''):
            self.get_logger().info(cpu_core_msg)
        else:
            self.get_logger().error(cpu_core_error_msg)

        # 初始化 StaticTransformBroadcaster
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # 声明参数 'transforms'，并指定默认值为包含一个空字符串的列表
        self.declare_parameter('transforms', ['0 0 0 0 0 0 map odom'])

        # 获取参数
        transforms = self.get_parameter('transforms').get_parameter_value().string_array_value

        # 解析参数并发布静态变换
        static_transforms = []
        for tf_str in transforms:
            # 期望每个变换为一个字符串，以空格分隔的 8 个值
            # 格式: x y z roll pitch yaw frame_id child_frame_id
            parts = tf_str.split()
            if len(parts) != 8:
                self.get_logger().warn(f"Invalid transform format: '{tf_str}'. Expected 8 elements.")
                continue

            try:
                x, y, z, roll, pitch, yaw = map(float, parts[:6])
                frame_id, child_frame_id = parts[6], parts[7]
            except ValueError as e:
                self.get_logger().warn(f"Error parsing transform '{tf_str}': {e}")
                continue

            # 创建 TransformStamped 消息
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = frame_id
            t.child_frame_id = child_frame_id
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z

            # 将欧拉角转换为四元数
            quat = quaternion_from_euler(roll, pitch, yaw)
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]

            static_transforms.append(t)
            self.get_logger().info(f"Publishing static transform from '{frame_id}' to '{child_frame_id}'")

        if static_transforms:
            # 发布所有静态变换
            self.broadcaster.sendTransform(static_transforms)
            self.get_logger().info("All static transforms have been published.")
        else:
            self.get_logger().warn("No valid transforms to publish.")

def main(args=None):
    # 在rclpy初始化之前解析命令行参数
    parser = argparse.ArgumentParser(description="APM Controller Node")
    parser.add_argument('--cpu', type=int, default=None, help="CPU core to bind to")
    parsed_args, unknown_args = parser.parse_known_args(args)  # 解析CPU参数

    # 只处理CPU核心绑定参数，其他参数传递给rclpy
    global cpu_core_msg,cpu_core_error_msg
    if parsed_args.cpu is not None:
        bind_to_cpu(parsed_args.cpu)
    else:
        cpu_core_error_msg='parsed_args.cpu is None'

    # 初始化rclpy并传递未被argparse处理的参数
    rclpy.init(args=unknown_args)

    node = MultiStaticTransformBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

def bind_to_cpu(cpu_core):
    global cpu_core_msg,cpu_core_error_msg
    try:
        p = psutil.Process()  # 获取当前进程
        p.cpu_affinity([cpu_core])  # 设置进程的CPU核心亲和力
        cpu_core_msg=f'Process is bound to CPU core {cpu_core}'
        print(f'Process is bound to CPU core {cpu_core}')
    except Exception as e:
        cpu_core_error_msg=f'Failed to set CPU affinity: {e}'
        print(f'Failed to set CPU affinity: {e}')

if __name__ == '__main__':
    main()
