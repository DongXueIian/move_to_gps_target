import cv2
import apriltag
import threading
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
import tf_transformations
import tf2_ros
from cv_bridge import CvBridge
import argparse
import psutil
import time
import sys

class AprilTagTFPublisher(Node):
    def __init__(self):
        super().__init__('apriltag_tf_publisher')
        # 初始化TF广播器和CV桥接
        self.br = tf2_ros.TransformBroadcaster(self)
        self.bridge = CvBridge()
        
        # 线程同步变量
        self.frame_condition = threading.Condition()
        self.latest_frame = None
        self.running = True
        
        # 订阅图像话题
        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=10
            )
        )
        
        # 启动检测线程
        self.detection_thread = threading.Thread(target=self.detection_loop)
        self.detection_thread.start()

    def image_callback(self, msg):
        """图像话题回调函数"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.frame_condition:
                self.latest_frame = cv_image
                self.frame_condition.notify_all()
        except Exception as e:
            self.get_logger().error(f"图像转换失败: {str(e)}")

    def detection_loop(self):
        """5Hz检测循环"""
        detector = apriltag.Detector()
        tag_size = 2.0
        camera_matrix, _ = self.read_camera_parameters()
        
        # 速率控制
        rate = self.create_rate(5)  # 设置为 5Hz
        
        while self.running:
            print("detection_loop")

            start_time = time.time()
            
            # 获取最新帧
            with self.frame_condition:
                if self.latest_frame is None:
                    rate.sleep()
                    continue
                frame = self.latest_frame.copy()
            
            # AprilTag检测处理
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            results = detector.detect(gray)
            
            for r in results:
                try:
                    # 位姿估计（保持原有处理逻辑）
                    pose, _, _ = detector.detection_pose(
                        r,
                        camera_params=(
                            camera_matrix[0,0],
                            camera_matrix[1,1],
                            camera_matrix[0,2],
                            camera_matrix[1,2]
                        ),
                        tag_size=tag_size
                    )
                    
                    # 坐标变换处理
                    translation = pose[:3, 3]
                    translation[0], translation[1], translation[2] = -translation[1], -translation[0], -translation[2]
                    
                    quaternion = tf_transformations.quaternion_from_matrix(pose)
                    self.send_transform(translation, quaternion, r.tag_id)
                    
                except Exception as e:
                    self.get_logger().error(f"位姿估计失败: {str(e)}")
            
            # 精确速率控制
            elapsed = time.time() - start_time
            sleep_time = max(0.2 - elapsed, 0)  # 控制每次循环的时间间隔为 0.2秒（即5Hz）
            time.sleep(sleep_time)

    def send_transform(self, translation, quaternion, tag_id):
        """发布TF变换"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_link'  # 根据实际坐标系修改
        t.child_frame_id = f'tag_{tag_id}'
        
        t.transform.translation.x = float(translation[0])
        t.transform.translation.y = float(translation[1])
        t.transform.translation.z = float(translation[2])
        
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]
        
        print(t)

        self.br.sendTransform(t)

    def read_camera_parameters(self, file_path='ost.yaml'):
        """读取相机标定参数"""
        with open(file_path) as file:
            data = yaml.safe_load(file)
        
        return (
            np.array(data['camera_matrix']['data']).reshape(3,3),
            np.array(data['distortion_coefficients']['data'])
        )

    def destroy_node(self):
        """资源清理"""
        self.running = False
        with self.frame_condition:
            self.frame_condition.notify_all()
        if hasattr(self, 'detection_thread'):
            self.detection_thread.join()
        super().destroy_node()

def main(args=None):
    # 在rclpy初始化之前解析命令行参数
    parser = argparse.ArgumentParser(description="apriltag tf publisher node")
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
    node = AprilTagTFPublisher()
    

    # 检测仿真时钟（新增代码）
    start_time = time.time()
    clock_detected = False
    while time.time() - start_time < 5:  # 最多等待5秒
        if node.count_publishers('/clock') > 0:
            clock_detected = True
            break
        node.get_logger().info("等待/clock话题...", throttle_duration_sec=1)
        time.sleep(0.5)

    if clock_detected:
        # 设置仿真时间模式
        node.set_parameters([rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        node.get_logger().info("已启用仿真时间模式")
    else:
        node.get_logger().info("使用系统时间")

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
