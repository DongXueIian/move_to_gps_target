import cv2
import apriltag
import threading
import numpy as np
import signal
import sys
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
import tf_transformations
import tf2_ros
from cv_bridge import CvBridge
import time

class AprilTagTFPublisher(Node):
    def __init__(self):
        # 在节点初始化时直接设置参数
        super().__init__('apriltag_tf_publisher',
                        parameter_overrides=[
                            rclpy.Parameter('use_sim_time', value=False)
                        ])
        
        # 初始化TF广播器和CV桥接
        self.br = tf2_ros.TransformBroadcaster(self)
        self.bridge = CvBridge()
        
        # 线程同步变量
        self.frame_condition = threading.Condition()
        self.latest_frame = None
        self.running = True
        
        # 检测仿真模式的代码需要调整
        time.sleep(1)
        simulation = self.count_publishers('/clock') > 0
        
        # 使用正确的方式更新参数
        self.set_parameters([
            rclpy.Parameter('use_sim_time', value=simulation)
        ])
        self.get_logger().info(f"Operating in {'simulation' if simulation else 'real'} mode")

        if simulation:
            # 仿真模式：订阅图像话题
            self.subscription = self.create_subscription(
                Image,
                '/camera',
                self.image_callback,
                QoSProfile(
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    durability=QoSDurabilityPolicy.VOLATILE,
                    depth=1
                )
            )
        else:
            # 真实模式：初始化摄像头
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                self.get_logger().error("Could not open video device")
                sys.exit(1)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.capture_thread = threading.Thread(target=self.capture_frames)
            self.capture_thread.start()

        # 启动检测线程
        self.detection_thread = threading.Thread(target=self.detection_loop)
        self.detection_thread.start()

    def image_callback(self, msg):
        """处理仿真图像的回调函数"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.frame_condition:
                self.latest_frame = cv_image
                self.frame_condition.notify_all()
        except Exception as e:
            self.get_logger().error(f"图像转换失败: {str(e)}")

    def capture_frames(self):
        """真实模式下的摄像头捕获循环"""
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("视频帧捕获失败")
                break
            with self.frame_condition:
                self.latest_frame = frame
                self.frame_condition.notify_all()

    def detection_loop(self):
        """持续运行的检测循环"""
        detector = apriltag.Detector()
        tag_size = 2.0
        camera_matrix, _ = read_camera_parameters()

        while self.running:
            with self.frame_condition:
                self.frame_condition.wait_for(lambda: self.latest_frame is not None or not self.running)
                if not self.running:
                    break
                frame = self.latest_frame.copy()

            # AprilTag检测处理
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            results = detector.detect(gray)
            
            for r in results:
                try:
                    # 位姿估计
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
                    translation[0], translation[1],translation[2] = -translation[1], -translation[0],-translation[2]
                    
                    # quaternion = tf_transformations.quaternion_from_matrix(pose)
                    self.send_transform(translation, r.tag_id)
                except Exception as e:
                    self.get_logger().error(f"位姿估计失败: {str(e)}")

    def send_transform(self, translation, tag_id):
        print("发布TF变换")
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = f'tag_{tag_id}'
        
        t.transform.translation.x = float(translation[0])
        t.transform.translation.y = float(translation[1])
        t.transform.translation.z = float(translation[2])
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.br.sendTransform(t)

    def destroy_node(self):
        """资源清理"""
        self.running = False
        with self.frame_condition:
            self.frame_condition.notify_all()
        
        if hasattr(self, 'cap'):
            self.cap.release()
        if hasattr(self, 'capture_thread'):
            self.capture_thread.join()
        super().destroy_node()

def read_camera_parameters(file_path='ost.yaml'):
    """读取相机标定参数"""
    with open(file_path) as file:
        data = yaml.safe_load(file)
    
    return (
        np.array(data['camera_matrix']['data']).reshape(3,3),
        np.array(data['distortion_coefficients']['data'])
    )

def main():
    rclpy.init()
    node = AprilTagTFPublisher()
    
    def shutdown(sig, frame):
        node.get_logger().info("收到关闭信号，正在终止...")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, shutdown)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()