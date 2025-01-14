import cv2
import apriltag
import threading
import numpy as np
import signal
import sys
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf_transformations  # 用于将旋转矩阵转换为四元数
import tf2_ros

# ROS2节点类
class AprilTagTFPublisher(Node):
    def __init__(self):
        super().__init__('apriltag_tf_publisher')
        self.br = tf2_ros.TransformBroadcaster(self)
    
    def send_transform(self, translation, quaternion, tag_id):
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = f'tag_{tag_id}'

        t.transform.translation.x = float(translation[0])
        t.transform.translation.y = float(translation[1])
        t.transform.translation.z = float(translation[2])

        t.transform.rotation.x = float(quaternion[0])
        t.transform.rotation.y = float(quaternion[1])
        t.transform.rotation.z = float(quaternion[2])
        t.transform.rotation.w = float(quaternion[3])

        self.br.sendTransform(t)

# 读取ost.yaml文件
def read_camera_parameters(file_path='ost.yaml'):
    with open(file_path) as file:
        data = yaml.safe_load(file)
    
    camera_matrix_data = data['camera_matrix']['data']
    camera_matrix = np.array(camera_matrix_data).reshape((3, 3))

    dist_coeffs_data = data['distortion_coefficients']['data']
    dist_coeffs = np.array(dist_coeffs_data)

    return camera_matrix, dist_coeffs

# 获取摄像头参数
camera_matrix, dist_coeffs = read_camera_parameters()

# 初始化摄像头
device_id = 0
cap = cv2.VideoCapture(device_id)

# 检查摄像头是否打开
if not cap.isOpened():
    print("Error: Could not open video device.")
    sys.exit()

# 设置摄像头分辨率
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# 初始化AprilTag检测器
detector = apriltag.Detector()

# 用于线程同步的条件变量
frame_condition = threading.Condition()
latest_frame = None

# 标签大小（以米为单位）
tag_size = 0.1545  # 30厘米

def capture_frames():
    global latest_frame
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 使用条件变量更新最新的帧
        with frame_condition:
            latest_frame = frame.copy()
            frame_condition.notify()

def detect_and_publish():
    rclpy.init(args=None)
    node = AprilTagTFPublisher()

    while True:
        with frame_condition:
            frame_condition.wait()
            frame = latest_frame.copy()
        
        # 转换为灰度图
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 进行AprilTag检测
        results = detector.detect(gray)
        
        for r in results:
            # 位姿估计
            pose, e0, e1 = detector.detection_pose(
                r, 
                camera_params=(
                    camera_matrix[0, 0], 
                    camera_matrix[1, 1], 
                    camera_matrix[0, 2], 
                    camera_matrix[1, 2]
                ), 
                tag_size=tag_size
            )

            translation = pose[:3, 3]
            t=translation[0]
            translation[0] = translation[1]  # 对y轴取反
            translation[1]=t
            translation[2] = -translation[2]  # 对z轴取反

            # 转换为四元数
            quaternion = tf_transformations.quaternion_from_matrix(pose)

            # 发送ROS2变换
            node.send_transform(translation, quaternion, r.tag_id)

    rclpy.shutdown()

def signal_handler(sig, frame):
    print("Ctrl+C detected. Exiting gracefully...")
    cap.release()
    sys.exit(0)

# 捕获Ctrl+C信号
signal.signal(signal.SIGINT, signal_handler)

# 创建和启动线程
capture_thread = threading.Thread(target=capture_frames)
publish_thread = threading.Thread(target=detect_and_publish)

capture_thread.start()
publish_thread.start()

# 等待线程结束
capture_thread.join()
publish_thread.join()

# 释放摄像头
cap.release()
