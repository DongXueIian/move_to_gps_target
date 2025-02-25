import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import psutil
import argparse

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # 等待1秒让系统发现已有话题
        time.sleep(1)
        
        self.initialization_succeeded = True  # 新增初始化状态标志
        
        # 检查/camera话题是否存在
        time.sleep(5)
        if self.count_publishers('/camera') > 0:
            self.get_logger().info("/camera话题已存在，自动销毁节点")
            self.initialization_succeeded = False
            self.destroy_node()
            return

        # 初始化摄像头
        self.cap = cv2.VideoCapture("/dev/video0")
        if not self.cap.isOpened():
            self.get_logger().error(f"无法打开摄像头设备 /dev/video0 (错误码: {self.cap.get(cv2.CAP_PROP_POS_MSEC)})")
            self.initialization_succeeded = False
            self.destroy_node()
            return
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # 创建图像发布者
        self.publisher = self.create_publisher(
            Image,
            '/camera',
            QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=10
            )
        )
        self.bridge = CvBridge()
        
        # 启动摄像头线程
        self.running = True
        self.capture_thread = threading.Thread(target=self.capture_frames)
        self.capture_thread.start()
        self.get_logger().info("摄像头数据发布节点已启动")

    def capture_frames(self):
        """持续捕获并发布摄像头帧"""
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("视频帧捕获失败")
                break
            
            try:
                # 显示画面（新增以下两行）
                # cv2.imshow('Camera View', frame)
                # cv2.waitKey(1)  # 必须调用waitKey才能正常显示
                msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                msg.header.stamp = self.get_clock().now().to_msg()  # 新增时间戳
                msg.header.frame_id = "camera_frame"  # 新增坐标系标识
                self.publisher.publish(msg)
            except Exception as e:
                self.get_logger().error(f"图像转换失败: {str(e)}")

    def destroy_node(self):
        self.running = False
        if hasattr(self, 'cap'):
            self.cap.release()
        if hasattr(self, 'capture_thread'):
            self.capture_thread.join()
        super().destroy_node()

def main(args=None):
    # 在rclpy初始化之前解析命令行参数
    parser = argparse.ArgumentParser(description="camera_publisher_node")
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
    node = CameraPublisher()
    
    # 新增初始化状态检查
    if not node.initialization_succeeded:
        rclpy.shutdown()
        return 1  # 返回非零状态码表示异常退出


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
