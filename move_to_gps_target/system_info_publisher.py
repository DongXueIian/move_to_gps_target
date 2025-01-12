import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import psutil
import json

class SystemInfoPublisher(Node):
    def __init__(self):
        super().__init__('system_info_publisher')  # 创建一个名为 'system_info_publisher' 的节点
        self.publisher_ = self.create_publisher(String, 'system_info', 10)  # 创建一个发布者，发布到 'system_info' 话题
        self.timer = self.create_timer(1.0, self.publish_system_info)  # 每秒调用一次 publish_system_info 方法
    
    def publish_system_info(self):
        # 获取每个CPU核心的占用率
        cpu_usage = psutil.cpu_percent(interval=0, percpu=True)
        memory_info = psutil.virtual_memory()

        # 构建 JSON 格式的数据
        system_info = {
            "cpu_usage": cpu_usage,  # 包含每个 CPU 核心的使用率
            "memory_usage": memory_info.percent  # 内存使用率
        }
        
        # 将数据转为 JSON 字符串
        system_info_json = json.dumps(system_info)
        
        # 创建消息并发布 JSON 字符串
        system_info_msg = String()
        system_info_msg.data = system_info_json
        
        # 发布系统信息
        self.publisher_.publish(system_info_msg)
        # self.get_logger().info(f"Published system info: {system_info_msg.data}")

def main(args=None):
    rclpy.init(args=args)  # 初始化 ROS 2 系统
    system_info_publisher = SystemInfoPublisher()  # 创建节点实例
    rclpy.spin(system_info_publisher)  # 运行节点，直到节点关闭
    
    # 清理资源
    system_info_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
