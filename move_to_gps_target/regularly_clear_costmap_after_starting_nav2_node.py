import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
import time

class MySimpleNav2Command(Node):
    def __init__(self,timeLocal,timeGlobal):
        # 调用父类（Node类）的初始化函数，并设置节点名称为'regularly_clear_costmap_after_starting_nav2_node'
        super().__init__('regularly_clear_costmap_after_starting_nav2_node')
        self.nav2 = BasicNavigator()
        # 确保nav2已经完全启动
        time.sleep(10)
        # 创建一个定时器，定期清理全局和局部代价地图
        # self.timer = self.create_timer(timeLocal, self.nav2.clearLocalCostmap)
        self.timer = self.create_timer(timeGlobal, self.nav2.clearGlobalCostmap)


# 定义main函数，是程序的入口点
def main():
    # 初始化rclpy库
    rclpy.init()
    my_nav_command = MySimpleNav2Command(2.0,10.0)
    # 让ROS 2节点运行起来，监听和处理回调函数，直到节点被显式地关闭
    rclpy.spin(my_nav_command)
    # 销毁节点，进行清理
    my_nav_command.destroy_node()
    # 关闭rclpy库，释放资源
    rclpy.shutdown()

# Python的标准模板，当该文件被直接运行时，会执行main函数
if __name__ == '__main__':
    main()