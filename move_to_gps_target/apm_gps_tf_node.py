import rclpy
from rclpy.node import Node
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped, PoseStamped
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
import math
from rosgraph_msgs.msg import Clock 
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix
import utm
import psutil
import os
import argparse


apmControllernNameSpace='/apm_drone'

cpu_core_msg=''
cpu_core_error_msg=''

hasClock=False
simClock=None

class TFConverterNode(Node):
    def __init__(self):
        global hasClock,simClock
        super().__init__('tf_converter_node')

        global cpu_core_msg,cpu_core_error_msg
        if(cpu_core_msg != ''):
            self.get_logger().info(cpu_core_msg)
        else:
            self.get_logger().error(cpu_core_error_msg)


        self.tf_broadcaster = TransformBroadcaster(self)
        for topic_name, topic_type in self.get_topic_names_and_types():
            # print(self.get_topic_names_and_types())
            if topic_name == '/clock':
                # print(topic_name)
                hasClock = True
        if hasClock:
            self.subscription_velocity = self.create_subscription(
                Clock,
                '/clock',
                self.sim_clock_callback,
                10)
        self.subscription_local = self.create_subscription(
            PoseStamped,
            apmControllernNameSpace+'/current_local_location',
            self.local_location_callback,
            10)
        self.subscription_attitude = self.create_subscription(
            String,
            apmControllernNameSpace+'/current_attitude',
            self.attitude_callback,
            10)

        self.subscription_target_gps = self.create_subscription(
            NavSatFix,
            apmControllernNameSpace+'/target_gps_location',
            self.target_gps_callback,
            10
        )
        self.subscription_target_gps = self.create_subscription(
            NavSatFix,
            apmControllernNameSpace+'/home_gps_location',
            self.home_gps_callback,
            10
        )
        self.goal_publisher = self.create_publisher(
            PoseStamped, 
            '/goal_pose', 
            10
        )

        self.odom_frame_id = 'odom'
        self.base_link_frame_id = 'base_link'
        self.local_pose = None
        self.attitude = None
        self.target_gps=None
        self.home_gps=None
        self.loop_action_10hz_timer= self.create_timer(0.1, self.update_tf)
        # self.loop_action_1hz_timer= self.create_timer(1.0, self.update_1hz)

    def local_location_callback(self, msg):
        self.local_pose = msg.pose
        # print(str(self.get_clock().now().to_msg())+"---"+str(msg.pose))

    def sim_clock_callback(self,msg):
        global simClock
        # print(simClock)
        simClock=msg

    def attitude_callback(self, msg):
        self.attitude = msg.data

    def target_gps_callback(self, msg):
        self.target_gps=msg

    def home_gps_callback(self, msg):
        self.home_gps=msg

    def update_tf(self):
        if self.local_pose is not None and self.attitude is not None:
            # currentTime=str(self.get_clock().now().to_msg())
            t = TransformStamped()
            if hasClock and simClock!=None:
                # print(simClock)
                t.header = Header()
                t.header.stamp = simClock.clock
            else:
                t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.odom_frame_id
            t.child_frame_id = self.base_link_frame_id

            t.transform.translation.x = self.local_pose.position.x
            t.transform.translation.y = -self.local_pose.position.y
            t.transform.translation.z = -self.local_pose.position.z

            # 解析姿态数据字符串，例如 'Roll: -0.0003453207609709352, Pitch: 0.0001045228709699586, Yaw: 1.6075834035873413'
            parts = self.attitude.split(', ')
            roll = float(parts[0].split(': ')[1])
            pitch = float(parts[1].split(': ')[1])
            yaw = -float(parts[2].split(': ')[1])

            # 将欧拉角转换为四元数
            quaternion = quaternion_from_euler(roll, pitch, yaw)

            t.transform.rotation.x = quaternion[0]
            t.transform.rotation.y = quaternion[1]
            t.transform.rotation.z = quaternion[2]
            t.transform.rotation.w = quaternion[3]

            self.tf_broadcaster.sendTransform(t)

            # print(str(t.transform)+"---"+str(self.get_clock().now().to_msg()))

    def update_1hz(self):
        # print('update_1hz')
        if self.target_gps!=None and self.home_gps!=None:
            # print('转换起飞点和目标点到 UTM 坐标')
            # 转换起飞点和目标点到 UTM 坐标
            home_utm = utm.from_latlon(self.home_gps.latitude, self.home_gps.longitude)
            target_utm = utm.from_latlon(self.target_gps.latitude, self.target_gps.longitude)
            
            # 创建和发送 TF 消息
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'map'
            transform.child_frame_id = 'target_location'
            transform.transform.translation.x = target_utm[0] - home_utm[0]
            transform.transform.translation.y = target_utm[1] - home_utm[1]
            transform.transform.translation.z = 0.0  # 假设高度变化不考虑
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = 0.0
            transform.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(transform)

            # 创建并发布 PoseStamped 消息到 /goal_pose
            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = 'map'
            goal_msg.pose.position.x = transform.transform.translation.x
            goal_msg.pose.position.y = transform.transform.translation.y
            goal_msg.pose.position.z = 0.0
            goal_msg.pose.orientation.x = 0.0
            goal_msg.pose.orientation.y = 0.0
            goal_msg.pose.orientation.z = 0.0
            goal_msg.pose.orientation.w = 1.0

            self.goal_publisher.publish(goal_msg)
            self.get_logger().info(f'Published goal pose: x={goal_msg.pose.position.x}, y={goal_msg.pose.position.y}')

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

    node = TFConverterNode()
    rclpy.spin(node)
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
