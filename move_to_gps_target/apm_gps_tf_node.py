import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
import math
from rosgraph_msgs.msg import Clock 
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix
import utm
apmControllernNameSpace='/apm_drone'

hasClock=False
simClock=None

class TFConverterNode(Node):
    def __init__(self):
        global hasClock,simClock
        super().__init__('tf_converter_node')
        self.tf_broadcaster = TransformBroadcaster(self)
        for topic_name, topic_type in self.get_topic_names_and_types():
            # print(self.get_topic_names_and_types())
            if topic_name == '/clock':
                # print(topic_name)
                hasClock = True
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
        if hasClock:
            self.subscription_velocity = self.create_subscription(
                Clock,
                '/clock',
                self.sim_clock_callback,
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
        self.gps_changed=5
        self.loop_action_40hz_timer= self.create_timer(0.025, self.update_tf)
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
        print('target_gps_callback')
        if self.target_gps==None or (self.target_gps.latitude!=msg.latitude and self.target_gps.longitude!=msg.longitude):
            self.gps_changed=0
            self.target_gps=msg
        # print('target_gps    '+str(msg))

    def home_gps_callback(self, msg):
        print('home_gps_callback')
        if self.home_gps==None or (self.home_gps.latitude!=msg.latitude and self.home_gps.longitude!=msg.longitude):
            self.home_gps=msg
            self.gps_changed=0
        # print('home_gps    '+str(msg))

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

            # 根据roll、pitch和yaw计算四元数
            quaternion = self.calculate_quaternion(roll, pitch, yaw)

            t.transform.rotation.x = quaternion[0]
            t.transform.rotation.y = quaternion[1]
            t.transform.rotation.z = quaternion[2]
            t.transform.rotation.w = quaternion[3]

            self.tf_broadcaster.sendTransform(t)

            # print(str(t.transform)+"---"+str(self.get_clock().now().to_msg()))

    def update_1hz(self):
        # print('update_1hz')
        if self.gps_changed<5 and self.target_gps!=None and self.home_gps!=None:
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

            self.gps_changed=self.gps_changed+1





    def calculate_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cy * cp * cr + sy * sp * sr
        qx = cy * cp * sr - sy * sp * cr
        qy = sy * cp * sr + cy * sp * cr
        qz = sy * cp * cr - cy * sp * sr

        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = TFConverterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
