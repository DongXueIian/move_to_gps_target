import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
import math
from rosgraph_msgs.msg import Clock 
from std_msgs.msg import Header

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
        self.odom_frame_id = 'odom'
        self.base_link_frame_id = 'base_link'
        self.local_pose = None
        self.attitude = None
        self.loop_action_20hz_timer= self.create_timer(0.04, self.update_tf)
    def local_location_callback(self, msg):
        self.local_pose = msg.pose
        # print(str(self.get_clock().now().to_msg())+"---"+str(msg.pose))
    def sim_clock_callback(self,msg):
        global simClock
        # print(simClock)
        simClock=msg
    def attitude_callback(self, msg):
        self.attitude = msg.data

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
