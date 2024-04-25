# coding: utf-8
import sys, select, os
import tty, termios
from std_msgs.msg import String
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import subprocess
# import rospy
import threading
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Quaternion,Twist
from math import degrees
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
from std_msgs.msg import Float32 
import math
from rclpy.executors import SingleThreadedExecutor

mode=''

MAX_LINEAR = 10
MAX_ANG_VEL = 3.1415
LINEAR_STEP_SIZE = 0.1
ANG_VEL_STEP_SIZE = 3.1415/36
TAKE_OFF_ALTITUDE=1

forward  = 0.0
leftward  = 0.0
upward  = 0.0
angular = 0

cmd_vel_flu=None
vehicle=None

ros2NameSpace='drone'

lastTime=0

msg = """
保持冷静，控制你的飞机
w/x : 增加/减少前后的速度
a/d : 增加/减少左右的速度
i/, : 增加/减少上下的速度
j/l : 增加/减少旋转的速度
r   : 返航模式
t/y : arm/disarm
v/n : 起飞/降落
b   : GUIDED上位机模式(键盘控制模式)
s/k : 切换到GUIDED并且使所有速度降为0
u   : STABILIZE遥控器控制模式
o   : MOVEBASE动作控制模式
CTRL-C 退出（只有在降落之后才能退出）
"""

e = """
Communications Failed
"""

def check_connection(ip,port):
    try:
        output = subprocess.check_output(['nc', '-zv', ip, port], stderr=subprocess.STDOUT).decode()
        if("succeeded!" in output):return True
        else :return False
    except subprocess.CalledProcessError:
        return False
    
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def print_msg():
    global vehicle
    print(msg)      
    # 获取当前速度
    current_velocity = vehicle.velocity

    # 获取当前姿态信息
    current_attitude = vehicle.attitude

    # 输出当前速度和Z轴角速度
    print("当前速度 (地面参考坐标系)：", current_velocity)
    print("当前Z轴角度 (地面参考坐标系)：", current_attitude.yaw)

def tryConnect(ipAddress):
    print('Connecting '+ipAddress)
    vehicle = connect(ipAddress, wait_ready=True, baud=921600)
    print('successfully connect to '+ipAddress)
    lastTime=time.time()
    while not vehicle.is_armable:
      if(time.time()-lastTime>1):
          print("waiting to be armable")
          lastTime=time.time()
    return vehicle

#定义发送mavlink速度命令的功能
def set_velocity_body(vehicle, vx, vy, vz,vr):
    vy=-vy
    vz=-vz
    vr=-vr
    """ Remember: vz is positive downward!!! 
    Bitmask to indicate which dimensions should be ignored by the vehicle 
    (a value of 0b0000000000000000 or 0b0000001000000000 indicates that 
    none of the setpoint dimensions should be ignored). Mapping: 
    bit 1: x,  bit 2: y,  bit 3: z, 
    bit 4: vx, bit 5: vy, bit 6: vz, 
    bit 7: ax, bit 8: ay, bit 9:    

    "记住:vz是向下的正方向!!!

    掩码用于指示飞行器应忽略哪些维度
    (值为0b0000000000000000或0b0000001000000000表示
    不应忽略任何设置维度）。映射关系:
    第1位:x,第2位:y,第3位:z,
    第4位:vx,第5位:vy,第6位:vz,
    第7位:ax,第8位:ay,第9位:
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            1479, #-- BITMASK -> Consider only the velocities
            0, 0, 0,        #-- POSITION
            vx, vy, vz,     #-- VELOCITY
            0, 0, 0,        #-- ACCELERATIONS
            0, vr)
    vehicle.send_mavlink(msg)
    vehicle.flush()

def node_spin(node):
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()

class DroneSpeedPublisher(Node):
    def __init__(self):
        super().__init__('drone_speed_publisher')
        # 根据给定的命名空间构造完整的主题名称
        topic_name = f'/{ros2NameSpace}/vel'
        self.publisher = self.create_publisher(Twist, topic_name, 10)
        timer_period = 0.05  # 20Hz
        self.timer = self.create_timer(timer_period, self.publish_drone_speed)
        self.drone_speed = Twist()

    def publish_drone_speed(self):
        global vehicle
        # 假设获取速度和姿态
        current_velocity = vehicle.velocity  # [0.04, -1.16, 0.0]
        current_attitude = vehicle.attitude  # -1.7527387142181396

        # 使用当前的偏航角来旋转速度向量
        yaw = current_attitude.yaw
        # print('---------------------------')
        # print(yaw)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        # 旋转速度向量
        # 设置x, y, z方向的速度
        self.drone_speed.linear.x = current_velocity[0] * cos_yaw - current_velocity[1] * sin_yaw
        self.drone_speed.linear.y = current_velocity[0] * sin_yaw + current_velocity[1] * cos_yaw
        self.drone_speed.linear.z = current_velocity[2]  # Z方向的速度通常不受偏航角影响

        # 发布速度信息
        self.publisher.publish(self.drone_speed)
        # self.get_logger().info(f'Publishing: x={vel_x:.2f}, y={vel_y:.2f}, z={vel_z:.2f}')

def start_drone_speed_publisher_node():
    if not rclpy.ok():
        rclpy.init()
    drone_speed_publisher = DroneSpeedPublisher()
    try:
        rclpy.spin(drone_speed_publisher)
    except KeyboardInterrupt:
        # Properly handle ROS node lifecycle.
        drone_speed_publisher.destroy_node()
        rclpy.shutdown()


class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            '/my_cmd_vel',
            self.cmd_vel_callback,
            20)
        self.subscription  # prevent unused variable warning

    def cmd_vel_callback(self, msg):
        global cmd_vel_flu
        cmd_vel_flu = msg
        # print("Received cmd_vel:")
        # print(msg)

def start_CmdVelSubscriber_ros_node():
    if not rclpy.ok():
        rclpy.init()
    cmd_vel_subscriber = CmdVelSubscriber()
    rclpy.spin(cmd_vel_subscriber)
    cmd_vel_subscriber.destroy_node()
    rclpy.shutdown()

def publish_data():
    rate = rospy.Rate(20)  # 发布频率，可以根据需要进行调整
    # 发布GPS信息的ROS话题
    gps_pub = rospy.Publisher('/gps_data', NavSatFix, queue_size=10)
    # 发布角度信息的ROS话题
    imu_pub = rospy.Publisher('/imu_data', Imu, queue_size=10)
    while not rospy.is_shutdown():
        # 读取GPS信息
        gps_msg = NavSatFix()
        gps_msg.header.stamp = rospy.Time.now()
        gps_msg.latitude = vehicle.location.global_frame.lat
        gps_msg.longitude = vehicle.location.global_frame.lon
        gps_msg.altitude = vehicle.location.global_frame.alt
        gps_pub.publish(gps_msg)

        # 读取角度信息
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()

        # 直接使用欧拉角信息
        imu_msg.orientation = Quaternion(
            x=vehicle.attitude.roll,
            y=vehicle.attitude.pitch,
            z=vehicle.attitude.yaw,
            w=0.0  # 在这里w设置为0，因为Quaternion的构造需要四个参数，但通常情况下这个值是不需要的
        )
        imu_pub.publish(imu_msg)

        rate.sleep()

def key_event(vehicle):
    global mode
    global forward
    global leftward
    global upward
    global angular
    global TAKE_OFF_ALTITUDE
    key = getKey()
    if key == 'w' :
        forward = forward + LINEAR_STEP_SIZE
        print_msg()
        print("forward vel %.2f\t leftward vel %.2f\n upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

    elif key == 'x' :
        forward = forward - LINEAR_STEP_SIZE
        print_msg()
        print("forward vel %.2f\t leftward vel %.2f\n upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

    elif key == 'a' :

        leftward = leftward + LINEAR_STEP_SIZE
        print_msg()
        print("forward vel %.2f\t leftward vel %.2f\n upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

    elif key == 'd' :
        leftward = leftward - LINEAR_STEP_SIZE
        print_msg()
        print("forward vel %.2f\t leftward vel %.2f\n upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

    elif key == 'i' :
        upward = upward + LINEAR_STEP_SIZE
        print_msg()
        print("forward vel %.2f\t leftward vel %.2f\n upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

    elif key == ',' :
        upward = upward - LINEAR_STEP_SIZE
        print_msg()
        print("forward vel %.2f\t leftward vel %.2f\n upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

    elif key == 'j':
        angular = angular + ANG_VEL_STEP_SIZE
        print_msg()
        print("forward vel %.2f\t leftward vel %.2f\n upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

    elif key == 'l':
        angular = angular - ANG_VEL_STEP_SIZE
        print_msg()
        print("forward vel %.2f\t leftward vel %.2f\n upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

    elif key == 'r':
        vehicle.mode = VehicleMode("RTL")
        print('Returning home')
    elif key == 't':
        vehicle.armed = True
        print_msg()
        print('Arming!')
    elif key == 'y':
        vehicle.armed = False
        print_msg()
        print('Disarming')
    elif key == 'u':
        vehicle.mode = VehicleMode("STABILIZE")
        print_msg()
        print('STABILIZE')
        mode='STABILIZE'
        #print('Takeoff mode is disenabled now')
    elif key == 'b':
        vehicle.mode = VehicleMode("GUIDED")
        print_msg()
        print('GUIDED')
        mode='GUIDED'
    elif key == 'n':
        vehicle.mode = VehicleMode("LAND")
        print('Landing')
        mode='LAND'
    elif key == 'o':
        vehicle.mode = VehicleMode("GUIDED")
        print('move_base导航中')
        mode='MOVEBASE'
    elif key == 'p':
        vehicle.mode = VehicleMode("GUIDED")
        mode='TEST'
        msg = vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            100,    # param 1, yaw in degrees
            100,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            1, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        vehicle.send_mavlink(msg)
        vehicle.flush()
    elif key in ['k', 's']:
        forward   = 0.0
        leftward   = 0.0
        upward   = 0.0
        angular  = 0.0
        print_msg()
        print("forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))
        print('Hover')
        vehicle.mode = VehicleMode("GUIDED")
        mode='GUIDED'
        set_velocity_body(vehicle, forward, leftward, upward,angular)
    elif key == 'v':
        print_msg()
        vehicle.mode = VehicleMode("GUIDED")
        if not vehicle.armed: 
            print('please arm!请解锁！')
            return
        print("Taking Off!起飞！")
        vehicle.simple_takeoff(TAKE_OFF_ALTITUDE)
        mode='TAKEOFF'
    if (key == '\x03' ):
        if(vehicle.armed == False or connectUrl=='127.0.0.1:14550' or connectUrl=='10.10.10.20:14550'):
            return 2
        print('Please land and lock!') 

def mode_event(vehicle):
    global lastTime
    global mode
    global forward
    global leftward
    global upward
    global angular
    global TAKE_OFF_ALTITUDE
    global cmd_vel_flu
    if(mode=='TAKEOFF'):
        if(time.time()-lastTime>1):
            lastTime=time.time()
            v_alt = vehicle.location.global_relative_frame.alt
            print(">> Altitude = %.1f m"%v_alt)
            if v_alt >= TAKE_OFF_ALTITUDE * 0.95:
                mode='GUIDED'
                print("Target altitude reached")
    if(mode=='GUIDED'):
        if forward > MAX_LINEAR:
            forward = MAX_LINEAR
        elif forward < -MAX_LINEAR:
            forward = -MAX_LINEAR
        if leftward > MAX_LINEAR:
            leftward = MAX_LINEAR
        elif leftward < -MAX_LINEAR:
            leftward = -MAX_LINEAR
        if upward > MAX_LINEAR:
            upward = MAX_LINEAR
        elif upward < -MAX_LINEAR:
            upward = -MAX_LINEAR
        if angular > MAX_ANG_VEL:
            angular = MAX_ANG_VEL
        elif angular < -MAX_ANG_VEL:
            angular = - MAX_ANG_VEL
        set_velocity_body(vehicle, forward, leftward, upward,angular)
    if(mode=='MOVEBASE'):
        if(cmd_vel_flu!=None):
            # rospy.spin()
            set_velocity_body(vehicle, cmd_vel_flu.linear.x, cmd_vel_flu.linear.y, cmd_vel_flu.linear.z,cmd_vel_flu.angular.z)
            print_msg()
            print("forward vel %.2f\t leftward vel %.2f\n upward vel %.2f \t angular vel %.2f" % (cmd_vel_flu.linear.x, cmd_vel_flu.linear.y, cmd_vel_flu.linear.z,cmd_vel_flu.angular.z))
            # print("forward vel %.2f\t leftward vel %.2f\n upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))
        else:
            print('路径规划开了吗？')

count=1
if __name__=="__main__":
    # try:
        # connectUrl='127.0.0.1:14550'
        connectUrl='10.10.10.20:14550'
        # connectUrl='/dev/ttyUSB1'
        settings = termios.tcgetattr(sys.stdin)
        vehicle=tryConnect(connectUrl)
        print_msg()
        rclpy.init()
        try:
            cmd_vel_subscriber = CmdVelSubscriber()
            drone_speed_publisher = DroneSpeedPublisher()
            # 为每个节点创建独立线程
            subscriber_thread = threading.Thread(target=node_spin, args=(cmd_vel_subscriber,))
            publisher_thread = threading.Thread(target=node_spin, args=(drone_speed_publisher,))
            subscriber_thread.start()
            publisher_thread.start()
            subscriber_thread.join()
            publisher_thread.join()
        finally:
            rclpy.shutdown()
        while(1):
            # count=count+1
            # if(count%10==0):
            #     if(not check_connection('10.10.10.10','7777')):
            #         print('disconnect with pc!')
            #         vehicle.mode = VehicleMode("LAND")
            #         print('Landing')
            if(key_event(vehicle)==2):
                break
            mode_event(vehicle)
        vehicle.close()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    # except rospy.ROSInterruptException:
    #     pass
    # finally:
    #     # 关闭连接
    #     vehicle.close()






