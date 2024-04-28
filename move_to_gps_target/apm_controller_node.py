import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dronekit import connect, VehicleMode
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import BatteryState, NavSatFix
import time
from rclpy.clock import ROSClock
from geometry_msgs.msg import Twist
from pymavlink import mavutil
import math

# connectUrl='127.0.0.1:14550'
connectUrl='10.10.10.20:14550'
# connectUrl='/dev/ttyUSB1'
apmControllernNameSpace='/apm_drone'

TAKE_OFF_ALTITUDE=1.0

class apmControllernNode(Node):
    def __init__(self):
        super().__init__('apm_controller_node')
        global connectUrl
        # self.ros2_mode=''
        self.setup_time=self.get_clock().now()
        self.last_call_loop_action_10hz=self.get_clock().now()
        super().__init__('apm_controller_node')
        # 连接到无人机
        self.vehicle = self.tryConnect(connectUrl)
        self.vehicle.mode = VehicleMode('GUIDED')

        self.gyro_x=0.0
        self.gyro_y=0.0
        self.gyro_z=0.0
        
        self.control_mode='GUIDED'
        # 在连接后添加消息监听器
        self.vehicle.add_message_listener('RAW_IMU', self.listen_raw_imu)

        self.action_count=0
        self.high_permission_velocity_call=False

        # 订阅来自ROS的主题
        self.subscription_mode = self.create_subscription(
            String,
            apmControllernNameSpace+'/target_mode',
            self.mode_callback,
            10)
        self.subscription_velocity = self.create_subscription(
            Twist,
            # '/target_velocity',
            apmControllernNameSpace+'/target_cmd_vel',
            self.velocity_callback,
            10)
        self.subscription_velocity = self.create_subscription(
            Twist,
            # '/target_velocity',
            apmControllernNameSpace+'/high_permission_target_cmd_vel',
            self.high_permission_velocity_callback,
            10)
        # 发布到ROS的主题
        self.publisher_current_velocity = self.create_publisher(
            Twist,
            apmControllernNameSpace+'/current_velocity',
            10)
        self.publisher_current_mode_state = self.create_publisher(
            String,
            apmControllernNameSpace+'/current_mode_state',
            10)
        self.publisher_current_mode = self.create_publisher(
            String,
            apmControllernNameSpace+'/current_mode',
            10)
        self.publisher_current_attitude = self.create_publisher(
            String,
            apmControllernNameSpace+'/current_attitude',
            10)
        self.publisher_gps = self.create_publisher(
            NavSatFix,
            apmControllernNameSpace+'/current_GPS',
            10)
        self.publisher_battery = self.create_publisher(
            BatteryState,
            apmControllernNameSpace+'/current_battery',
            10)
        self.publisher_attitude = self.create_publisher(
            String,
            apmControllernNameSpace+'/current_attitude',
            10)

        # 定时发布无人机状态
        self.update_state_10hz_timer = self.create_timer(0.1, self.update_state_10hz)
        self.update_state_5hz_timer = self.create_timer(0.2, self.update_state_5hz)
        # 循环动作
        self.loop_action_10hz_timer= self.create_timer(0.1, self.loop_action_10hz)

    def tryConnect(self,ipAddress):
        # print('Connecting '+ipAddress)
        self.get_logger().info('Connecting '+ipAddress)
        vehicle = connect(ipAddress, wait_ready=True, baud=921600)
        # print('successfully connect to '+ipAddress)
        self.get_logger().info('successfully connect to '+ipAddress)
        lastTime=time.time()
        while not vehicle.is_armable:
            if(time.time()-lastTime>1):
                # print("waiting to be armable")
                self.get_logger().info("waiting to be armable")
                lastTime=time.time()
        self.get_logger().info("get ready to be arm")
        return vehicle

    def monitor_loop_time(self, last_time, expected_interval):
        current_time = self.get_clock().now()
        # print(current_time)
        if(last_time.nanoseconds-self.setup_time.nanoseconds<1000000000):
             return current_time
        elapsed = current_time - last_time
        elapsed_sec = elapsed.nanoseconds
        
        # if elapsed_sec < expected_interval:
        #     self.get_logger().warn(f'Loop running too fast: {elapsed_sec} seconds, expected {expected_interval} seconds')
        if elapsed_sec > expected_interval*1000000000*1.1:
            self.get_logger().warn(f'Loop running too slow: {elapsed_sec} nanoseconds, expected {expected_interval} seconds')

        return current_time

    #定义发送mavlink速度命令的功能
    def set_velocity_body(self, vx, vy, vz,vr):
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
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,
                0, 0,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                1479, #-- BITMASK -> Consider only the velocities
                0, 0, 0,        #-- POSITION
                vx, vy, vz,     #-- VELOCITY
                0, 0, 0,        #-- ACCELERATIONS
                0, vr)
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def mode_callback(self, msg):
        if self.control_mode!=msg.data:
            self.control_mode=msg.data
            self.get_logger().info(f'apm controller change mode to: {self.control_mode}')
            global TAKE_OFF_ALTITUDE
            # 直接更改无人机的模式
            if msg.data in ['RTL','STABILIZE','GUIDED','LAND']:
                self.vehicle.mode = VehicleMode(msg.data)
            elif msg.data=='ARM':
                self.vehicle.armed = True
            elif msg.data=='DISARM':
                self.vehicle.armed = False
            elif msg.data=='TAKEOFF':
                self.vehicle.mode = VehicleMode("GUIDED")
                if not self.vehicle.armed: 
                    self.get_logger().warn('can not take off , please arm!')
                    self.control_mode='GUIDED'
                else:
                    self.vehicle.simple_takeoff(TAKE_OFF_ALTITUDE)
    def velocity_callback(self, msg):
        # 设置无人机的目标速度
        if self.control_mode=='GUIDED' and self.vehicle.armed and not self.high_permission_velocity_call:
            self.set_velocity_body(msg.linear.x, msg.linear.y, msg.linear.z,msg.angular.z)
    
    def high_permission_velocity_callback(self, msg):
        # 设置无人机的目标速度
        if self.control_mode=='GUIDED' and self.vehicle.armed:
            self.set_velocity_body(msg.linear.x, msg.linear.y, msg.linear.z,msg.angular.z)
            self.high_permission_velocity_call=True

    def update_state_5hz(self):
        pass
    def loop_action_10hz(self):
        self.last_call_loop_action_10hz = self.monitor_loop_time(self.last_call_loop_action_10hz, 0.1)
        self.action_count=self.action_count+1
        if self.high_permission_velocity_call and self.action_count % 10 == 0:
            self.high_permission_velocity_call=False

    def update_state_10hz(self):
        # 发布无人机的当前飞行模式
        mode_msg = String(data=str(self.control_mode))
        self.publisher_current_mode.publish(mode_msg)

        # 发布无人机的当前状态
        mode_state_msg = String(data="armed" if self.vehicle.armed else "disarmed")
        self.publisher_current_mode_state.publish(mode_state_msg)

        # 发布无人机的当前速度
        # 使用当前的偏航角来旋转速度向量
        # 假设获取速度和姿态
        current_velocity = self.vehicle.velocity  # [0.04, -1.16, 0.0]
        current_attitude = self.vehicle.attitude  # -1.7527387142181396
        # print(self.vehicle.velocity,' ',self.vehicle.attitude.yaw)
        yaw = -current_attitude.yaw
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        velocity_msg = Twist()
        velocity_msg.linear.x = current_velocity[0] * cos_yaw - current_velocity[1] * sin_yaw
        velocity_msg.linear.y = -current_velocity[0] * sin_yaw -current_velocity[1] * cos_yaw
        velocity_msg.linear.z = current_velocity[2]
        velocity_msg.angular.x=self.gyro_x
        velocity_msg.angular.y=self.gyro_y
        velocity_msg.angular.z=self.gyro_z
        # print(velocity_msg.linear)
        self.publisher_current_velocity.publish(velocity_msg)

        # 发布无人机的当前姿态（Roll, Pitch, Yaw）
        attitude_msg = String(data=f"Roll: {self.vehicle.attitude.roll}, Pitch: {self.vehicle.attitude.pitch}, Yaw: {self.vehicle.attitude.yaw}")
        self.publisher_current_attitude.publish(attitude_msg)

        # 发布无人机的GPS数据
        gps_msg = NavSatFix()
        gps_msg.latitude = self.vehicle.location.global_frame.lat
        gps_msg.longitude = self.vehicle.location.global_frame.lon
        gps_msg.altitude = self.vehicle.location.global_frame.alt
        self.publisher_gps.publish(gps_msg)

        # 发布无人机的电池状态
        battery_msg = BatteryState()
        battery_msg.voltage = self.vehicle.battery.voltage
        battery_msg.current = 0.0
        battery_msg.percentage =  100.0  # Assuming 'level' is given as a percentage
        self.publisher_battery.publish(battery_msg)
        # print(self.vehicle.battery)

    # 定义一个回调函数来处理 RAW_IMU 消息
    def listen_raw_imu(self, vehicle, name, message):
        if message.get_type() == 'RAW_IMU':
            self.gyro_x = message.xgyro/1000
            self.gyro_y = message.ygyro/1000
            self.gyro_z = -message.zgyro/1000
            # print(f"Gyro X: {gyro_x}, Gyro Y: {gyro_y}, Gyro Z: {gyro_z}")



    def destroy_node(self):
        # 添加您希望在节点销毁时执行的任务
        self.get_logger().info('Shutting down: closing vehicle connection and cleaning up resources')
        if self.vehicle is not None:
            self.vehicle.close() 
        # 这里可以添加其他清理资源的代码
        super().destroy_node()
def main(args=None):
    rclpy.init(args=args)
    node = apmControllernNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
