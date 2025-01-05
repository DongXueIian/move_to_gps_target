import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rcl_interfaces.msg import ParameterType  # 导入rcl_interfaces.msg中的ParameterType消息
from rcl_interfaces.srv import SetParameters  # 导入rcl_interfaces.srv中的SetParameters服务
from rclpy.parameter import Parameter
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import threading
from rclpy.executors import SingleThreadedExecutor
range_degrees = 12
isTwirlingCriticEnabled=False
lastIsTwirlingCriticEnabled=False
apmControllernNameSpace='/apm_drone'
# class setNavParams(Node):
#     def __init__(self):
#         super().__init__('set_nav_params')
#         self.set_params = self.create_client(SetParameters, "/controller_server/set_parameters")
#         self.set_params.wait_for_service(1)
#         self.create_timer(0.1, self.get_params)
#     def get_params(self):
#         global isTwirlingCriticEnabled,lastIsTwirlingCriticEnabled
#         if lastIsTwirlingCriticEnabled!=isTwirlingCriticEnabled:
#             lastIsTwirlingCriticEnabled=isTwirlingCriticEnabled
#             # 创建设置参数请求
#             req = SetParameters.Request()
#             # 设置参数数据
#             req.parameters = [Parameter(name="FollowPath.TwirlingCritic.enabled",value=ParameterValue(bool_value=isTwirlingCriticEnabled,type=ParameterType.PARAMETER_BOOL))]
#             # 发送请求
#             self.set_params.call_async(req)
#             print('send---'+str(isTwirlingCriticEnabled))
class CmdVelModifier(Node):
    def __init__(self):
        super().__init__('cmd_vel_modifier')
        self.current_cmd_vel_data=Twist()
        self.current_cmd_vel_data.linear.x=0.0
        self.subscription_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            20
        )
        self.subscription_cuuent_cmd_vel = self.create_subscription(
            Twist,
            apmControllernNameSpace+'/current_velocity',
            self.current_cmd_vel_callback,
            20
        )
        self.subscription_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            20
        )

        self.publisher_ = self.create_publisher(Twist, apmControllernNameSpace+'/target_cmd_vel', 20)
        
        self.cmd_vel_data = Twist()
        self.laser_data = LaserScan()
        self.timer = self.create_timer(1/20, self.timer_callback)

        self.set_params = self.create_client(SetParameters, "/controller_server/set_parameters")
        self.set_params.wait_for_service(1)
        # self.create_timer(0.1, self.get_params)

    def get_params(self):
        global isTwirlingCriticEnabled,lastIsTwirlingCriticEnabled
        if lastIsTwirlingCriticEnabled!=isTwirlingCriticEnabled:
            lastIsTwirlingCriticEnabled=isTwirlingCriticEnabled
            # 创建设置参数请求
            req = SetParameters.Request()
            # 设置参数数据
            req.parameters = [Parameter(name="FollowPath.TwirlingCritic.enabled",value=ParameterValue(bool_value=isTwirlingCriticEnabled,type=ParameterType.PARAMETER_BOOL))]
            # 发送请求
            self.set_params.call_async(req)
            self.get_logger().info('send---'+str(isTwirlingCriticEnabled))


    def current_cmd_vel_callback(self,msg):
        # print(msg)
        self.current_cmd_vel_data = msg

    def cmd_vel_callback(self, msg):
        self.cmd_vel_data = msg

    def scan_callback(self, msg):
        self.laser_data = msg
    def timer_callback(self):
        global range_degrees,isTwirlingCriticEnabled
        if self.laser_data:
            # 计算雷达数据的正前方48度范围
            num_points = len(self.laser_data.ranges)
            if(num_points > 0):

                # print(self.laser_data)
                middle_index = num_points // 2
                

                # print(num_points)
                # print(self.laser_data.angle_increment)
                # range_index = int((range_degrees / 2) * num_points / self.laser_data.angle_increment / 360)
                range_index = int(num_points*range_degrees/((self.laser_data.angle_max-self.laser_data.angle_min)/6.283185482*360))
                # print(range_index)
                forward_ranges = self.laser_data.ranges[middle_index - range_index:middle_index + range_index]

                # 查找正前方最近的障碍物距离
                min_distance = min((distance for distance in forward_ranges if distance > 0), default=float('10'))
                if self.current_cmd_vel_data.linear.x>1.5:
                    isTwirlingCriticEnabled=True
                    range_degrees=5
                else:
                    rclpy.parameter.Parameter('/controller_server/FollowPath.TwirlingCritic.enabled', rclpy.Parameter.Type.BOOL, False) 
                    isTwirlingCriticEnabled=False
                    range_degrees=12
                # 根据距离设置速度限制
                if min_distance < 1.5:
                    max_speed = 0.6
                elif min_distance < 2.5:
                    max_speed = 1.0
                elif min_distance < 4.0:
                    max_speed = 1.5
                elif min_distance < 6.0:
                    max_speed = 2.0
                elif min_distance < 8.0:
                    max_speed = 2.5
                elif min_distance < 10.0:
                    max_speed = 3.0
                # elif min_distance < 13:
                #     max_speed = 3.0
                else:
                    max_speed = 4.0  # 如果没有障碍物或障碍物距离足够远，则使用原速度
                # max_speed=max_speed*10
                # 创建新的速度消息并发布
                # print(range_degrees)
                new_cmd_vel = self.cmd_vel_data
                new_cmd_vel.linear.x = min(self.cmd_vel_data.linear.x, max_speed)
                self.publisher_.publish(new_cmd_vel)
            else:
                self.publisher_.publish(self.cmd_vel_data)
def node_spin(node):
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
def main():
    # rclpy.init(args=args)
    # try:
    #     cmd_vel_modifier = CmdVelModifier()
    #     # set_nav_params = setNavParams()
    #     # 为每个节点创建独立线程
    #     cmd_vel_modifier_thread = threading.Thread(target=node_spin, args=(cmd_vel_modifier,))
    #     # set_nav_params_thread = threading.Thread(target=node_spin, args=(set_nav_params,))
    #     cmd_vel_modifier_thread.start()
    #     # set_nav_params_thread.start()
    #     cmd_vel_modifier_thread.join()
    #     # set_nav_params_thread.join()
    # finally:
    #     rclpy.shutdown()

    # 初始化rclpy库
    rclpy.init()
    cmd_vel_modifier = CmdVelModifier()
    # 让ROS 2节点运行起来，监听和处理回调函数，直到节点被显式地关闭
    rclpy.spin(cmd_vel_modifier)
    # 销毁节点，进行清理
    cmd_vel_modifier.destroy_node()
    # 关闭rclpy库，释放资源
    rclpy.shutdown()
if __name__ == '__main__':
    main()
