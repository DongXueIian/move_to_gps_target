import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput.keyboard import Key, Listener
from std_msgs.msg import String

apmControllerNameSpace = '/apm_drone'

MAX_LINEAR = 10
MAX_ANG_VEL = 3.1415
LINEAR_STEP_SIZE = 0.1
ANG_VEL_STEP_SIZE = 3.1415/36
TAKE_OFF_ALTITUDE=1

target_forward  = 0.0
target_leftward  = 0.0
target_upward  = 0.0
target_angular = 0.0

current_forward=0.0
current_leftward=0.0
current_upward=0.0
current_angular_velocity_z=0.0

target_mode=''

debug=False

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
o   : nav2控制模式
CTRL-C 退出（只有在降落之后才能退出）
"""


class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_velocity_controller')
        global apmControllerNameSpace
        self.drone_mode=''
        self.publisher_cmd_vel = self.create_publisher(Twist,apmControllerNameSpace + '/high_permission_target_cmd_vel', 10)
        self.publisher_mode = self.create_publisher(String,apmControllerNameSpace + '/target_mode', 10)
        self.subscription_current_velocity = self.create_subscription(
            Twist, 
            apmControllerNameSpace + '/current_velocity',
            self.current_velocity_callback,
            10)
        self.subscription_mode = self.create_subscription(
            String,
            apmControllerNameSpace+'/current_mode',
            self.mode_callback,
            10)
        self.print_msg_10hz_timer=self.create_timer(0.1, self.print_msg_10hz)
        self.publish_msg_10hz_timer=self.create_timer(0.1, self.publish_msg_10hz)
        self.listener = Listener(on_press=self.on_press)
        self.listener.start()

    def mode_callback(self, msg):
        # 更改无人机的模式
        self.drone_mode = msg.data

    def print_msg_10hz(self):
        global debug,target_mode
        if debug:
            pass
        else:
            global current_forward,current_leftward,current_upward
            global target_forward,target_leftward,target_upward,target_angular
            print(msg)  
            print(f'飞机模式：{self.drone_mode} 键盘模式：{target_mode}')
            print(f'当前速度：{current_forward:.2f} {current_leftward:.2f} {current_upward:.2f} {current_angular_velocity_z:.2f}')
            print(f'键盘期望速度：{target_forward:.2f} {target_leftward:.2f} {target_upward:.2f} {target_angular:.2f}')

    def publish_msg_10hz(self):
        global debug,target_mode
        if debug==False and target_mode!='KEYBOARD_LOCK':
            global target_forward,target_leftward,target_upward,target_angular
            target_twist = Twist()
            target_twist.linear.x=target_forward
            target_twist.linear.y=target_leftward
            target_twist.linear.z=target_upward
            target_twist.angular.z=target_angular
            self.publisher_cmd_vel.publish(target_twist)
            if target_mode!='':
                mode_msg = String(data=target_mode)
                self.publisher_mode.publish(mode_msg)
                # target_mode=''
    def current_velocity_callback(self, msg):
        global current_forward,current_leftward,current_upward,current_angular_velocity_z
        current_forward=msg.linear.x
        current_leftward=msg.linear.y
        current_upward=msg.linear.z
        current_angular_velocity_z=msg.angular.z
        # self.get_logger().info('Current Velocity - Linear: x: %f, y: %f, z: %f' % (msg.linear.x, msg.linear.y, msg.linear.z))

    def on_press(self, key):
        global debug
        global target_forward,target_leftward,target_upward,target_angular
        global target_mode
        
        
        try:
            if debug:
                print(key.char)
                print('键盘期望速度：',target_forward,' ',target_leftward,' ',target_upward,' ',target_angular,' ')
            if key.char == 'w':  # forward
                target_forward = target_forward + LINEAR_STEP_SIZE
            elif key.char == 'x':  # backward
                target_forward = target_forward - LINEAR_STEP_SIZE
            elif key.char == 'a':  # left
                target_leftward = target_leftward + LINEAR_STEP_SIZE
            elif key.char == 'd':  # right
                target_leftward = target_leftward - LINEAR_STEP_SIZE
            elif key.char == 'i':
                target_upward = target_upward + LINEAR_STEP_SIZE
            elif key.char == ',':
                target_upward = target_upward - LINEAR_STEP_SIZE
            elif key.char == 'j':
                target_angular = target_angular + ANG_VEL_STEP_SIZE
            elif key.char == 'l':
                target_angular = target_angular - ANG_VEL_STEP_SIZE
            elif key.char == 'r':
                target_mode='RTL'
            elif key.char == 't':
                target_mode='ARM'
            elif key.char == 'y':
                target_mode='DISARM'
            elif key.char == 'u':
                target_mode='STABILIZE'
            elif key.char == 'b':
                target_mode='GUIDED'
            elif key.char == 'n':
                target_mode='LAND'
            elif key.char == 'o':
                target_mode='KEYBOARD_LOCK'
            elif key.char == 'p':
                target_mode='TEST'
            elif key.char in ['k', 's']:
                target_forward   = 0.0
                target_leftward   = 0.0
                target_upward   = 0.0
                target_angular  = 0.0
                target_mode='GUIDED'
            elif key.char == 'v':
                target_mode='TAKEOFF'
        except AttributeError:
            if key == Key.esc:
                # Stop listener
                return False


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardController()
    rclpy.spin(node)

    # Clean up
    node.listener.stop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
