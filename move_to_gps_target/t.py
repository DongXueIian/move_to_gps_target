from sqlite3 import Time
import rclpy
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rclpy.node import Node

index = 0
# set_params = None

def get_params():
    global set_params, index
    
    # 创建设置参数请求
    req = SetParameters.Request()
    # 设置参数数据
    req.parameters = [Parameter(name="FollowPath.TwirlingCritic.enabled",value=ParameterValue(bool_value=True,type=ParameterType.PARAMETER_BOOL))]
    # 发送请求
    set_params.call_async(req)
    print('send')

class setNavParams(Node):
    def __init__(self):
        super().__init__('set_nav_params')
        self.set_params = self.create_client(SetParameters, "/controller_server/set_parameters")
        self.set_params.wait_for_service(1)
        self.create_timer(0.1, self.get_params)
    def get_params(self):
        # 创建设置参数请求
        req = SetParameters.Request()
        # 设置参数数据
        req.parameters = [Parameter(name="FollowPath.TwirlingCritic.enabled",value=ParameterValue(bool_value=True,type=ParameterType.PARAMETER_BOOL))]
        # 发送请求
        self.set_params.call_async(req)
        print('send')

def main():
    global set_params
    rclpy.init()
    set_nav_params=setNavParams()
    # node = rclpy.create_node("set_params_node")
    # # 创建设置参数服务客户端
    # set_params = node.create_client(SetParameters, "/controller_server/set_parameters")
    # # 等待服务
    # set_params.wait_for_service(1)
    # node.create_timer(0.1, get_params)
    rclpy.spin(set_nav_params)

if __name__ == "__main__":
    main()
    # while(True):
    #     Time.wait(2)
    #     get_params()
    
