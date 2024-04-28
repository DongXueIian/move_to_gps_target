from dronekit import connect, VehicleMode
import time
connectUrl='10.10.10.20:14550'

# 连接到无人机
vehicle = connect(connectUrl, wait_ready=True)

# 定义处理 EKF_STATUS_REPORT 消息的回调函数
def ekf_status_callback(self, name, message):
    if message.get_type() == 'EKF_STATUS_REPORT':
        print("EKF Status:")
        print("  Flags: {}".format(message.flags))
        print("  Velocity Variance: {}".format(message.velocity_variance))
        print("  Pos Horiz Variance: {}".format(message.pos_horiz_variance))
        print("  Pos Vert Variance: {}".format(message.pos_vert_variance))
        print("  Compass Variance: {}".format(message.compass_variance))
        print("  Terrain Alt Variance: {}".format(message.terrain_alt_variance))

# 添加回调函数监听所有 MAVLink 消息
vehicle.add_message_listener('*', ekf_status_callback)

# 保持脚本运行一段时间，以便可以接收消息
print("Listening for EKF status messages...")
time.sleep(30)

# 关闭连接
vehicle.remove_message_listener('*', ekf_status_callback)
vehicle.close()
print("Done.")
