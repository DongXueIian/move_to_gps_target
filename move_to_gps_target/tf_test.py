#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division  # 保证除法的一致性
import rospy
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Quaternion, TransformStamped
import tf
import tf.transformations as transformations
from std_msgs.msg import Header
from array import array
from pyproj import Proj, transform
import time

# 定义全局变量
gps_msg = None
imu_msg = None
has_receive = False

def choose_utm_zone(lon):
    center_lon = (int(lon) // 6) * 6 + 3  
    utm_zone = int((center_lon + 180) / 6) + 1

    if lon < 0:
        hemisphere = 'S' if utm_zone > 30 else 'N'
    else:
        hemisphere = 'N' if utm_zone > 30 else 'S'

    return utm_zone, hemisphere

def convert_to_utm(lon, lat):
    utm_zone, hemisphere = choose_utm_zone(lon)
    utm_proj_params = {"proj": "utm", "zone": utm_zone, "ellps": "WGS84", "datum": "WGS84", "units": "m", "no_defs": True}

    wgs84 = Proj(init='epsg:4326')
    utm = Proj(projparams=utm_proj_params)

    x, y = transform(wgs84, utm, lon, lat)
    return x, y, utm_proj_params

# GPS数据回调函数
def gps_callback(data):
    global gps_msg, has_receive
    gps_msg = data
    has_receive = True

# IMU数据回调函数
def imu_callback(data):
    global imu_msg
    imu_msg = data

if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node('tf_publisher_node')

    # 创建tf变换广播器
    tf_pub = tf.TransformBroadcaster()

    # 订阅GPS和IMU数据
    rospy.Subscriber('/gps_data', NavSatFix, gps_callback)
    rospy.Subscriber('/imu_data', Imu, imu_callback)

    # 设置循环频率
    rate = rospy.Rate(20)
    while not has_receive:
          time.sleep(1)
          print("waiting")
    # 如果已经接收到GPS数据，则进行tf变换的发布
    if has_receive:
        # 创建数组存储GPS和IMU数据
        x1, y1, utm_proj_params1 = convert_to_utm(gps_msg.longitude, gps_msg.latitude)
        
        map_tf = array('f', [x1,y1 , gps_msg.altitude,0])

        # 循环发布tf变换
        while not rospy.is_shutdown():
            # 创建TransformStamped消息
            t = TransformStamped()
            t.header = Header()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "map"
            t.child_frame_id = "base_link"
            x2, y2, utm_proj_params2 = convert_to_utm(gps_msg.longitude, gps_msg.latitude)
            # 计算相对于map坐标系的平移变换
            t.transform.translation.x = y2 - map_tf[1]
            t.transform.translation.y = map_tf[0]- x2
            t.transform.translation.z = gps_msg.altitude - map_tf[2]

            # 计算相对于map坐标系的旋转变换
            quat = transformations.quaternion_from_euler(0, 0, - imu_msg.orientation.z )
            t.transform.rotation = Quaternion(*quat)

            # 发布tf变换
            tf_pub.sendTransformMessage(t)

            # 控制循环频率
            rate.sleep()
