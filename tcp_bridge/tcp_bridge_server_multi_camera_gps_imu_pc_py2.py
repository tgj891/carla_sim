#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import socket
import threading
import struct
from sensor_msgs.msg import Image, PointCloud2, Imu, NavSatFix
import signal
import sys
import time

TCP_PORT = 9999
conn = None
conn_lock = threading.Lock()

# 消息类型定义
MSG_TYPE_IMAGE = 1   # 四路拼接图像
MSG_TYPE_GPS = 2     # GPS数据
MSG_TYPE_IMU = 3     # IMU数据
MSG_TYPE_POINT_CLOUD = 4  # 点云数据

# 存储四路图像缓存
img_cache = {
    "front": None,
    "back": None,
    "left": None,
    "right": None
}
cache_lock = threading.Lock()

# 传感器数据缓存
sensor_data = {
    "gps": None,
    "imu": None,
    "point_cloud": None
}
sensor_lock = threading.Lock()

# 发送统计
send_count = {
    "image": 0,
    "gps": 0,
    "imu": 0,
    "point_cloud": 0
}
exit_flag = False

def handle_sigint(signum, frame):
    global exit_flag
    print("\n✅ 捕获到 Ctrl+C，执行清理逻辑...")
    # =========在这里写释放资源代码=========
    # 关闭相机、销毁carla客户端、保存文件、释放句柄等
    # =====================================
    print("程序安全退出")
    exit_flag = True
    rospy.signal_shutdown("Receive Ctrl+C")

# 注册信号回调
signal.signal(signal.SIGINT, handle_sigint)


def send_data(msg_type, data_bytes):
    """发送带类型标识的数据"""
    global conn
    with conn_lock:
        if conn is not None:
            try:
                # 数据格式: [16字节总长度][4字节消息类型][数据体]
                # 总长度 = 4(类型) + len(数据)
                total_len = 4 + len(data_bytes)
                header = str(total_len).encode("ascii").ljust(16)
                type_bytes = struct.pack("<I", msg_type)  # 小端4字节
                conn.sendall(header)
                conn.sendall(type_bytes)
                conn.sendall(data_bytes)
            except Exception as e:
                print("client disconnected:", e)
                conn = None


def send_frame(img_np):
    """发送四路拼接图像"""
    img_bytes = img_np.tobytes()
    send_data(MSG_TYPE_IMAGE, img_bytes)
    send_count["image"] += 1


def send_gps_data(gps_data_bytes):
    """发送GPS数据"""
    send_data(MSG_TYPE_GPS, gps_data_bytes)
    send_count["gps"] += 1


def send_imu_data(imu_data_bytes):
    """发送IMU数据"""
    send_data(MSG_TYPE_IMU, imu_data_bytes)
    send_count["imu"] += 1


def send_point_cloud(pc_data_bytes):
    """发送点云数据"""
    send_data(MSG_TYPE_POINT_CLOUD, pc_data_bytes)
    send_count["point_cloud"] += 1


def merge_and_send():
    """四路图像全部就绪后拼接并发送"""
    with cache_lock:
        # 校验四路图像是否都存在
        if (img_cache["front"] is not None and
            img_cache["back"] is not None and
            img_cache["left"] is not None and
            img_cache["right"] is not None):

            front = img_cache["front"]
            back  = img_cache["back"]
            left  = img_cache["left"]
            right = img_cache["right"]

            # 2行2列拼接
            row_top = np.hstack([front, left])
            row_bot = np.hstack([right, back])
            merged = np.vstack([row_top, row_bot])

            send_frame(merged)

            # 清空缓存，等待下一帧完整四组图像
            img_cache["front"] = None
            img_cache["back"]  = None
            img_cache["left"]  = None
            img_cache["right"] = None


def callback_factory(key):
    """生成对应相机的回调函数"""
    def callback(msg):
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        h = msg.height
        w = msg.width
        img = raw.reshape(h, w, 4)[:, :, :3]  # BGRA -> BGR

        with cache_lock:
            img_cache[key] = img

        merge_and_send()
    return callback


def gps_callback(msg):
    """GPS数据回调 - 序列化并发送"""
    try:
        # GPS数据: [timestamp, latitude, longitude, altitude]
        gps_data = struct.pack("<4d",
                               msg.header.stamp.to_sec(),
                               msg.latitude,
                               msg.longitude,
                               msg.altitude)
        with sensor_lock:
            sensor_data["gps"] = {
                "timestamp": msg.header.stamp.to_sec(),
                "latitude": msg.latitude,
                "longitude": msg.longitude,
                "altitude": msg.altitude
            }
        send_gps_data(gps_data)
    except Exception as e:
        rospy.logwarn("GPS callback error: %s", e)


def imu_callback(msg):
    """IMU数据回调 - 序列化并发送"""
    try:
        # IMU数据: [timestamp, orientation(4), angular_velocity(3), linear_acceleration(3)]
        imu_data = struct.pack("<11d",
                               msg.header.stamp.to_sec(),
                               msg.orientation.x, msg.orientation.y,
                               msg.orientation.z, msg.orientation.w,
                               msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                               msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        with sensor_lock:
            sensor_data["imu"] = {
                "timestamp": msg.header.stamp.to_sec(),
                "orientation": (msg.orientation.x, msg.orientation.y,
                               msg.orientation.z, msg.orientation.w),
                "angular_velocity": (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z),
                "linear_acceleration": (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
            }
        send_imu_data(imu_data)
    except Exception as e:
        rospy.logwarn("IMU callback error: %s", e)


def point_cloud_callback(msg):
    """点云数据回调 - 序列化并发送"""
    try:
        # 将点云转换为numpy数组
        import ros_numpy
        pc_array = ros_numpy.point_cloud2.pointcloud2_to_array(msg)

        # 转换为bytes发送
        pc_bytes = pc_array.tobytes()

        # 发送格式: [timestamp(8B)][height(4B)][width(4B)][point_step(4B)][数据]
        header_bytes = struct.pack("<dIII",
                                   msg.header.stamp.to_sec(),
                                   msg.height,
                                   msg.width,
                                   msg.point_step)
        pc_data = header_bytes + pc_bytes

        with sensor_lock:
            sensor_data["point_cloud"] = {
                "timestamp": msg.header.stamp.to_sec(),
                "height": msg.height,
                "width": msg.width,
                "point_step": msg.point_step,
                "data": pc_array
            }
        send_point_cloud(pc_data)
    except ImportError:
        # 如果没有ros_numpy，使用简单的序列化
        try:
            raw_data = bytes(msg.data)
            header_bytes = struct.pack("<dIII",
                                       msg.header.stamp.to_sec(),
                                       msg.height,
                                       msg.width,
                                       msg.point_step)
            pc_data = header_bytes + raw_data

            with sensor_lock:
                sensor_data["point_cloud"] = {
                    "timestamp": msg.header.stamp.to_sec(),
                    "height": msg.height,
                    "width": msg.width,
                    "point_step": msg.point_step
                }
            send_point_cloud(pc_data)
        except Exception as e:
            rospy.logwarn("Point cloud callback error: %s", e)
    except Exception as e:
        rospy.logwarn("Point cloud callback error: %s", e)


def tcp_listener():
    global conn
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("127.0.0.1", TCP_PORT))
    sock.listen(1)
    print("TCP bridge server started 127.0.0.1:{}".format(TCP_PORT))
    while not exit_flag:
        new_conn, addr = sock.accept()
        print("Py3 client connected from", addr)
        with conn_lock:
            conn = new_conn


if __name__ == "__main__":
    threading.Thread(target=tcp_listener).start()
    rospy.init_node("py2_tcp_bridge")

    # 订阅四路相机话题
    # rospy.Subscriber("/carla/ego_vehicle/rgb_front/image", Image, callback_factory("front"), queue_size=1)
    rospy.Subscriber("/carla/ego_vehicle/rgb_main_front/image", Image, callback_factory("front"), queue_size=1)
    rospy.Subscriber("/carla/ego_vehicle/rgb_back/image",  Image, callback_factory("back"),  queue_size=1)
    rospy.Subscriber("/carla/ego_vehicle/rgb_left/image",  Image, callback_factory("left"),  queue_size=1)
    rospy.Subscriber("/carla/ego_vehicle/rgb_right/image", Image, callback_factory("right"), queue_size=1)

    # 订阅GPS话题
    rospy.Subscriber("/carla/ego_vehicle/gnss", NavSatFix, gps_callback, queue_size=1)

    # 订阅IMU话题
    rospy.Subscriber("/carla/ego_vehicle/imu", Imu, imu_callback, queue_size=1)

    # 订阅点云话题
    rospy.Subscriber("/carla/ego_vehicle/lidar", PointCloud2, point_cloud_callback, queue_size=1)

    print("Subscribed topics:")
    print("  - 4 camera topics (front, back, left, right)")
    print("  - GPS: /carla/ego_vehicle/gnss")
    print("  - IMU: /carla/ego_vehicle/imu")
    print("  - PointCloud: /carla/ego_vehicle/lidar")

    # 定期打印发送统计
    def print_stats(event):
        with sensor_lock:
            gps = sensor_data["gps"]
            imu = sensor_data["imu"]
            pc = sensor_data["point_cloud"]
        print("[Stats] img={}, gps={}, imu={}, pc={}".format(
            send_count["image"], send_count["gps"], send_count["imu"], send_count["point_cloud"]))
        if gps:
            print("  GPS: lat={:.6f}, lon={:.6f}, alt={:.2f}".format(
                gps["latitude"], gps["longitude"], gps["altitude"]))

    rospy.Timer(rospy.Duration(5.0), print_stats)

    rospy.spin()