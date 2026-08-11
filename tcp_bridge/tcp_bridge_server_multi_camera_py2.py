#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import socket
import threading
from sensor_msgs.msg import Image
import signal
import sys

TCP_PORT = 9999
conn = None
conn_lock = threading.Lock()

# 存储四路图像缓存
img_cache = {
    "front": None,
    "back": None,
    "left": None,
    "right": None
}
cache_lock = threading.Lock()
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


def send_frame(img_np):
    global conn
    with conn_lock:
        if conn is not None:
            try:
                img_bytes = img_np.tobytes()
                # 16字节长度头
                header = str(len(img_bytes)).encode("ascii").ljust(16)
                conn.sendall(header)
                conn.sendall(img_bytes)
            except Exception as e:
                print("client disconnected:", e)
                conn = None


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
    rospy.Subscriber("/carla/ego_vehicle/rgb_front/image", Image, callback_factory("front"), queue_size=1)
    rospy.Subscriber("/carla/ego_vehicle/rgb_back/image",  Image, callback_factory("back"),  queue_size=1)
    rospy.Subscriber("/carla/ego_vehicle/rgb_left/image",  Image, callback_factory("left"),  queue_size=1)
    rospy.Subscriber("/carla/ego_vehicle/rgb_right/image", Image, callback_factory("right"), queue_size=1)

    print("Subscribed 4 camera topics")
    rospy.spin()