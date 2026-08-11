#!/usr/bin/env python2
import rospy
import numpy as np
import socket
import threading
from sensor_msgs.msg import Image

TCP_PORT = 9999
conn = None
conn_lock = threading.Lock()


def send_frame(img_np):
    global conn
    with conn_lock:
        if conn is not None:
            try:
                img_bytes = img_np.tobytes()
                header = str(len(img_bytes)).encode("ascii").ljust(16)
                conn.sendall(header)
                conn.sendall(img_bytes)
            except Exception as e:
                print("client disconnected:", e)
                conn = None


def image_callback(msg):
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    h = msg.height
    w = msg.width
    img = raw.reshape(h, w, 4)[:, :, :3]
    send_frame(img)


def tcp_listener():
    global conn
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("127.0.0.1", TCP_PORT))
    sock.listen(1)
    print("TCP bridge server started 127.0.0.1:{}".format(TCP_PORT))
    while True:
        new_conn, addr = sock.accept()
        print("Py3 client connected from", addr)
        with conn_lock:
            conn = new_conn


if __name__ == "__main__":
    threading.Thread(target=tcp_listener).start()
    rospy.init_node("py2_tcp_bridge")
    image_topic = rospy.get_param('~img_topic', "/carla/ego_vehicle/rgb_main_front/image")
    rospy.Subscriber(image_topic, Image, image_callback, queue_size=1)
    rospy.spin()
