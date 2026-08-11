# -*- coding: utf-8 -*-
import base64
import numpy as np
import cv2
import roslibpy

# ==================== 配置区 ====================
ROS_BRIDGE_HOST = "127.0.0.1"
ROS_BRIDGE_PORT = 9099
IMAGE_TOPIC = "/carla/ego_vehicle/rgb_front/image"
MSG_TYPE = "sensor_msgs/Image"
# ================================================

client = roslibpy.Ros(host=ROS_BRIDGE_HOST, port=ROS_BRIDGE_PORT)

def image_callback(msg):
    h = msg["height"]
    w = msg["width"]
    encoding = msg["encoding"]

    # rosbridge二进制数据经过base64编码，必须解码
    raw_bytes = base64.b64decode(msg["data"])
    arr = np.frombuffer(raw_bytes, dtype=np.uint8)
    img = None

    if encoding == "bgra8":
        img = arr.reshape(h, w, 4)[:, :, :3]  # 丢弃Alpha透明通道
    elif encoding == "rgb8":
        img = arr.reshape(h, w, 3)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif encoding == "bgr8":
        img = arr.reshape(h, w, 3)
    else:
        print(f"不支持图像编码: {encoding}")
        return

    cv2.imshow("Rosbridge Camera View", img)
    # 按下 q 退出程序
    if cv2.waitKey(1) & 0xFF == ord('q'):
        client.terminate()

def on_connected():
    print("✅ Python3 成功连接 rosbridge server")
    topic = roslibpy.Topic(client, IMAGE_TOPIC, MSG_TYPE)
    topic.subscribe(image_callback)

client.on_ready(on_connected)
client.run()

try:
    while client.is_connected:
        pass
except KeyboardInterrupt:
    pass
finally:
    client.terminate()
    cv2.destroyAllWindows()