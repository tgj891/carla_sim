import socket
import numpy as np
import cv2
from PIL import Image
import torch
from config import *

from transformers import SegformerImageProcessor, SegformerForSemanticSegmentation

FOURE_FUSION = True
TCP_PORT = 9999
HOST = "127.0.0.1"
# 四合一的图像尺寸
IMG_HEIGHT = 960
IMG_WIDTH = 1280

# 相机内参
image_height = 480
image_width = 640
cx = image_width / 2
cy = image_height / 2
fx = 184.7520861406803
fy = 184.7520861406803
#

# 加载预训练SegFormer (ADE20K 语义分割)
processor = SegformerImageProcessor.from_pretrained("./weight/segformer-b2-finetuned-ade-512-512")
model = SegformerForSemanticSegmentation.from_pretrained("./weight/segformer-b2-finetuned-ade-512-512").cuda()
model.eval()

def predict_semantic_seg(rgb_frame):
    """
    rgb_frame: BGR numpy数组(OpenCV读取) H,W,3
    return: seg_mask(类别索引图), seg_vis(彩色可视化图)
    """
    # 转RGB
    image_rgb = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2RGB)
    inputs = processor(images=image_rgb, return_tensors="pt").to("cuda")

    with torch.no_grad():
        outputs = model(**inputs)

    # 核心：argmax 获取类别索引 0~36
    logits = outputs.logits
    pred_mask = torch.argmax(logits, dim=1).squeeze().cpu().numpy()

    # 截断兜底（防止异常数值，杜绝索引越界）
    pred_mask = np.clip(pred_mask, 0, ADE_PALETTE.shape[0]-1)

    h,w = rgb_frame.shape[:2]
    pred_mask = cv2.resize(pred_mask.astype(np.float32), (w,h), interpolation=cv2.INTER_NEAREST)
    pred_mask = pred_mask.astype(np.int32)

    # 上色可视化
    seg_vis = ADE_PALETTE[pred_mask]
    seg_vis = seg_vis.astype(np.uint8)
    
    return pred_mask, seg_vis

def recv_all(sock, length):
    buffer = b""
    while len(buffer) < length:
        chunk = sock.recv(length - len(buffer))
        if not chunk:
            raise ConnectionError("Server disconnected")
        buffer += chunk
    return buffer


def connect_socket():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((HOST, TCP_PORT))
    print("Connected to server at {}:{}".format(HOST, TCP_PORT))
    return sock

def cv2_crop(img, x1, y1, x2, y2):
    """
    OpenCV图像裁剪
    :param img: cv2读取的图像
    :param x1,y1: 左上角坐标
    :param x2,y2: 右下角坐标
    :return: 裁剪后图像
    """
    return img[y1:y2, x1:x2]

def main():
    while True:
        try:
            sock = connect_socket()
            while True:
                header_buf = recv_all(sock, 16)
                data_len = int(header_buf.strip())

                buffer = recv_all(sock, data_len)

                img = np.frombuffer(buffer, dtype=np.uint8).reshape(IMG_HEIGHT, IMG_WIDTH, 3).copy()
                if FOURE_FUSION:
                    img = np.frombuffer(buffer, dtype=np.uint8).reshape(IMG_HEIGHT, IMG_WIDTH, 3).copy()
                    front_image = img
                else:
                    img = np.frombuffer(buffer, dtype=np.uint8).reshape(image_height, image_width, 3).copy()
                    front_image = img
                mask, vis = predict_semantic_seg(front_image)
                blend = cv2.addWeighted(front_image, 1.0, vis, 0.4, 0)
                    
                cv2.imshow("Carla RGB", blend)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    cv2.destroyAllWindows()
                    sock.close()
                    return

        except ConnectionError:
            print("Connection lost, reconnecting...")
            cv2.destroyAllWindows()
            import time
            time.sleep(1)
        except Exception as e:
            print("Error:", e)
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()