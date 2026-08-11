import socket
import numpy as np
import cv2
from ultralytics import YOLO

TCP_PORT = 9999
HOST = "127.0.0.1"
IMG_HEIGHT = 600
IMG_WIDTH = 800
YOLO_MODEL_PATH = "/opt/carla_ws/sim_workspace/image_detect/weights/yolov8s.pt"
print("Loading YOLO model...")
model = YOLO(YOLO_MODEL_PATH)
# 获取类别名称（COCO 80类）
class_names = model.names
print(f"✅ YOLO model loaded, {len(class_names)} classes")

def recv_all(sock, length):
    buffer = b""
    while len(buffer) < length:
        chunk = sock.recv(length - len(buffer))
        if not chunk:
            raise ConnectionError("Server disconnected")
        buffer += chunk
    return buffer

def draw_detections(img, boxes, confs, cls_ids, class_names):
    """
    绘制检测框 + 类别标签 + 置信度
    """
    for box, conf, cls_id in zip(boxes, confs, cls_ids):
        x1, y1, x2, y2 = map(int, box)
        cls_name = class_names[cls_id]
        label = f"{cls_name} {conf:.2f}"
        # 框颜色（绿色）
        color = (0, 255, 0)
        # 画框
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        # 文字背景（提升可读性）
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
        cv2.rectangle(img, (x1, y1 - th - 4), (x1 + tw, y1), color, -1)
        # 写文字
        cv2.putText(img, label, (x1, y1 - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    return img

def connect_socket():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((HOST, TCP_PORT))
    print("Connected to server at {}:{}".format(HOST, TCP_PORT))
    return sock


def main():
    while True:
        try:
            sock = connect_socket()
            while True:
                header_buf = recv_all(sock, 16)
                data_len = int(header_buf.strip())

                buffer = recv_all(sock, data_len)

                img = np.frombuffer(buffer, dtype=np.uint8).reshape(IMG_HEIGHT, IMG_WIDTH, 3)
                img = img.copy()
                # YOLO推理，关闭打印减少阻塞
                results = model(
                    img,
                    conf=0.25,       # 置信度阈值
                    iou=0.45,        # NMS IoU阈值
                    agnostic_nms=False,
                    max_det=300,
                    verbose=False
                )

                # 3. 提取NMS后的结果
                for res in results:
                    boxes = res.boxes.xyxy.cpu().numpy()    # 框坐标
                    confs = res.boxes.conf.cpu().numpy()    # 置信度
                    cls_ids = res.boxes.cls.cpu().numpy().astype(int)  # 类别ID
                    # 绘制
                    img = draw_detections(img, boxes, confs, cls_ids, class_names)

                cv2.imshow("Carla RGB", img)
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
