import socket
import numpy as np
import cv2
from ultralytics import YOLO
import argparse

TCP_PORT = 9999
HOST = "127.0.0.1"
IMG_HEIGHT = 480
IMG_WIDTH = 640
fx = 184.7520861406803
fy = 184.7520861406803
cx = IMG_WIDTH / 2.0  # 主点x
cy = IMG_HEIGHT / 2.0  # 主点y
camera_height = 1.0  # 相机离地高度（米）

YOLO_MODEL_PATH = "/opt/carla_ws/sim_workspace/image_detect/weights/yolov8s.pt"
print("Loading YOLO model...")
model = YOLO(YOLO_MODEL_PATH)
# 获取类别名称（COCO 80类）
class_names = model.names
print(f"✅ YOLO model loaded, {len(class_names)} classes")
print("Class names:", class_names)

def recv_all(sock, length):
    buffer = b""
    while len(buffer) < length:
        chunk = sock.recv(length - len(buffer))
        if not chunk:
            raise ConnectionError("Server disconnected")
        buffer += chunk
    return buffer

def draw_detections(img, boxes, confs, cls_ids, class_names, mode):
    """
    绘制检测框 + 类别标签 + 置信度
    """
    for box, conf, cls_id in zip(boxes, confs, cls_ids):
        x1, y1, x2, y2 = map(int, box)
        cls_name = class_names[cls_id]
        if y2 < cy:
            continue
        # 框颜色（绿色）
        color = (0, 255, 0)
        if mode == 1:
            # 地平面测距
            distance = ground_plane_distance([x1, y1, x2, y2])
            # print(f"Ground Plane Distance: {distance:.2f} m")
        elif mode == 2:
            # 点云融合图像方法
            #distance = pointcloud_fusion_distance([x1, y1, x2, y2], pointcloud)
            #print(f"Pointcloud Fusion Distance: {distance:.2f} m")
            pass
        
        elif mode == 3:
            # 迭代测距
            distance = iterative_projection_distance([x1, y1, x2, y2], cls_id)
            print(f"Iterative Distance: {distance:.2f} m")
            
        elif mode == 4:
            # 单目深度估计
            # distance = monocular_depth_distance([x1, y1, x2, y2], depth_map)
            # print(f"Monocular Depth Distance: {distance:.2f} m")
            pass
        
        label = f"{cls_name} {conf:.2f} {distance:.2f}m"
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

def ground_plane_distance(box):
    """
    方法1：地平面假设方法
    原理：假设物体底部接触地面，利用相机高度和底部像素位置
    通过透视投影反推距离
    参数：
        box: [x1, y1, x2, y2] 检测框坐标
    返回：估计距离（米）
    """
    _, _, _, y2 = box
    # 物体底部的y坐标（像素）
    bottom_y = y2
    # 像素偏离主点的距离
    dy = bottom_y - cy
    if abs(dy) < 1e-6:
        return float('inf')
    # 由透视投影公式：Z = fy * H / (bottom_y - cy)
    # 其中H为相机高度
    distance = fy * camera_height / abs(dy)
    return distance


def iterative_projection_distance(box, class_id=0, max_iter=50):
    """
    方法2：迭代投影方法
    原理：假设物体为标准尺寸，迭代调整距离使得投影框与检测框匹配
    参数：
        box: [x1, y1, x2, y2] 检测框坐标
        class_id: 类别ID（用于获取标准尺寸）
        max_iter: 最大迭代次数
    返回：估计距离（米）
    """
    # 各类别的平均物理尺寸（宽, 高, 深），单位：米
    class_sizes = {
        0: (0.5, 1.5, 0.5),   # person
        2: (1.8, 1.5, 4.5),   # car
        3: (1.5, 1.2, 3.5),   # motorcycle
        5: (2.0, 1.8, 5.0),   # bus
        7: (2.5, 1.2, 5.0),   # truck
        1: (1.0, 1.2, 0.8),   # bicycle
    }
    if class_id not in class_sizes:
        class_id = 0
    obj_w, obj_h, obj_d = class_sizes[class_id]
    x1, y1, x2, y2 = box
    # 检测框的像素宽度和高度
    box_w = x2 - x1
    box_h = y2 - y1
    # 初始距离估计
    distance = 10.0
    for _ in range(max_iter):
        # 根据当前距离计算投影后的框宽度
        proj_w = obj_w * fx / distance
        proj_h = obj_h * fy / distance
        # 计算投影框与检测框的误差
        error_w = proj_w - box_w
        error_h = proj_h - box_h
        error = error_w + error_h
        if abs(error) < 1.0:
            break
        # 调整距离：误差越大，调整步长越大
        ratio_w = obj_w * fx / (box_w + 1e-6)
        ratio_h = obj_h * fy / (box_h + 1e-6)
        distance = (ratio_w + ratio_h) / 2.0
        if distance < 0.5:
            distance = 0.5
    return distance


def pointcloud_fusion_distance(box, pointcloud):
    """
    方法3：点云融合图像方法
    原理：将LiDAR点云投影到图像平面，在检测框内取平均深度
    参数：
        box: [x1, y1, x2, y2] 检测框坐标
        pointcloud: Nx3 点云数据 (x, y, z)
    返回：估计距离（米）
    """
    x1, y1, x2, y2 = box
    # 点云投影到图像平面
    # 只保留前方点（z > 0）
    valid_mask = pointcloud[:, 2] > 0
    valid_pts = pointcloud[valid_mask]
    if len(valid_pts) == 0:
        return float('inf')
    # 投影计算像素坐标
    pts_x = (valid_pts[:, 0] * fx / valid_pts[:, 2]) + cx
    pts_y = (valid_pts[:, 1] * fy / valid_pts[:, 2]) + cy
    # 查找落入检测框内的点
    in_box_mask = (pts_x >= x1) & (pts_x <= x2) & (pts_y >= y1) & (pts_y <= y2)
    in_box_pts_z = valid_pts[in_box_mask, 2]
    if len(in_box_pts_z) == 0:
        return float('inf')
    # 使用中位数距离更鲁棒
    distance = np.median(in_box_pts_z)
    return distance


def monocular_depth_distance(box, depth_map):
    """
    方法4：单目深度估计方法
    原理：利用深度估计模型得到每个像素的深度，在检测框区域取平均
    参数：
        box: [x1, y1, x2, y2] 检测框坐标
        depth_map: HxW 深度图（米）
    返回：估计距离（米）
    """
    x1, y1, x2, y2 = box
    # 边界检查
    x1 = max(0, int(x1))
    y1 = max(0, int(y1))
    x2 = min(IMG_WIDTH - 1, int(x2))
    y2 = min(IMG_HEIGHT - 1, int(y2))
    # 提取检测框区域的深度值
    roi_depth = depth_map[y1:y2, x1:x2]
    if roi_depth.size == 0:
        return float('inf')
    # 使用10%分位数，避免异常值影响
    distance = np.percentile(roi_depth, 10)
    return distance

def main(mode):
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
                    img = draw_detections(img, boxes, confs, cls_ids, class_names, mode)

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
    parser = argparse.ArgumentParser(description="BEV环视标定板生成工具 (CARLA 0.9.13)")
    parser.add_argument('--mode', type=int, default=-1, 
                        help='指定模式，1-地平面测距，2-点云融合图像方法，3-迭代测距，4-单目深度估计')
    
    args = parser.parse_args()
    main(args.mode)
