import socket
import sys
import numpy as np
import cv2
from PIL import Image
import torch
from config import *

sys.path.append("/opt/carla_ws/sim_workspace")

from bev_surround.config import *
from transformers import SegformerImageProcessor, SegformerForSemanticSegmentation

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

calib_H_param = {}
calib_polygons_param = {
    "front": np.array([[0, 0], [600, 0], [346, 285], [254, 285]], dtype=np.int32),
    "back": np.array([[0, 800], [254, 515], [346, 515], [600, 800]], dtype=np.int32),
    "left": np.array([[0, 0], [254, 285], [254, 512], [0, 800]], dtype=np.int32),
    "right": np.array([[600, 0], [600, 800], [346, 515], [346, 285]], dtype=np.int32)
}
#read car
tesla_car = cv2.imread(f"../bev_surround/data/tesla_car.png")
car_width = int(CAR_WIDTH / RESOLUTION)
car_height = int(CAR_LENGTH / RESOLUTION)
car_x1 = int((BEV_WIDTH - car_width) / 2)
car_x2 = int((BEV_WIDTH + car_width) / 2)
car_y1 = int((BEV_HEIGHT - car_height) / 2)
car_y2 = int((BEV_HEIGHT + car_height) / 2)

# 加载预训练SegFormer (ADE20K 语义分割)
processor = SegformerImageProcessor.from_pretrained("./weight/segformer-b2-finetuned-ade-512-512")
model = SegformerForSemanticSegmentation.from_pretrained("./weight/segformer-b2-finetuned-ade-512-512").cuda()
model.eval()

def init_params():
    #read homography
    f_H = np.load(f"../bev_surround/data/front_homography.npy")
    calib_H_param["front"] = f_H
    b_H = np.load(f"../bev_surround/data/back_homography.npy")
    calib_H_param["back"] = b_H
    l_H = np.load(f"../bev_surround/data/left_homography.npy")
    calib_H_param["left"] = l_H
    r_H = np.load(f"../bev_surround/data/right_homography.npy")
    calib_H_param["right"] = r_H
    
def copy_polygon_roi_same_size(img_src, img_dst, polygon):
    """
    【两张图像尺寸完全相同】多边形区域复制
    :param img_src: 源图像 BGR
    :param img_dst: 目标图像 BGR
    :param polygon: 多边形顶点 np.array(N,2) int32, 闭合轮廓
    :return: 复制完成后的目标图像（新副本，不修改原输入）
    """
    # 保护：创建副本，避免修改原始图片
    dst_out = img_dst.copy()
    h, w = img_src.shape[:2]

    # 校验两张图尺寸一致
    assert img_src.shape[:2] == img_dst.shape[:2], "源图与目标图像宽高必须一致！"

    # 创建掩码
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.fillPoly(mask, [polygon.astype(np.int32)], 255)

    # 源图提取多边形区域
    roi_region = cv2.bitwise_and(img_src, img_src, mask=mask)

    # 将多边形区域覆盖写入目标图
    # mask==255 的位置，使用源图像像素；其余位置保持原图不变
    dst_out[mask > 0] = roi_region[mask > 0]
    return dst_out
 
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

def semantic_segmentation(image):
    mask, vis = predict_semantic_seg(image)
    blend = cv2.addWeighted(image, 1.0, vis, 0.4, 0)
    return blend

def main():
    resize_tesla_car = cv2.resize(tesla_car, (car_width, car_height))
    while True:
        try:
            sock = connect_socket()
            while True:
                header_buf = recv_all(sock, 16)
                data_len = int(header_buf.strip())

                buffer = recv_all(sock, data_len)

                img = np.frombuffer(buffer, dtype=np.uint8).reshape(IMG_HEIGHT, IMG_WIDTH, 3).copy()
                # 4路图像拼接
                # [ front ][ left ]
                # [ right ][ back ]
                bev_img = np.zeros((BEV_HEIGHT, BEV_WIDTH, 3), dtype=np.uint8)
                bev_img[car_y1:car_y2, car_x1:car_x2] = resize_tesla_car
                
                front_image = cv2_crop(img, 0, 0, image_width, image_height)
                front_image = semantic_segmentation(front_image)
                front_bev_img = cv2.warpPerspective(front_image, calib_H_param["front"], (BEV_WIDTH, BEV_HEIGHT))
                bev_img = copy_polygon_roi_same_size(front_bev_img, bev_img, calib_polygons_param["front"])
                
                back_image = cv2_crop(img, image_width, image_height, image_width * 2, image_height * 2)
                back_image = semantic_segmentation(back_image)
                back_bev_img = cv2.warpPerspective(back_image, calib_H_param["back"], (BEV_WIDTH, BEV_HEIGHT))
                bev_img = copy_polygon_roi_same_size(back_bev_img, bev_img, calib_polygons_param["back"])
                
                left_image = cv2_crop(img, image_width, 0, image_width * 2, image_height)
                left_image = semantic_segmentation(left_image)
                left_bev_img = cv2.warpPerspective(left_image, calib_H_param["left"], (BEV_WIDTH, BEV_HEIGHT))
                bev_img = copy_polygon_roi_same_size(left_bev_img, bev_img, calib_polygons_param["left"])
                
                right_image = cv2_crop(img, 0, image_height, image_width, image_height * 2)
                right_image = semantic_segmentation(right_image)
                right_bev_img = cv2.warpPerspective(right_image, calib_H_param["right"], (BEV_WIDTH, BEV_HEIGHT))
                bev_img = copy_polygon_roi_same_size(right_bev_img, bev_img, calib_polygons_param["right"])
                    
                cv2.imshow("Carla BEV", bev_img)
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
    init_params()
    main()