import socket
import numpy as np
import cv2
from gridboard_manualdetect_py3 import *
from config import *

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

def main(args):
    while True:
        try:
            sock = connect_socket()
            while True:
                header_buf = recv_all(sock, 16)
                data_len = int(header_buf.strip())

                buffer = recv_all(sock, data_len)

                img = np.frombuffer(buffer, dtype=np.uint8).reshape(IMG_HEIGHT, IMG_WIDTH, 3)
                # 4路图像拼接
                # [ front ][ left ]
                # [ right ][ back ]
                if args.name == "front":
                    calib_image = cv2_crop(img, 0, 0, image_width, image_height)
                elif args.name == "back":
                    calib_image = cv2_crop(img, image_width, image_height, image_width * 2, image_height * 2)
                elif args.name == "left":
                    calib_image = cv2_crop(img, image_width, 0, image_width * 2, image_height)
                elif args.name == "right":
                    calib_image = cv2_crop(img, 0, image_height, image_width, image_height * 2)
                else:
                    print("Error: 未支持的相机名称")
                    return
                # 棋盘格检测
                picker = ManualCornerPicker(calib_image)
                corners = picker.run()
                if corners:
                    corners_np = np.array(corners, dtype=np.float32).reshape(-1, 1, 2)
                    world_corners = build_chessboard_world_points(args.name)
                    world_corners_np = np.array(world_corners, dtype=np.float32).reshape(-1, 1, 3)
                    print("\n" + "=" * 60)
                    print("角点坐标列表 (图像像素坐标):")
                    print("=" * 60)
                    for i, (x, y) in enumerate(corners):
                        print("角点 {:2d}: ({:6.1f}, {:6.1f})".format(i + 1, x, y))
                        print("world: {}".format(world_corners[i]))
                    
                    print("\n" + "=" * 60)
                    print("OpenCV格式 (N, 1, 2):")
                    print("=" * 60)
                    print(corners_np)
                    
                    bev_img, H = bev_by_homography(calib_image, corners_np, world_corners_np, bev_w=BEV_WIDTH, bev_h=BEV_HEIGHT)
                    
                    print("\n" + "=" * 60)
                    print("H矩阵 (单应性矩阵):")
                    print("=" * 60)
                    print(H)
                    #保存H矩阵
                    save_homography(H, f"data/{args.name}_homography.npy")
                     
                cv2.imshow("BEV RGB", bev_img)
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
    import argparse
    
    parser = argparse.ArgumentParser(description="手动角点选择工具")
    parser.add_argument("--name", default="front", help="相机名称")
    args = parser.parse_args()
    main(args)