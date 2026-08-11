import socket
import numpy as np
import cv2

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

def main():
    while True:
        try:
            sock = connect_socket()
            while True:
                header_buf = recv_all(sock, 16)
                data_len = int(header_buf.strip())

                buffer = recv_all(sock, data_len)

                img = np.frombuffer(buffer, dtype=np.uint8).reshape(IMG_HEIGHT, IMG_WIDTH, 3).copy()
            
                cv2.line(img, (int(IMG_WIDTH/4), 0), (int(IMG_WIDTH/4), IMG_HEIGHT), (0, 0, 255), 1)
                cv2.line(img, (int(IMG_WIDTH -IMG_WIDTH/4), 0), (int(IMG_WIDTH -IMG_WIDTH/4), IMG_HEIGHT), (0, 0, 255), 1)

                # # 4路图像拼接
                # # [ front ][ left ]
                # [ right ][ back ]
                # front_image = cv2_crop(img, 0, 0, image_width, image_height)
                # 棋盘格检测
                # found, corners = detect_grid_corners_by_hough(front_image)
                # if found:
                #     print("✅ 直线交点法获取角点成功，未使用原生findChessboardCorners")
                #     # 绘制角点
                #     for p in corners:
                #         cv2.circle(front_image, tuple(np.int32(p[0])),4,(0,0,255),-1)
                    
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