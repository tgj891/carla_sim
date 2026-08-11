import socket
import numpy as np
import struct
import time
import cv2
import threading
from queue import Queue, Empty
from collections import defaultdict

gui_lock = threading.Lock()

TCP_PORT = 9999
HOST = "127.0.0.1"
IMG_HEIGHT = 480
IMG_WIDTH = 640

# 消息类型定义
MSG_TYPE_IMAGE = 1   # 四路拼接图像
MSG_TYPE_GPS = 2     # GPS数据
MSG_TYPE_IMU = 3     # IMU数据
MSG_TYPE_POINT_CLOUD = 4  # 点云数据

# 消息队列（每个类型一个队列）
msg_queues = {
    MSG_TYPE_IMAGE: Queue(maxsize=10),
    MSG_TYPE_GPS: Queue(maxsize=100),
    MSG_TYPE_IMU: Queue(maxsize=100),
    MSG_TYPE_POINT_CLOUD: Queue(maxsize=10)
}

# 时间同步缓存（按时间戳缓存消息）
time_sync_buffer = defaultdict(dict)
time_sync_lock = threading.Lock()
TIME_SYNC_WINDOW = 0.05  # 50ms 时间窗口

# 传感器数据缓存
sensor_data = {
    "gps": None,
    "imu": None,
    "point_cloud": None
}
data_lock = threading.Lock()

# 统计计数
stats = {
    "received": defaultdict(int),
    "processed": defaultdict(int),
    "synced": 0
}


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


def extract_timestamp(msg_type, data):
    """从消息数据中提取时间戳"""
    try:
        if msg_type == MSG_TYPE_GPS:
            values = struct.unpack("<4d", data)
            return values[0]
        elif msg_type == MSG_TYPE_IMU:
            values = struct.unpack("<11d", data)
            return values[0]
        elif msg_type == MSG_TYPE_POINT_CLOUD:
            header = struct.unpack("<dIII", data[:20])
            return header[0]
        elif msg_type == MSG_TYPE_IMAGE:
            # 图像没有时间戳，使用当前时间
            return time.time()
    except:
        return time.time()
    return time.time()


def time_sync_worker():
    """时间同步工作线程：合并同一时间窗口内的消息"""
    while True:
        time.sleep(0.01)  # 10ms 检查间隔

        with time_sync_lock:
            if not time_sync_buffer:
                continue

            # 找出最早的时间戳
            timestamps = sorted(time_sync_buffer.keys())
            if not timestamps:
                continue

            # 检查是否有完整的数据组（GPS+IMU+Image）
            for ts in timestamps[:]:
                msg_group = time_sync_buffer[ts]

                # 检查是否包含关键数据
                has_image = MSG_TYPE_IMAGE in msg_group
                has_imu = MSG_TYPE_IMU in msg_group
                has_gps = MSG_TYPE_GPS in msg_group
                has_pc = MSG_TYPE_POINT_CLOUD in msg_group
                # 如果时间窗口已过，或有足够数据，则处理
                current_time = time.time()
                if has_image and has_imu and has_gps and has_pc:
                    stats["synced"] += 1

                    # 打印同步信息
                    print("[Sync] ts={:.3f}: img={}, gps={}, imu={}, pc={}".format(
                        ts,
                        "Y" if has_image else "N",
                        "Y" if has_gps else "N",
                        "Y" if has_imu else "N",
                        "Y" if has_pc else "N"))

                    # 清理已处理的时间戳
                    del time_sync_buffer[ts]


def handle_image(data):
    """处理四路拼接图像"""
    try:
        img = np.frombuffer(data, dtype=np.uint8).reshape(IMG_HEIGHT * 2, IMG_WIDTH * 2, 3)
        stats["processed"][MSG_TYPE_IMAGE] += 1
        if stats["processed"][MSG_TYPE_IMAGE] % 30 == 0:
            print("[Image] Frame {}: shape={}".format(stats["processed"][MSG_TYPE_IMAGE], img.shape))
        
        with gui_lock:
            cv2.imshow("Carla RGB", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
            
    except Exception as e:
        print("[Image] Error: {}".format(e))


def handle_gps(data):
    """处理GPS数据"""
    try:
        values = struct.unpack("<4d", data)
        gps = {
            "timestamp": values[0],
            "latitude": values[1],
            "longitude": values[2],
            "altitude": values[3]
        }
        with data_lock:
            sensor_data["gps"] = gps
        stats["processed"][MSG_TYPE_GPS] += 1
        if stats["processed"][MSG_TYPE_GPS] % 100 == 0:
            print("[GPS] lat={:.6f}, lon={:.6f}, alt={:.2f}m".format(
                gps["latitude"], gps["longitude"], gps["altitude"]))
    except Exception as e:
        print("[GPS] Error: {}".format(e))


def handle_imu(data):
    """处理IMU数据"""
    try:
        values = struct.unpack("<11d", data)
        imu = {
            "timestamp": values[0],
            "orientation": {"x": values[1], "y": values[2], "z": values[3], "w": values[4]},
            "angular_velocity": {"x": values[5], "y": values[6], "z": values[7]},
            "linear_acceleration": {"x": values[8], "y": values[9], "z": values[10]}
        }
        with data_lock:
            sensor_data["imu"] = imu
        stats["processed"][MSG_TYPE_IMU] += 1
        if stats["processed"][MSG_TYPE_IMU] % 100 == 0:
            print("[IMU] accel=({:.2f},{:.2f},{:.2f})".format(
                imu["linear_acceleration"]["x"],
                imu["linear_acceleration"]["y"],
                imu["linear_acceleration"]["z"]))
    except Exception as e:
        print("[IMU] Error: {}".format(e))

def convert_to_image(points):
    side_length = 720
    half_side = side_length / 2
    image = np.zeros((side_length, side_length, 3), dtype=np.uint8)
    range = 60.0  # 50米范围
    resolution = half_side / range  # 每米多少像素
    for point in points:
        # 将点坐标转换为图像坐标
        x = int(half_side - point[1] * resolution)
        y = int(half_side - point[0] * resolution)
        if 0 <= x < side_length and 0 <= y < side_length:
            image[y, x] = [255, 255, 255]  # 白色点
    return image

def handle_point_cloud(data):
    """处理点云数据"""
    try:
        # 解析点云数据: [timestamp(8B)][height(4B)][width(4B)][point_step(4B)][数据]
        header = struct.unpack("<dIII", data[:20])
        pc_header = {
            "timestamp": header[0],
            "height": header[1],
            "width": header[2],
            "point_step": header[3]
        }
        point_step = header[3]
        pc_bytes = data[20:]
        num_points = pc_header["height"] * pc_header["width"]
        # # 如果point_step是16（xyz+intensity，每个4字节float）
        if point_step == 16:
            points = np.frombuffer(pc_bytes, dtype=np.float32).reshape(-1, 4) # XYZI
        elif point_step == 12:
            points = np.frombuffer(pc_bytes, dtype=np.float32).reshape(-1, 3) # XYZ
        else:
            points = np.frombuffer(pc_bytes, dtype=np.float32).reshape(-1, point_step // 4)[:, :3]

        # convert image
        if False:
            pc_image = convert_to_image(points)
            with gui_lock:
                cv2.imshow("Carla point cloud RGB", pc_image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    cv2.destroyAllWindows()
    
        with data_lock:
            sensor_data["point_cloud"] = pc_header
        stats["processed"][MSG_TYPE_POINT_CLOUD] += 1
        if stats["processed"][MSG_TYPE_POINT_CLOUD] % 30 == 0:
            print("[PointCloud] pts={}, shape={}x{}, step={}".format(
                num_points, pc_header["height"], pc_header["width"], pc_header["point_step"]))
    except Exception as e:
        print("[PointCloud] Error: {}".format(e))

def o3d_render_loop():
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Lidar", width=1000, height=600)
    pcd = o3d.geometry.PointCloud()
    vis.add_geometry(pcd)

    while True:
        try:
            # 非阻塞取数据
            xyz = q.get_nowait()
            # ✅ 主线程才修改open3d几何体
            pcd.points = o3d.utility.Vector3dVector(xyz)
            vis.update_geometry(pcd)
            vis.poll_events()
            vis.update_renderer()
        except queue.Empty:
            # 没有新点云，正常刷新窗口
            vis.poll_events()
            vis.update_renderer()
            continue
    vis.destroy_window()

def message_processor(msg_type):
    """消息处理线程"""
    handler_map = {
        MSG_TYPE_IMAGE: handle_image,
        MSG_TYPE_GPS: handle_gps,
        MSG_TYPE_IMU: handle_imu,
        MSG_TYPE_POINT_CLOUD: handle_point_cloud
    }
    handler = handler_map.get(msg_type)
    if not handler:
        return

    while True:
        try:
            # 从队列获取消息（带超时）
            msg_data = msg_queues[msg_type].get(timeout=1.0)

            # 提取时间戳并加入时间同步缓存
            timestamp = extract_timestamp(msg_type, msg_data)
            with time_sync_lock:
                time_sync_buffer[timestamp][msg_type] = msg_data

            # 处理消息
            handler(msg_data)

        except Empty:
            continue
        except Exception as e:
            print("[Processor {}] Error: {}".format(msg_type, e))


def main():
    print("=" * 60)
    print("TCP Bridge Client Started (Parallel Processing)")
    print("=" * 60)
    print("Waiting for data from server...")
    print("Press Ctrl+C to exit\n")

    # 启动消息处理线程（每种消息类型一个线程）
    processor_threads = []
    for msg_type in [MSG_TYPE_IMAGE, MSG_TYPE_GPS, MSG_TYPE_IMU, MSG_TYPE_POINT_CLOUD]:
        t = threading.Thread(target=message_processor, args=(msg_type,), daemon=True)
        t.start()
        processor_threads.append(t)

    print("[Thread] Started {} processor threads + 1 sync thread".format(len(processor_threads)))

    # 接收循环（主线程只负责接收）
    while True:
        try:
            sock = connect_socket()
            while True:
                # 接收16字节长度头
                header_buf = recv_all(sock, 16)
                total_len = int(header_buf.strip())

                # 接收剩余数据 (4字节类型 + 数据体)
                remaining = recv_all(sock, total_len)

                # 解析消息类型和数据
                msg_type = struct.unpack("<I", remaining[:4])[0]
                msg_data = remaining[4:]

                # 统计接收
                stats["received"][msg_type] += 1

                # 放入对应队列（非阻塞，队列满则丢弃）
                try:
                    msg_queues[msg_type].put_nowait(msg_data)
                except:
                    print("\nmsg_queues full, lost ...")

        except ConnectionError:
            print("\nConnection lost, reconnecting...")
            time.sleep(1)
        except KeyboardInterrupt:
            print("\n\nClient stopped by user")
            print("\n[Stats] Received: img={}, gps={}, imu={}, pc={}".format(
                stats["received"][MSG_TYPE_IMAGE],
                stats["received"][MSG_TYPE_GPS],
                stats["received"][MSG_TYPE_IMU],
                stats["received"][MSG_TYPE_POINT_CLOUD]))
            print("[Stats] Processed: img={}, gps={}, imu={}, pc={}".format(
                stats["processed"][MSG_TYPE_IMAGE],
                stats["processed"][MSG_TYPE_GPS],
                stats["processed"][MSG_TYPE_IMU],
                stats["processed"][MSG_TYPE_POINT_CLOUD]))
            print("[Stats] Synced groups: {}".format(stats["synced"]))
            break
        except Exception as e:
            print("Error:", e)
            time.sleep(1)


if __name__ == "__main__":
    main()
