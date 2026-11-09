#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
多传感器标定 Web 应用后端
Python 2.7 + Flask + ROS
"""

import threading
import signal
import sys
import os
import json
import numpy as np
import flask
from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO

import rospy
import tf as ros_tf
from sensor_msgs.msg import PointCloud2, Image, CompressedImage
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge

# ========= Flask 配置 =========
app = Flask(__name__)
app.config['SECRET_KEY'] = 'multisensor_calib'
socketio = SocketIO(app, async_mode='threading', cors_allowed_origins='*')

# ========= ROS 配置 =========
bridge = CvBridge()
ros_subscribers = {}
ros_lock = threading.Lock()
tf_listener = None  # TF 监听器，在 main() 中初始化

# 自车坐标系名（ego_vehicle），lidar 点云会转到此坐标系下
BASE_FRAME = 'ego_vehicle'

# 保存的标定参数文件
CALIB_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'calib_params.json')
# {topic_name: 4x4 np.array} 从文件加载，标定页保存后更新
saved_transforms = {}
saved_lock = threading.Lock()

# 当前标定的激光雷达话题
current_lidar_topic = None
current_transform = {
    'x': 0.0, 'y': 0.0, 'z': 0.0,
    'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0
}

# 投影线配置
projection_lines = {
    'xy_h': 0.0, 'xy_v': 0.0,
    'xz_h': 0.0, 'xz_v': 0.0,
    'yz_h': 0.0, 'yz_v': 0.0
}

# ========= 共享数据（ROS回调写入，HTTP 端点读取）=========
latest_pointcloud_data = {
    'projections': None,
    'topic': None,
    'num_points': 0,
    'lines': None
}
latest_message_data = {
    'topic': None,
    'type': None,
    'data': None
}
# 图像数据通过 HTTP 端点提供，不走 WebSocket（避免大数据导致 500 错误）
latest_image_data = {
    'jpeg': None,
    'topic': None,
    'width': 0,
    'height': 0
}
data_lock = threading.Lock()

def create_transform_matrix(params):
    """创建 4x4 变换矩阵"""
    x, y, z = params['x'], params['y'], params['z']
    roll = np.radians(params['roll'])
    pitch = np.radians(params['pitch'])
    yaw = np.radians(params['yaw'])

    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr]
    ])

    T = np.eye(4)
    T[:3, :3] = R
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    return T

def transform_pointcloud(points, T):
    """对点云应用变换"""
    if len(points) == 0:
        return points
    ones = np.ones((points.shape[0], 1))
    pts_homo = np.hstack([points, ones])
    return np.dot(pts_homo, T.T)[:, :3]

def load_saved_transforms():
    """从文件加载保存的标定参数"""
    global saved_transforms
    if not os.path.exists(CALIB_FILE):
        return
    try:
        with open(CALIB_FILE, 'r') as f:
            data = json.load(f)
        with saved_lock:
            saved_transforms = {}
            for topic, params in data.items():
                saved_transforms[topic] = create_transform_matrix(params)
        rospy.loginfo("Loaded %d saved transforms" % len(saved_transforms))
    except Exception as e:
        rospy.logwarn("Failed to load calib params: %s" % str(e))

def save_transform(topic, params):
    """保存单个话题的标定参数"""
    with saved_lock:
        saved_transforms[topic] = create_transform_matrix(params)
    # 写入文件
    data = {}
    try:
        if os.path.exists(CALIB_FILE):
            with open(CALIB_FILE, 'r') as f:
                data = json.load(f)
    except:
        pass
    data[topic] = params
    try:
        with open(CALIB_FILE, 'w') as f:
            json.dump(data, f, indent=2)
    except Exception as e:
        rospy.logerr("Failed to save calib params: %s" % str(e))

def pointcloud_to_projections(points, max_points=2000):
    """将点云投影到三个平面（固定索引采样，避免每次投影点不一样）"""
    if len(points) == 0:
        return {'xy': [], 'xz': [], 'yz': []}
    if len(points) > max_points:
        # 用固定步长均匀采样，不使用随机采样
        step = len(points) / max_points
        indices = (np.arange(max_points) * step).astype(int)
        points = points[indices]
    # 四舍五入到 2 位小数，减小 JSON 大小
    points = np.round(points, 2)
    return {
        'xy': points[:, :2].tolist(),
        'xz': points[:, [0, 2]].tolist(),
        'yz': points[:, 1:3].tolist()
    }

# ========= ROS 回调（只写共享数据，不直接 emit）=========
def lidar_callback(msg, topic_name):
    """激光雷达点云回调 - 只存储数据"""
    try:
        points_list = []
        for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            points_list.append([p[0], p[1], p[2]])

        if len(points_list) == 0:
            return

        points = np.array(points_list)
        # 只保留半径 10m 范围内的点
        dists = np.sqrt(points[:, 0]**2 + points[:, 1]**2 + points[:, 2]**2)
        points = points[dists <= 10.0]

        if len(points) == 0:
            return

        T = create_transform_matrix(current_transform)
        points_transformed = transform_pointcloud(points, T)
        projections = pointcloud_to_projections(points_transformed)

        with data_lock:
            latest_pointcloud_data['projections'] = projections
            latest_pointcloud_data['topic'] = topic_name
            latest_pointcloud_data['num_points'] = len(points_transformed)
            latest_pointcloud_data['lines'] = dict(projection_lines)
    except Exception as e:
        rospy.logerr("Error in lidar_callback: %s" % str(e))

def image_callback(msg, topic_name):
    """图像回调 - 存储 JPEG 原始字节（通过 HTTP 端点提供）"""
    try:
        cv_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        import cv2
        _, buffer = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        jpeg_bytes = buffer.tostring()

        with data_lock:
            latest_image_data['jpeg'] = jpeg_bytes
            latest_image_data['topic'] = topic_name
            latest_image_data['width'] = msg.width
            latest_image_data['height'] = msg.height
        # print("Image stored: %s, %d bytes, %dx%d" % (topic_name, len(jpeg_bytes), msg.width, msg.height))
    except Exception as e:
        rospy.logwarn("Failed to process image from %s: %s" % (topic_name, str(e)))

def generic_callback(msg, topic_name, msg_type):
    """通用消息回调 - 只存储数据"""
    try:
        msg_dict = {}
        for slot in msg.__slots__:
            if not slot.startswith('_'):
                value = getattr(msg, slot)
                if hasattr(value, '__slots__'):
                    nested = {}
                    for s in value.__slots__:
                        if not s.startswith('_'):
                            nested[s] = getattr(value, s)
                    msg_dict[slot] = nested
                else:
                    msg_dict[slot] = str(value)

        with data_lock:
            latest_message_data['topic'] = topic_name
            latest_message_data['type'] = msg_type
            latest_message_data['data'] = msg_dict
    except Exception as e:
        rospy.logwarn("Failed to process message from %s: %s" % (topic_name, str(e)))

def unsubscribe_all():
    """取消所有订阅"""
    global ros_subscribers
    with ros_lock:
        for topic_name, sub in ros_subscribers.items():
            sub.unregister()
            rospy.loginfo("Unsubscribed from: %s" % topic_name)
        ros_subscribers.clear()
    # 清空缓存数据
    with data_lock:
        latest_pointcloud_data['projections'] = None
        latest_pointcloud_data['topic'] = None
        latest_pointcloud_data['num_points'] = 0
        latest_pointcloud_data['lines'] = None
        latest_message_data['topic'] = None
        latest_message_data['type'] = None
        latest_message_data['data'] = None
        latest_image_data['jpeg'] = None
        latest_image_data['topic'] = None
        latest_image_data['width'] = 0
        latest_image_data['height'] = 0

def subscribe_topic(topic_name, msg_type):
    """订阅指定话题（同时只允许一个话题）"""
    # 先取消之前的所有订阅
    unsubscribe_all()

    with ros_lock:
        try:
            if msg_type == 'sensor_msgs/PointCloud2':
                sub = rospy.Subscriber(topic_name, PointCloud2,
                                      lidar_callback, callback_args=topic_name,
                                      queue_size=1)
            elif msg_type in ('sensor_msgs/Image', 'sensor_msgs/CompressedImage'):
                msg_cls = Image if msg_type == 'sensor_msgs/Image' else CompressedImage
                sub = rospy.Subscriber(topic_name, msg_cls,
                                      image_callback, callback_args=topic_name,
                                      queue_size=1)
            else:
                from std_msgs.msg import String
                sub = rospy.Subscriber(topic_name, String,
                                      generic_callback,
                                      callback_args=(topic_name, msg_type),
                                      queue_size=10)
            ros_subscribers[topic_name] = sub
            rospy.loginfo("Subscribed to: %s (%s)" % (topic_name, msg_type))
        except Exception as e:
            rospy.logerr("Failed to subscribe %s: %s" % (topic_name, str(e)))

# ========= 点云融合：多话题订阅 =========
fusion_subscribers = {}   # {topic_name: rospy.Subscriber}
fusion_pointclouds = {}   # {topic_name: np.array(Nx3)}
fusion_lock = threading.Lock()

def fusion_lidar_callback(msg, topic_name):
    """融合模式点云回调 - 先应用保存的标定 T 矩阵，再转到自车坐标系"""
    try:
        points_list = []
        for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            points_list.append([p[0], p[1], p[2]])
        if len(points_list) == 0:
            return
        points = np.array(points_list)

        # 1. 先应用保存的标定变换矩阵 T（lidar 自身坐标系内的微调）
        with saved_lock:
            T_calib = saved_transforms.get(topic_name)
        if T_calib is not None:
            points = transform_pointcloud(points, T_calib)

        # 2. 再通过 TF 将点云从 lidar 坐标系转到 ego_vehicle
        lidar_frame = msg.header.frame_id
        if tf_listener is not None and lidar_frame:
            try:
                # 等待 TF 可用（最多 0.1 秒）
                tf_listener.waitForTransform(
                    BASE_FRAME, lidar_frame,
                    msg.header.stamp, rospy.Duration(0.1))
                # 获取变换 (translation, rotation_quaternion)
                (trans, quat) = tf_listener.lookupTransform(
                    BASE_FRAME, lidar_frame, rospy.Time(0))
                # 将四元数转为 4x4 齐次变换矩阵
                T = ros_tf.transformations.quaternion_matrix(quat)
                T[0:3, 3] = trans
                # 对所有点做变换
                points = transform_pointcloud(points, T)
            except Exception as tf_err:
                rospy.logwarn("TF transform failed for %s->%s: %s" %
                              (lidar_frame, BASE_FRAME, str(tf_err)))

        # 只保留半径 10m 范围内的点
        dists = np.sqrt(points[:, 0]**2 + points[:, 1]**2 + points[:, 2]**2)
        points = points[dists <= 10.0]
        # 限制每话题最多 5000 点
        if len(points) > 5000:
            step = len(points) / 5000
            indices = (np.arange(5000) * step).astype(int)
            points = points[indices]
        with fusion_lock:
            fusion_pointclouds[topic_name] = points
    except Exception as e:
        rospy.logerr("Error in fusion_lidar_callback: %s" % str(e))

# ========= Flask 路由 =========
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/fusion')
def fusion_page():
    # 进入融合页时关闭标定页订阅
    unsubscribe_all()
    return render_template('fusion.html')

@app.route('/api/unsubscribe_all', methods=['POST'])
def unsubscribe_all_api():
    """关闭所有标定页订阅"""
    unsubscribe_all()
    return jsonify({'success': True})

@app.route('/api/fusion/unsubscribe_all', methods=['POST'])
def fusion_unsubscribe_all_api():
    """关闭所有融合页订阅"""
    global fusion_subscribers, fusion_pointclouds
    with ros_lock:
        for topic, sub in fusion_subscribers.items():
            try:
                sub.unregister()
            except:
                pass
        fusion_subscribers = {}
    with fusion_lock:
        fusion_pointclouds = {}
    rospy.loginfo("All fusion subscriptions cleared")
    return jsonify({'success': True})

@app.route('/api/fusion/subscribe', methods=['POST'])
def fusion_subscribe():
    """订阅点云话题用于融合（可同时订阅多个）"""
    data = request.json
    topic_name = data.get('topic')
    if not topic_name:
        return jsonify({'success': False, 'error': 'no topic'})
    with ros_lock:
        if topic_name in fusion_subscribers:
            return jsonify({'success': True, 'msg': 'already subscribed'})
        try:
            sub = rospy.Subscriber(topic_name, PointCloud2,
                                  fusion_lidar_callback, callback_args=topic_name,
                                  queue_size=1)
            fusion_subscribers[topic_name] = sub
            rospy.loginfo("Fusion subscribed: %s" % topic_name)
            return jsonify({'success': True})
        except Exception as e:
            return jsonify({'success': False, 'error': str(e)})

@app.route('/api/fusion/unsubscribe', methods=['POST'])
def fusion_unsubscribe():
    """取消订阅点云话题"""
    data = request.json
    topic_name = data.get('topic')
    with ros_lock:
        if topic_name in fusion_subscribers:
            fusion_subscribers[topic_name].unregister()
            del fusion_subscribers[topic_name]
            with fusion_lock:
                if topic_name in fusion_pointclouds:
                    del fusion_pointclouds[topic_name]
            rospy.loginfo("Fusion unsubscribed: %s" % topic_name)
    return jsonify({'success': True})

@app.route('/api/fusion/pointcloud')
def fusion_get_pointcloud():
    """获取融合后的点云数据（3D 格式）"""
    with fusion_lock:
        all_points = []
        for topic, pts in fusion_pointclouds.items():
            all_points.append(pts)
    if len(all_points) == 0:
        return jsonify({'points': [], 'num_points': 0})
    merged = np.vstack(all_points)
    # 转换为扁平 [x,y,z, x,y,z, ...] 数组，减少 JSON 大小
    flat = np.round(merged, 2).flatten().tolist()
    return jsonify({'points': flat, 'num_points': len(merged)})

@app.errorhandler(500)
def handle_500(e):
    import traceback
    traceback.print_exc()
    return 'Internal Server Error: ' + str(e), 500

@app.route('/api/topics')
def get_topics():
    topics = []
    try:
        result = rospy.get_master().getTopicTypes()
        topic_type_list = result[2] if len(result) >= 3 else []
        for item in topic_type_list:
            if isinstance(item, (list, tuple)) and len(item) >= 2:
                topics.append({'name': item[0], 'type': item[1]})
    except Exception as e:
        rospy.logerr("Failed to get topics: %s" % str(e))
    return jsonify({'topics': topics})

@app.route('/api/latest_image')
def get_latest_image():
    """HTTP 端点提供最新图像（避免 WebSocket 传输大数据）"""
    with data_lock:
        jpeg_data = latest_image_data.get('jpeg')
        topic = latest_image_data.get('topic', '')
        width = latest_image_data.get('width', 0)
        height = latest_image_data.get('height', 0)
    if jpeg_data is not None:
        response = flask.Response(jpeg_data, mimetype='image/jpeg')
        response.headers['X-Topic'] = topic
        response.headers['X-Width'] = str(width)
        response.headers['X-Height'] = str(height)
        response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
        response.headers['Access-Control-Expose-Headers'] = 'X-Topic, X-Width, X-Height'
        return response
    return 'No image available', 404

@app.route('/api/latest_pointcloud')
def get_latest_pointcloud():
    """HTTP 端点提供最新点云投影数据"""
    with data_lock:
        projections = latest_pointcloud_data.get('projections')
        topic = latest_pointcloud_data.get('topic', '')
        num_points = latest_pointcloud_data.get('num_points', 0)
        lines = latest_pointcloud_data.get('lines')
    if projections is not None:
        return jsonify({
            'topic': topic,
            'projections': projections,
            'num_points': num_points,
            'lines': lines or {}
        })
    return jsonify({'topic': '', 'projections': {'xy': [], 'xz': [], 'yz': []},
                    'num_points': 0, 'lines': {}})

@app.route('/api/latest_message')
def get_latest_message():
    """HTTP 端点提供最新消息数据"""
    with data_lock:
        topic = latest_message_data.get('topic', '')
        msg_type = latest_message_data.get('type', '')
        data = latest_message_data.get('data')
    if data is not None:
        return jsonify({'topic': topic, 'type': msg_type, 'data': data})
    return jsonify({'topic': '', 'type': '', 'data': {}})

@app.route('/api/subscribe', methods=['POST'])
def subscribe():
    data = request.json
    topic_name = data.get('topic')
    msg_type = data.get('type')
    if topic_name and msg_type:
        subscribe_topic(topic_name, msg_type)
        return jsonify({'success': True})
    return jsonify({'success': False, 'error': 'Missing parameters'})

@app.route('/api/transform', methods=['POST'])
def update_transform():
    global current_transform
    data = request.json
    current_transform.update({
        'x': float(data.get('x', 0)),
        'y': float(data.get('y', 0)),
        'z': float(data.get('z', 0)),
        'roll': float(data.get('roll', 0)),
        'pitch': float(data.get('pitch', 0)),
        'yaw': float(data.get('yaw', 0))
    })
    T = create_transform_matrix(current_transform)
    return jsonify({
        'success': True,
        'transform': current_transform,
        'matrix': T.tolist()
    })

@app.route('/api/projection_lines', methods=['POST'])
def update_projection_lines():
    global projection_lines
    data = request.json
    projection_lines.update({
        'xy_h': float(data.get('xy_h', 0)),
        'xy_v': float(data.get('xy_v', 0)),
        'xz_h': float(data.get('xz_h', 0)),
        'xz_v': float(data.get('xz_v', 0)),
        'yz_h': float(data.get('yz_h', 0)),
        'yz_v': float(data.get('yz_v', 0))
    })
    return jsonify({'success': True, 'lines': dict(projection_lines)})

@app.route('/api/select_lidar', methods=['POST'])
def select_lidar():
    global current_lidar_topic
    data = request.json
    topic = data.get('topic')
    if topic:
        current_lidar_topic = topic
        subscribe_topic(topic, 'sensor_msgs/PointCloud2')
        return jsonify({'success': True, 'topic': topic})
    return jsonify({'success': False, 'error': 'Missing topic'})

@app.route('/api/save_params', methods=['POST'])
def save_params():
    """保存标定参数（变换矩阵 T + 话题）"""
    data = request.json
    topic = data.get('topic')
    if not topic:
        return jsonify({'success': False, 'error': 'Missing topic'})
    params = {
        'x': float(data.get('x', 0)),
        'y': float(data.get('y', 0)),
        'z': float(data.get('z', 0)),
        'roll': float(data.get('roll', 0)),
        'pitch': float(data.get('pitch', 0)),
        'yaw': float(data.get('yaw', 0))
    }
    save_transform(topic, params)
    rospy.loginfo("Saved transform for %s: %s" % (topic, str(params)))
    return jsonify({'success': True, 'topic': topic})

# ========= WebSocket 事件 =========
@socketio.on('connect')
def handle_connect():
    rospy.loginfo('Client connected')

@socketio.on('disconnect')
def handle_disconnect():
    rospy.loginfo('Client disconnected')

# ========= 主函数 =========
def signal_handler(sig, frame):
    print("\nShutting down...")
    try:
        rospy.signal_shutdown("KeyboardInterrupt")
    except:
        pass
    sys.exit(0)

def main():
    global tf_listener
    rospy.init_node('multisensor_calib_web', anonymous=True)

    # 初始化 TF 监听器
    tf_listener = ros_tf.TransformListener()
    rospy.loginfo("TF listener initialized, base_frame=%s" % BASE_FRAME)

    # 加载保存的标定参数
    load_saved_transforms()

    # 注册 Ctrl+C 信号处理
    signal.signal(signal.SIGINT, signal_handler)

    print("=" * 50)
    print("多传感器标定 Web 应用启动中...")
    print("访问地址: http://localhost:5566")
    print("=" * 50)

    socketio.run(app, host='0.0.0.0', port=5566, debug=False)

if __name__ == '__main__':
    main()
