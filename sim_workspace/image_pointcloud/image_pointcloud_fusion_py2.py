#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
激光雷达点云投影到图像融合脚本
功能：订阅 Carla ROS 话题，将激光雷达点云投影到相机图像上

订阅话题：
  - /carla/ego_vehicle/rgb_main_front/camera_info  (相机内参)
  - /carla/ego_vehicle/rgb_main_front/image       (相机图像)
  - /carla/ego_vehicle/lidar                       (激光雷达点云)
  - /tf                                             (外参变换)

发布话题：
  - /carla/ego_vehicle/fusion/image                (融合图像)
"""

import rospy
import numpy as np
import threading
import math
import tf
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import cv2
from pyquaternion import Quaternion

class LidarCameraFusion:
    """激光雷达和相机融合类"""

    def __init__(self):
        # 相机内参（通过 camera_info 话题获取）
        self.camera_matrix = None
        self.image_width = None
        self.image_height = None

        # 外参变换矩阵（通过 /tf 话题获取）
        self.lidar_to_camera_transform = None
        self.tf_initialized = False

        # TF 监听
        self.tf_listener = tf.TransformListener()

        # ROS 接口
        self.bridge = CvBridge()
        self.fusion_pub = rospy.Publisher('/carla/ego_vehicle/fusion/image', Image, queue_size=1)

        # 订阅话题
        self.camera_info_sub = rospy.Subscriber(
            '/carla/ego_vehicle/rgb_main_front/camera_info', CameraInfo,
            self.camera_info_callback, queue_size=1
        )
        self.image_sub = rospy.Subscriber(
            '/carla/ego_vehicle/rgb_main_front/image', Image,
            self.image_callback, queue_size=1
        )
        self.lidar_sub = rospy.Subscriber(
            '/carla/ego_vehicle/lidar', PointCloud2,
            self.lidar_callback, queue_size=1
        )

        # 数据缓存
        self.lidar_data = None
        self.camera_data = None
        self.lidar_lock = threading.Lock()
        self.camera_lock = threading.Lock()

        # 统计
        self.frame_count = 0
        self.projected_points = 0

        rospy.loginfo("Waiting for camera_info and TF transforms...")

    def homo_transform_inv(self, T):
        """
        4x4齐次变换矩阵求逆
        T: shape (4,4)
        return T_inv (4,4)
        """
        T_inv = np.eye(4)
        R = T[0:3, 0:3]
        t = T[0:3, 3]
        R_T = R.T          # 旋转矩阵的逆=转置
        T_inv[0:3,0:3] = R_T
        T_inv[0:3,3] = -np.dot(R_T, t)
        return T_inv
    
    def update_transforms(self):
        """
        从 /tf 话题获取外参并计算 lidar 到 camera 的变换矩阵
        """
        try:
            # 等待 TF 数据可用
            self.tf_listener.waitForTransform(
                'ego_vehicle', 'ego_vehicle/rgb_main_front',
                rospy.Time(0), rospy.Duration(5.0)
            )
            self.tf_listener.waitForTransform(
                'ego_vehicle', 'ego_vehicle/lidar',
                rospy.Time(0), rospy.Duration(5.0)
            )

            # 获取相机相对车辆的变换 (camera <- ego_vehicle)
            (cam_trans, cam_rot) = self.tf_listener.lookupTransform(
                'ego_vehicle', 'ego_vehicle/rgb_main_front', rospy.Time(0)
            )

            # 获取激光雷达相对车辆的变换 (lidar <- ego_vehicle)
            (lidar_trans, lidar_rot) = self.tf_listener.lookupTransform(
                'ego_vehicle', 'ego_vehicle/lidar', rospy.Time(0)
            )

            # 计算变换矩阵
            # T_camera_ego: 相机坐标系到车辆坐标系的逆变换
            # T_lidar_ego: 车辆坐标系到激光雷达坐标系

            # 相机相对车辆的变换矩阵 (camera <- ego_vehicle)
            T_camera_ego = self.build_transform_matrix(cam_trans, cam_rot)

            # 激光雷达相对车辆的变换矩阵 (lidar <- ego_vehicle)
            T_lidar_ego = self.build_transform_matrix(lidar_trans, lidar_rot)

            # 计算 lidar 到 camera 的变换 (camera <- lidar)
            # T_camera_lidar = T_camera_ego * inv(T_lidar_ego)
            # 即: P_camera = T_camera_ego * inv(T_lidar_ego) * P_lidar
            # 或: T_camera_lidar = T_camera_ego * T_ego_lidar
            # 其中 T_ego_lidar = inv(T_lidar_ego)

            T_ego_camera = self.homo_transform_inv(T_camera_ego)
            T_lidar_camera = np.dot(T_ego_camera, T_lidar_ego)

            self.lidar_to_camera_transform = T_lidar_camera

            if not self.tf_initialized:
                rospy.loginfo("TF transforms initialized!")
                rospy.loginfo("Camera transform (camera <- ego):")
                rospy.loginfo("  translation: ({:.3f}, {:.3f}, {:.3f})".format(*cam_trans))
                rospy.loginfo("  rotation: ({:.3f}, {:.3f}, {:.3f}, {:.3f})".format(*cam_rot))
                rospy.loginfo("Lidar transform (lidar <- ego):")
                rospy.loginfo("  translation: ({:.3f}, {:.3f}, {:.3f})".format(*lidar_trans))
                rospy.loginfo("  rotation: ({:.3f}, {:.3f}, {:.3f}, {:.3f})".format(*lidar_rot))
                rospy.loginfo("Lidar to camera transform:")
                rospy.loginfo(self.lidar_to_camera_transform)
                self.tf_initialized = True

            return True

        except tf.Exception as e:
            rospy.logwarn_throttle(2.0, "TF lookup failed: {}".format(e))
            return False
        except Exception as e:
            rospy.logwarn_throttle(2.0, "TF update error: {}".format(e))
            return False

    def build_transform_matrix(self, translation, quaternion):
        """
        构建 4x4 变换矩阵（局部到父坐标系）
        translation: (x, y, z)
        quaternion: (x, y, z, w)
        """
        x, y, z = translation
        qx, qy, qz, qw = quaternion
        
        # 从四元数计算旋转矩阵
        q = Quaternion(w=qw, x=qx, y=qy, z=qz)
        q_unit = q.normalised
        R = q_unit.rotation_matrix

        # 构建变换矩阵
        T = np.eye(4)
        T[:3, :3] = R
        T[0, 3] = x
        T[1, 3] = y
        T[2, 3] = z

        return T

    def build_transform_matrix_inverse(self, translation, quaternion):
        """
        构建 4x4 变换矩阵的逆矩阵（父到局部）
        """
        x, y, z = translation
        qx, qy, qz, qw = quaternion

        # 归一化四元数
        norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
        if norm < 1e-8:
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        else:
            qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm

        # 从四元数计算旋转矩阵
        R = np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)]
        ])

        # 逆变换：旋转转置，平移取反
        R_inv = R.T
        t_inv = -np.dot(R_inv, np.array([x, y, z]))

        T_inv = np.eye(4)
        T_inv[:3, :3] = R_inv
        T_inv[:3, 3] = t_inv

        return T_inv

    def camera_info_callback(self, msg):
        """相机内参回调"""
        try:
            # 从 CameraInfo 消息提取内参矩阵 K
            # K 是 3x3 矩阵，按行存储在 msg.K 中
            # [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            k = list(msg.K)
            self.camera_matrix = np.array([
                [k[0], k[1], k[2]],
                [k[3], k[4], k[5]],
                [k[6], k[7], k[8]]
            ])
            self.image_width = msg.width
            self.image_height = msg.height

            rospy.loginfo("Camera info received: {}x{}".format(msg.width, msg.height))
            rospy.loginfo("  fx={:.6f}, fy={:.6f}, cx={:.6f}, cy={:.6f}".format(
                self.camera_matrix[0, 0], self.camera_matrix[1, 1],
                self.camera_matrix[0, 2], self.camera_matrix[1, 2]))
        except Exception as e:
            rospy.logerr("Camera info callback error: {}".format(e))

    def image_callback(self, msg):
        """相机图像回调"""
        try:
            # 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            with self.camera_lock:
                self.camera_data = {
                    'timestamp': msg.header.stamp.to_sec(),
                    'image': cv_image
                }
        except Exception as e:
            rospy.logerr("Image callback error: {}".format(e))

    def lidar_callback(self, msg):
        """激光雷达点云回调"""
        try:
            # 解析点云数据
            points = self.parse_pointcloud(msg)

            with self.lidar_lock:
                self.lidar_data = {
                    'timestamp': msg.header.stamp.to_sec(),
                    'points': points
                }
        except Exception as e:
            rospy.logerr("Lidar callback error: {}".format(e))

    def parse_pointcloud(self, msg):
        """
        解析 PointCloud2 消息
        返回 numpy 数组 (N, 4): [x, y, z, intensity]
        """
        # 获取字段信息
        fields = msg.fields
        field_names = [f.name for f in fields]
        field_offsets = [f.offset for f in fields]
        field_dtypes = [f.datatype for f in fields]

        # 确定点的结构
        point_step = msg.point_step
        num_points = msg.height * msg.width

        # 解析原始数据
        raw_data = np.frombuffer(msg.data, dtype=np.uint8)

        # 构建点数组
        points = np.zeros((num_points, 4), dtype=np.float32)

        # 找到 x, y, z, intensity 的偏移
        offset_x = offset_y = offset_z = offset_i = None
        for i, name in enumerate(field_names):
            if name == 'x':
                offset_x = field_offsets[i]
            elif name == 'y':
                offset_y = field_offsets[i]
            elif name == 'z':
                offset_z = field_offsets[i]
            elif name == 'intensity':
                offset_i = field_offsets[i]

        # 提取数据
        for i in range(num_points):
            base = i * point_step
            points[i, 0] = np.frombuffer(raw_data[base + offset_x:base + offset_x + 4], dtype=np.float32)[0]
            points[i, 1] = np.frombuffer(raw_data[base + offset_y:base + offset_y + 4], dtype=np.float32)[0]
            points[i, 2] = np.frombuffer(raw_data[base + offset_z:base + offset_z + 4], dtype=np.float32)[0]
            if offset_i is not None:
                points[i, 3] = np.frombuffer(raw_data[base + offset_i:base + offset_i + 4], dtype=np.float32)[0]

        return points

    def project_points_to_image(self, points_3d):
        """
        将3D点投影到图像平面
        """
        if self.camera_matrix is None:
            return None, None

        # 过滤在相机前方的点 (z > 0)
        valid_mask = points_3d[:, 2] > 0.1  # 避免除零
        points_valid = points_3d[valid_mask]

        if len(points_valid) == 0:
            return np.array([]), np.array([])

        # 投影到图像平面 (针孔模型)
        # p_image = K * P_camera
        points_proj = np.dot(self.camera_matrix, points_valid.T).T

        # 归一化
        u = points_proj[:, 0] / points_proj[:, 2]
        v = points_proj[:, 1] / points_proj[:, 2]
        depth = points_valid[:, 2]

        return np.column_stack([u, v]), depth

    def fuse_data(self):
        """融合激光雷达和相机数据"""
        if self.camera_matrix is None or self.lidar_to_camera_transform is None:
            return None

        # 获取当前数据
        with self.lidar_lock:
            lidar_data = self.lidar_data.copy() if self.lidar_data else None

        with self.camera_lock:
            camera_data = self.camera_data.copy() if self.camera_data else None

        if lidar_data is None or camera_data is None:
            return None

        # 获取点云数据
        points_lidar = lidar_data['points'][:, :3]  # 只取 xyz

        # 将点云从激光雷达坐标系转换到相机坐标系
        # P_camera = T_camera_lidar * P_lidar
        points_homo = np.column_stack([points_lidar, np.ones(len(points_lidar))])  # (N, 4)
        points_camera_homo = np.dot(self.lidar_to_camera_transform, points_homo.T).T
        points_camera = points_camera_homo[:, :3]  # (N, 3)

        # 投影到图像平面
        points_2d, depths = self.project_points_to_image(points_camera)

        if len(points_2d) == 0:
            return None

        # 过滤在图像范围内的点
        height, width = camera_data['image'].shape[:2]
        valid_mask = (
            (points_2d[:, 0] >= 0) & (points_2d[:, 0] < width) &
            (points_2d[:, 1] >= 0) & (points_2d[:, 1] < height)
        )
        points_2d_valid = points_2d[valid_mask]
        depths_valid = depths[valid_mask]

        if len(points_2d_valid) == 0:
            return None

        # 在图像上绘制点云
        fusion_image = camera_data['image'].copy()

        # 绘制点云
        self.draw_points_vectorized(fusion_image, points_2d_valid, depths_valid)

        self.projected_points = len(points_2d_valid)
        return fusion_image

    def draw_points_vectorized(self, image, points_2d, depths):
        """
        向量化绘制点云（性能优化）
        """
        height, width = image.shape[:2]

        # 过滤有效点
        valid = (
            (points_2d[:, 0] >= 0) & (points_2d[:, 0] < width) &
            (points_2d[:, 1] >= 0) & (points_2d[:, 1] < height)
        )
        points_2d = points_2d[valid]
        depths = depths[valid]

        if len(points_2d) == 0:
            return

        # 转换为整数坐标
        u_coords = points_2d[:, 0].astype(np.int32)
        v_coords = points_2d[:, 1].astype(np.int32)

        # 根据深度计算颜色 (近红远蓝)
        max_depth = 50.0
        min_depth = 1.0
        depth_norm = np.clip((depths - min_depth) / (max_depth - min_depth), 0.0, 1.0)

        # BGR 颜色
        b_colors = (255 * depth_norm).astype(np.uint8)
        r_colors = (255 * (1.0 - depth_norm)).astype(np.uint8)
        g_colors = np.zeros_like(b_colors)

        # 绘制点（使用 cv2.circle 批量绘制，半径2）
        for i in range(len(u_coords)):
            cv2.circle(image, (int(u_coords[i]), int(v_coords[i])), 2,
                       (int(b_colors[i]), int(g_colors[i]), int(r_colors[i])), -1)

    def run(self):
        """主循环"""
        rospy.loginfo("Lidar-Camera fusion node started")
        rospy.loginfo("Subscribed topics:")
        rospy.loginfo("  - /carla/ego_vehicle/rgb_main_front/camera_info")
        rospy.loginfo("  - /carla/ego_vehicle/rgb_main_front/image")
        rospy.loginfo("  - /carla/ego_vehicle/lidar")
        rospy.loginfo("  - /tf")
        rospy.loginfo("Publishing topics:")
        rospy.loginfo("  - /carla/ego_vehicle/fusion/image")

        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            # 定时更新 TF 变换
            if not self.tf_initialized or self.frame_count % 50 == 0:
                self.update_transforms()

            # 融合数据
            fusion_image = self.fuse_data()

            if fusion_image is not None:
                # 发布融合图像
                try:
                    msg = self.bridge.cv2_to_imgmsg(fusion_image, 'bgr8')
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = "ego_vehicle/fusion"
                    self.fusion_pub.publish(msg)

                    self.frame_count += 1
                    if self.frame_count % 30 == 0:
                        rospy.loginfo("Frame {}: projected {} points".format(
                            self.frame_count, self.projected_points))
                except Exception as e:
                    rospy.logerr("Failed to publish fusion image: {}".format(e))

            rate.sleep()


def main():
    rospy.init_node('lidar_camera_fusion', anonymous=True)

    fusion = LidarCameraFusion()

    try:
        fusion.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
