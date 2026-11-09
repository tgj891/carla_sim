#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import message_filters
import tf
import cv2
import numpy as np
import os
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class CarlaToColmapSaver(object):
    """从 CARLA ROS bridge 采集数据，保存为 COLMAP 原生格式 (text)，供 LiteGS/3DGS 训练使用。

    原理：
      在 CARLA 中相机位姿 (TF) 是精确的，无需跑 COLMAP 做 SfM。
      但 3DGS 训练需要 COLMAP 格式的输入，因此本脚本直接利用：
        - 图像 + TF → cameras.txt / images.txt  (位姿 + 内参)
        - 深度图反投影 → points3D.txt            (初始 Gaussian 位置, 带颜色)
    """

    def __init__(self):
        rospy.init_node('carla_to_colmap', anonymous=True)
        self.bridge = CvBridge()

        # --- 输出目录结构 (与 COLMAP 一致) ---
        self.save_dir = "./carla_colmap_data"
        self.image_dir = os.path.join(self.save_dir, "images")
        self.sparse_dir = os.path.join(self.save_dir, "sparse", "0")

        for d in [self.image_dir, self.sparse_dir]:
            if not os.path.exists(d):
                os.makedirs(d)

        self.tf_listener = tf.TransformListener()

        # ---------- 话题订阅 (三路时间同步) ----------
        self.image_sub = message_filters.Subscriber(
            '/carla/ego_vehicle/rgb_main_front/image', Image)
        self.depth_sub = message_filters.Subscriber(
            '/carla/ego_vehicle/depth_front/image', Image)
        self.info_sub = message_filters.Subscriber(
            '/carla/ego_vehicle/rgb_main_front/camera_info', CameraInfo)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub, self.info_sub],
            queue_size=10, slop=0.1)
        self.ts.registerCallback(self.camera_callback)

        # ---------- 状态 ----------
        self.frame_count = 0
        self.camera_id = 1
        self.camera_model = None
        self.image_entries = []

        # 深度图反投影点云累积
        self.accumulated_pts = []         # Nx6 list (x, y, z, r, g, b) 世界坐标系
        self.depth_frame_count = 0
        self.max_depth_frames = 80        # 最多累积 80 帧
        self.depth_stride = 4             # 每隔 4 个像素取一个点 (1/16 像素)
        self.voxel_size = 0.05            # 体素下采样 5cm

        rospy.loginfo("Ready to capture COLMAP format. Saving to: %s", self.save_dir)

    # ==================================================================
    # 工具函数
    # ==================================================================
    @staticmethod
    def rotation_matrix_to_quaternion(R):
        """旋转矩阵 (3x3) → (qw, qx, qy, qz)"""
        q = np.zeros(4)
        tr = R[0, 0] + R[1, 1] + R[2, 2]
        if tr > 0:
            S = np.sqrt(tr + 1.0) * 2
            q[0] = 0.25 * S
            q[1] = (R[2, 1] - R[1, 2]) / S
            q[2] = (R[0, 2] - R[2, 0]) / S
            q[3] = (R[1, 0] - R[0, 1]) / S
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            q[0] = (R[2, 1] - R[1, 2]) / S
            q[1] = 0.25 * S
            q[2] = (R[1, 0] + R[0, 1]) / S
            q[3] = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            q[0] = (R[0, 2] - R[2, 0]) / S
            q[1] = (R[1, 0] + R[0, 1]) / S
            q[2] = 0.25 * S
            q[3] = (R[2, 1] + R[1, 2]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            q[0] = (R[1, 0] - R[0, 1]) / S
            q[1] = (R[0, 2] + R[2, 0]) / S
            q[2] = (R[2, 1] + R[1, 2]) / S
            q[3] = 0.25 * S
        return q

    def _voxel_downsample(self):
        """体素下采样，控制内存"""
        if not self.accumulated_pts:
            return
        cloud = np.array(self.accumulated_pts)        # Nx6
        idx = np.floor(cloud[:, :3] / self.voxel_size).astype(np.int32)
        _, unique_idx = np.unique(idx, axis=0, return_index=True)
        self.accumulated_pts = cloud[unique_idx].tolist()

    # ==================================================================
    # 深度图 → 3D 点云 (反投影 + 着颜色)
    # ==================================================================
    def _depth_to_pointcloud(self, depth_img, rgb_img, fx, fy, cx, cy, R_w2c, t_w2c):
        """将深度图反投影到世界坐标系，并用 RGB 图像着色

        返回:
            list of (x, y, z, r, g, b) 世界坐标系下的点
        """
        h, w = depth_img.shape
        stride = self.depth_stride

        # 生成像素网格
        v, u = np.mgrid[0:h:stride, 0:w:stride]
        u = u.ravel()
        v = v.ravel()

        # 读取深度值 (CARLA 32FC1, 单位米)
        z = depth_img[v, u].astype(np.float64)

        # 过滤无效深度 (0, NaN, Inf, 太大太小)
        valid = (z > 0.1) & (z < 200.0) & np.isfinite(z)
        u, v, z = u[valid], v[valid], z[valid]
        if len(z) == 0:
            return []

        # 反投影到相机坐标系: X_cam = K^{-1} * [u * z, v * z, z]
        x_cam = (u - cx) * z / fx
        y_cam = (v - cy) * z / fy
        # z_cam = z  (已经是深度)

        # 堆叠为 Nx3 相机坐标系点
        pts_cam = np.column_stack((x_cam, y_cam, z))

        # 转换到世界坐标系: X_world = R_w2c^T * (X_cam - t_w2c)
        # 因为 COLMAP w2c: X_cam = R_w2c * X_world + t_w2c
        # => X_world = R_w2c^T * (X_cam - t_w2c)
        R_w2c_T = R_w2c.T
        pts_world = np.dot(pts_cam - t_w2c, R_w2c_T.T)  # (N x 3)

        # 采样 RGB 颜色
        rgb = rgb_img[v, u]  # Nx3 (BGR)
        rgb = rgb[:, ::-1]   # BGR → RGB

        # 合并: Nx6 (x, y, z, r, g, b)
        pts = np.column_stack((pts_world, rgb))

        return pts.tolist()

    # ==================================================================
    # 主回调 (RGB + 深度 + 内参同步)
    # ==================================================================
    def camera_callback(self, img_msg, depth_msg, info_msg):
        try:
            cam_frame = info_msg.header.frame_id
            world_frame = "map"

            # 1. 保存 RGB 图像
            img_cv = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            file_name = "frame_%05d.png" % self.frame_count
            cv2.imwrite(os.path.join(self.image_dir, file_name), img_cv)

            # 2. 获取 TF 位姿
            self.tf_listener.waitForTransform(
                world_frame, cam_frame,
                img_msg.header.stamp, rospy.Duration(5.0))
            (trans, rot) = self.tf_listener.lookupTransform(
                world_frame, cam_frame, img_msg.header.stamp)

            R = tf.transformations.quaternion_matrix(rot)[:3, :3]
            t = np.array(trans)

            # --- COLMAP w2c: X_cam = R_w2c * X_world + t_w2c ---
            R_w2c = R.T
            t_w2c = -np.dot(R.T, t)

            # --- 坐标系: ROS 相机 → OpenCV 相机 ---
            R_ros2cv = np.array([
                [0, -1, 0],
                [0, 0, -1],
                [1, 0, 0]
            ])
            R_w2c_cv = np.dot(R_ros2cv, R_w2c)
            t_w2c_cv = np.dot(R_ros2cv, t_w2c)

            qw, qx, qy, qz = self.rotation_matrix_to_quaternion(R_w2c_cv)

            # 3. 相机内参 (首次)
            if self.camera_model is None:
                fx = info_msg.K[0]
                fy = info_msg.K[4]
                cx = info_msg.K[2]
                cy = info_msg.K[5]
                self.camera_model = {
                    'id': self.camera_id,
                    'model': 'PINHOLE',
                    'width': info_msg.width,
                    'height': info_msg.height,
                    'params': [fx, fy, cx, cy],
                    'fx': fx, 'fy': fy, 'cx': cx, 'cy': cy,
                }
            else:
                fx = self.camera_model['fx']
                fy = self.camera_model['fy']
                cx = self.camera_model['cx']
                cy = self.camera_model['cy']

            # 4. 记录位姿
            image_id = self.frame_count + 1
            self.image_entries.append({
                'id': image_id,
                'qw': qw, 'qx': qx, 'qy': qy, 'qz': qz,
                'tx': t_w2c_cv[0], 'ty': t_w2c_cv[1], 'tz': t_w2c_cv[2],
                'camera_id': self.camera_id,
                'name': file_name,
            })
            self.frame_count += 1

            # 5. 深度图 → 3D 点云累积 (前 max_depth_frames 帧)
            if self.depth_frame_count < self.max_depth_frames:
                depth_img = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
                pts = self._depth_to_pointcloud(
                    depth_img, img_cv,
                    fx, fy, cx, cy,
                    R_w2c_cv, t_w2c_cv)

                if pts:
                    self.accumulated_pts.extend(pts)
                    self.depth_frame_count += 1

                    # 每 20 帧下采样一次
                    if self.depth_frame_count % 20 == 0:
                        self._voxel_downsample()
                        rospy.loginfo(
                            "Depth: %d frames, %d pts (after downsample)",
                            self.depth_frame_count, len(self.accumulated_pts))

            if self.frame_count % 10 == 0:
                rospy.loginfo("Captured %d frames, depth %d/%d",
                              self.frame_count, self.depth_frame_count, self.max_depth_frames)

        except Exception as e:
            rospy.logerr("Camera callback error: %s", str(e))

    # ==================================================================
    # 保存 COLMAP 格式文件
    # ==================================================================
    def save_colmap_format(self):
        # --- cameras.txt ---
        cam_path = os.path.join(self.sparse_dir, "cameras.txt")
        with open(cam_path, 'w') as f:
            f.write("# Camera list with one line of data per camera:\n")
            f.write("#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n")
            f.write("# Number of cameras: 1\n")
            cam = self.camera_model
            params = " ".join(["%.6f" % p for p in cam['params']])
            f.write("%d %s %d %d %s\n" % (
                cam['id'], cam['model'], cam['width'], cam['height'], params))
        rospy.loginfo("Saved %s", cam_path)

        # --- images.txt ---
        img_path = os.path.join(self.sparse_dir, "images.txt")
        with open(img_path, 'w') as f:
            f.write("# Image list with two lines of data per image:\n")
            f.write("#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME\n")
            f.write("#   POINTS2D[] as (X, Y, POINT3D_ID)\n")
            f.write("# Number of images: %d, mean observations per image: 0\n" % len(self.image_entries))
            for e in self.image_entries:
                f.write("%d %.10f %.10f %.10f %.10f %.10f %.10f %.10f %d %s\n" % (
                    e['id'], e['qw'], e['qx'], e['qy'], e['qz'],
                    e['tx'], e['ty'], e['tz'],
                    e['camera_id'], e['name']))
                f.write("\n")
        rospy.loginfo("Saved %s", img_path)

        # --- points3D.txt (从深度图反投影点云生成) ---
        self._voxel_downsample()
        num_pts = len(self.accumulated_pts)
        pts_path = os.path.join(self.sparse_dir, "points3D.txt")
        with open(pts_path, 'w') as f:
            f.write("# 3D point list with one line of data per point:\n")
            f.write("#   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)\n")
            f.write("# Number of points: %d, mean track length: 0\n" % num_pts)
            for i, pt in enumerate(self.accumulated_pts):
                # pt: (x, y, z, r, g, b)
                f.write("%d %.6f %.6f %.6f %d %d %d 0.0\n" % (
                    i + 1, pt[0], pt[1], pt[2],
                    int(pt[3]), int(pt[4]), int(pt[5])))
        rospy.loginfo("Saved %s with %d points", pts_path, num_pts)

    # ==================================================================
    # 主循环
    # ==================================================================
    def run(self):
        rospy.spin()
        if self.camera_model is not None:
            self.save_colmap_format()
            rospy.loginfo("All data saved in COLMAP format to: %s", self.save_dir)
        else:
            rospy.logwarn("No data captured, nothing saved.")


if __name__ == '__main__':
    try:
        collector = CarlaToColmapSaver()
        collector.run()
    except rospy.ROSInterruptException:
        pass