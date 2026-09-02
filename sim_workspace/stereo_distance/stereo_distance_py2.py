#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
双目视差图与点云生成脚本
功能：订阅 CARLA 左右相机图像，生成双目视差图和点云

订阅话题：
  - /carla/ego_vehicle/rgb_stereo_left/camera_info
  - /carla/ego_vehicle/rgb_stereo_left/image
  - /carla/ego_vehicle/rgb_stereo_right/camera_info
  - /carla/ego_vehicle/rgb_stereo_right/image

显示：
  - 左相机灰度图
  - 视差图（彩色热力图）
  - 点云（Open3D 或 matplotlib 3D 散点图）
"""

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
import threading
import struct
import math


class StereoDepthEstimator:
    """双目立体深度估计类"""

    def __init__(self):
        rospy.init_node("stereo_depth_estimator", anonymous=True)

        # ROS 接口
        self.bridge = CvBridge()

        # 相机内参
        self.left_K = None       # 左相机内参矩阵 (3x3)
        self.right_K = None      # 右相机内参矩阵 (3x3)
        self.left_D = None       # 左相机畸变系数
        self.right_D = None      # 右相机畸变系数
        self.img_w = None
        self.img_h = None
        self.baseline = 0.0      # 双目基线长度 (m)

        # 图像缓存
        self.left_image = None
        self.right_image = None
        self.left_lock = threading.Lock()
        self.right_lock = threading.Lock()

        # 立体校正映射表（延迟初始化）
        self.rect_maps_initialized = False
        self.left_map1 = None
        self.left_map2 = None
        self.right_map1 = None
        self.right_map2 = None
        self.Q = None             # 重投影矩阵

        # SGBM 立体匹配器
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=128,
            blockSize=9,
            P1=8 * 3 * 9 ** 2,
            P2=32 * 3 * 9 ** 2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )

        # 点云发布
        self.pointcloud_pub = rospy.Publisher(
            '/carla/ego_vehicle/stereo/pointcloud', PointCloud2, queue_size=1
        )

        # 订阅话题
        self.left_info_sub = rospy.Subscriber(
            '/carla/ego_vehicle/rgb_stereo_left/camera_info', CameraInfo,
            self.left_info_callback, queue_size=1
        )
        self.right_info_sub = rospy.Subscriber(
            '/carla/ego_vehicle/rgb_stereo_right/camera_info', CameraInfo,
            self.right_info_callback, queue_size=1
        )
        self.left_image_sub = rospy.Subscriber(
            '/carla/ego_vehicle/rgb_stereo_left/image', Image,
            self.left_image_callback, queue_size=1
        )
        self.right_image_sub = rospy.Subscriber(
            '/carla/ego_vehicle/rgb_stereo_right/image', Image,
            self.right_image_callback, queue_size=1
        )

        # 统计
        self.frame_count = 0
        self.fps_timer = rospy.Time.now()

        # 显示帧缓存（主线程读取）
        self.display_frame = None
        self.display_lock = threading.Lock()

        rospy.loginfo("StereoDepthEstimator initialized, waiting for camera topics...")

    # --------------------------------------------------------------------------
    # 相机内参回调
    # --------------------------------------------------------------------------
    def left_info_callback(self, msg):
        if self.left_K is not None:
            return
        self.left_K = np.array(msg.K).reshape(3, 3)
        self.left_D = np.array(msg.D)
        self.img_w = msg.width
        self.img_h = msg.height
        rospy.loginfo("Left camera info received: %dx%d", self.img_w, self.img_h)
        self.try_init_rectification()

    def right_info_callback(self, msg):
        if self.right_K is not None:
            return
        self.right_K = np.array(msg.K).reshape(3, 3)
        self.right_D = np.array(msg.D)
        rospy.loginfo("Right camera info received")
        self.try_init_rectification()

    def try_init_rectification(self):
        """当左右相机内参都收到后，初始化立体校正"""
        if self.rect_maps_initialized:
            return
        if self.left_K is None or self.right_K is None:
            return
        if self.img_w is None or self.img_h is None:
            return

        # 基线 = 左右相机光心间距（假设左右相机水平排列，且内参相同）
        # 从右相机内参的平移部分推算基线
        # 若无外参用默认值 0.24m（CARLA 默认 stereo 基线）
        fx = self.left_K[0, 0]
        cx = self.left_K[0, 2]
        # 用右相机主点偏移估算基线: baseline = |cx_left - cx_right| / fx * 某种近似
        # 更准确需要外参，这里用默认值
        self.baseline = 0.4

        rospy.loginfo("Stereo baseline: %.3f m", self.baseline)
        rospy.loginfo("Left K:\n%s", self.left_K)
        rospy.loginfo("Right K:\n%s", self.right_K)

        # 立体校正 - 使用 Bouguet 算法
        # 构造单位旋转和平移（假设已校正或相机接近水平）
        R = np.eye(3, dtype=np.float64)
        T = np.array([[-self.baseline], [0.0], [0.0]], dtype=np.float64)

        R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
            self.left_K, self.left_D,
            self.right_K, self.right_D,
            (self.img_w, self.img_h),
            R, T,
            alpha=0  # 裁剪黑边
        )
        self.Q = Q

        self.left_map1, self.left_map2 = cv2.initUndistortRectifyMap(
            self.left_K, self.left_D, R1, P1,
            (self.img_w, self.img_h), cv2.CV_16SC2
        )
        self.right_map1, self.right_map2 = cv2.initUndistortRectifyMap(
            self.right_K, self.right_D, R2, P2,
            (self.img_w, self.img_h), cv2.CV_16SC2
        )

        self.rect_maps_initialized = True
        rospy.loginfo("Stereo rectification initialized (Q matrix ready)")

    # --------------------------------------------------------------------------
    # 图像回调
    # --------------------------------------------------------------------------
    def left_image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn("Left image conversion failed: %s", e)
            return
        with self.left_lock:
            self.left_image = img

        self.try_process_frame()

    def right_image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn("Right image conversion failed: %s", e)
            return
        with self.right_lock:
            self.right_image = img

        self.try_process_frame()

    # --------------------------------------------------------------------------
    # 双目立体匹配处理
    # --------------------------------------------------------------------------
    def try_process_frame(self):
        """当左右图像都就绪时，执行一次立体匹配"""
        with self.left_lock:
            left = self.left_image
        with self.right_lock:
            right = self.right_image

        if left is None or right is None:
            return
        if not self.rect_maps_initialized:
            return

        # 灰度化
        gray_left = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)

        # 立体校正
        rect_left = cv2.remap(gray_left, self.left_map1, self.left_map2,
                              cv2.INTER_LINEAR)
        rect_right = cv2.remap(gray_right, self.right_map1, self.right_map2,
                               cv2.INTER_LINEAR)

        # 计算视差图
        disparity = np.abs(self.stereo.compute(rect_left, rect_right).astype(np.float32) / 16.0)

        # 过滤无效视差（保留负值，因为右相机可能在左侧）

        # 深度图: depth = fx * baseline / |disparity|
        fx = self.left_K[0, 0]
        depth = np.zeros_like(disparity)
        valid_mask = np.abs(disparity) > 0.1
        depth[valid_mask] = fx * self.baseline / np.abs(disparity[valid_mask])

        # 限制深度范围
        depth = np.clip(depth, 0.0, 100.0)

        # 生成点云
        self.generate_pointcloud(disparity, depth, rect_left)

        # 可视化
        self.visualize(left, rect_left, rect_right, disparity, depth)

        self.frame_count += 1

    def generate_pointcloud(self, disparity, depth, rect_left):
        """从深度图生成点云"""
        if self.Q is None:
            return

        # 使用 reprojectImageTo3D 生成点云（用绝对值确保 Z 为正）
        points_3d = cv2.reprojectImageTo3D(np.abs(disparity), self.Q)

        # 过滤无效点
        valid = (np.abs(disparity) > 0.1) & (depth < 80.0) & (depth > 0.1)
        points = points_3d[valid]  # shape (N, 3)
        colors = rect_left[valid]  # 灰度值

        if len(points) == 0:
            if self.frame_count % 30 == 0:
                rospy.logwarn("Pointcloud: 0 valid points (valid_mask=%d)" % valid.sum())
            return

        # 过滤远处的稀疏点
        mask = (np.abs(points[:, 2]) < 80.0) & (np.abs(points[:, 2]) > 0.1)
        points = points[mask]
        colors = colors[mask]

        if len(points) == 0:
            if self.frame_count % 30 == 0:
                rospy.logwarn("Pointcloud: all points filtered out (depth range [%.1f, %.1f])" % (
                    points_3d[valid][:, 2].min(), points_3d[valid][:, 2].max()))
            return

        # 发布点云
        if self.frame_count % 10 == 0:
            rospy.loginfo("Pointcloud: %d points, depth range [%.1f, %.1f] m",
                          len(points), points[:, 2].min(), points[:, 2].max())
        
        # 发布点云
        self.publish_pointcloud(points)
      
    def publish_pointcloud(self, points_xyz, frame_id="stereo_pointcloud"):
        """
        :param pub: rospy.Publisher 对象
        :param points_xyz: N×3 numpy array (X,Y,Z)，单位米
        :param frame_id: 点云所属坐标系
        """
        # 过滤无效点：去除nan/inf
        mask = np.isfinite(points_xyz).all(axis=1)
        pts_valid = points_xyz[mask]

        # 构造PointCloud2
        cloud_msg = pc2.create_cloud_xyz32(
            header=rospy.Header(
                frame_id=frame_id,
                stamp=rospy.Time.now()
            ),
            points=pts_valid
        )
        self.pointcloud_pub.publish(cloud_msg)
        
    def _render_pointcloud_topdown(self):
        """渲染点云俯视图（X-Z 投影，Y 为高度）"""
        points, colors = self.pointcloud_data
        if len(points) == 0:
            return None

        # 俯视图：X 轴（左右）和 Z 轴（深度）
        canvas = np.zeros((self.img_h, self.img_w, 3), dtype=np.uint8)

        # 坐标映射：X -> 列，Z -> 行（远处在上方）
        x = points[:, 0]  # 左右
        z = points[:, 2]  # 深度

        # 缩放到画布
        x_min, x_max = -30.0, 30.0
        z_min, z_max = 0.0, 80.0

        col = ((x - x_min) / (x_max - x_min) * (self.img_w - 1)).astype(np.int32)
        row = ((z - z_min) / (z_max - z_min) * (self.img_h - 1)).astype(np.int32)

        valid = (col >= 0) & (col < self.img_w) & (row >= 0) & (row < self.img_h)
        col, row = col[valid], row[valid]
        c = colors[valid]

        # 颜色映射：按深度着色
        for i in range(len(col)):
            depth_color = int(min(255, z[valid][i] / 80.0 * 255))
            cv2.circle(canvas, (col[i], row[i]), 1,
                       (0, 255 - depth_color, depth_color), -1)

        cv2.putText(canvas, "PointCloud Top-Down (X-Z)", (5, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        return canvas

    def visualize(self, original_left, rect_left, rect_right, disparity, depth):
        """可视化结果"""
        # 视差图彩色热力图
        disparity_norm = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        disparity_color = cv2.applyColorMap(disparity_norm, cv2.COLORMAP_JET)

        # 深度图彩色热力图
        depth_valid = depth.copy()
        depth_valid[depth_valid <= 0] = 0
        depth_norm = cv2.normalize(depth_valid, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_HOT)

        # 拼接显示
        # 第一行：原图 | 校正左图 | 校正右图
        gray_left_3ch = cv2.cvtColor(rect_left, cv2.COLOR_GRAY2BGR)
        gray_right_3ch = cv2.cvtColor(rect_right, cv2.COLOR_GRAY2BGR)

        row1 = np.hstack([original_left, gray_left_3ch, gray_right_3ch])

        # 第二行：视差图 | 深度图
        row2 = np.hstack([disparity_color, depth_color])

        # 确保两行宽度一致
        if row1.shape[1] != row2.shape[1]:
            target_w = max(row1.shape[1], row2.shape[1])
            if row1.shape[1] < target_w:
                pad = np.zeros((row1.shape[0], target_w - row1.shape[1], 3), dtype=np.uint8)
                row1 = np.hstack([row1, pad])
            if row2.shape[1] < target_w:
                pad = np.zeros((row2.shape[0], target_w - row2.shape[1], 3), dtype=np.uint8)
                row2 = np.hstack([row2, pad])

        display = np.vstack([row1, row2])

        # 添加文字标注
        cv2.putText(display, "Original Left", (5, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(display, "Rectified Left", (self.img_w + 5, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(display, "Rectified Right", (self.img_w * 2 + 5, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(display, "Disparity Map", (5, self.img_h + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(display, "Depth Map", (self.img_w + 5, self.img_h + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # FPS
        now = rospy.Time.now()
        dt = (now - self.fps_timer).to_sec()
        if dt > 2.0:
            fps = self.frame_count / dt
            self.frame_count = 0
            self.fps_timer = now
            rospy.loginfo("Stereo FPS: %.1f", fps)

        # 存入缓存供主线程显示
        with self.display_lock:
            self.display_frame = display


def main():
    estimator = StereoDepthEstimator()
    rate = rospy.Rate(30)  # 30Hz 显示刷新
    try:
        while not rospy.is_shutdown():
            with estimator.display_lock:
                frame = estimator.display_frame
            if frame is not None:
                cv2.imshow("Stereo Depth Estimation", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == 27:  # ESC
                    break
            else:
                cv2.waitKey(10)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()