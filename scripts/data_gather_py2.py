#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
标签和ID
ID	类别名称	中文释义	RGB 颜色
0	Unlabeled	未标注 / 背景	(0, 0, 0)
1	Building	建筑物	(70, 70, 70)
2	Fence	围栏	(100, 40, 40)
3	Other	其他物体	(55, 90, 80)
4	Pedestrian	行人	(220, 20, 60)
5	Pole	电线杆 / 立柱	(153, 153, 153)
6	RoadLine	车道标线	(157, 234, 50)
7	Road	路面	(128, 64, 128)
8	Sidewalk	人行道	(244, 35, 232)
9	Vegetation	植被（树木、草地）	(107, 142, 35)
10	Vehicles	机动车（汽车、卡车、巴士）	(0, 0, 142)
11	Wall	墙体	(102, 102, 156)
12	TrafficSign	交通标志牌	(220, 220, 0)
13	Sky	天空	(70, 130, 180)
14	Ground	地面（非铺装土路）	(81, 0, 81)
15	Bridge	桥梁	(150, 100, 100)
16	RailTrack	铁轨	(230, 150, 140)
17	GuardRail	护栏	(180, 165, 180)
18	TrafficLight	交通信号灯	(250, 170, 30)
19	Static	静态杂物	(110, 190, 160)
20	Dynamic	动态杂物	(170, 120, 50)
21	Water	水体	(45, 60, 150)
22	Terrain	地形 / 荒地	(145, 170, 100)

CARLA 语义分割数据采集脚本
采集 RGB 图像和语义标签图像，用于语义分割模型训练

采集话题：
  RGB:        /carla/ego_vehicle/rgb_front/image
              /carla/ego_vehicle/rgb_left/image
  语义标签:   /carla/ego_vehicle/semantic_segmentation_front/image
              /carla/ego_vehicle/semantic_segmentation_left/image

输出目录结构：
  output_dir/
    front/
      rgb/         # 前视 RGB 图像
      semantic/    # 前视语义标签（灰度图，像素值=类别ID）
    left/
      rgb/         # 左视 RGB 图像
      semantic/    # 左视语义标签
"""

import rospy
import os
import threading
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# 类别映射：CARLA ID → 训练 ID（只保留 6 类 + 背景）
CARLA_TO_TRAIN = {
    4:  5,   # Pedestrian → 行人
    5:  6,   # Pole       → 电线杆/立柱
    6:  2,   # RoadLine   → 车道标线
    7:  1,   # Road       → 路面
    10: 4,   # Vehicles   → 机动车
    14: 3,   # Ground     → 地面/非铺装土路
}

# CARLA 颜色 → 标签 ID 映射（BGR 格式，ROS Bridge 着色后的像素值）
CARLA_COLOR_TO_TAG = {
    (0, 0, 0):       0,   # Unlabeled
    (70, 70, 70):    1,   # Building
    (40, 40, 100):   2,   # Fence
    (80, 90, 55):    3,   # Other
    (60, 20, 220):   4,   # Pedestrian
    (153, 153, 153): 5,   # Pole
    (50, 234, 157):  6,   # RoadLine
    (128, 64, 128):  7,   # Road
    (232, 35, 244):  8,   # Sidewalk
    (35, 142, 107):  9,   # Vegetation
    (142, 0, 0):     10,  # Vehicles
    (156, 102, 102): 11,  # Wall
    (0, 220, 220):   12,  # TrafficSign
    (180, 130, 70):  13,  # Sky
    (81, 0, 81):     14,  # Ground
    (100, 100, 150): 15,  # Bridge
    (140, 150, 230): 16,  # RailTrack
    (180, 165, 180): 17,  # GuardRail
    (30, 170, 250):  18,  # TrafficLight
    (160, 190, 110): 19,  # Static
    (50, 120, 170):  20,  # Dynamic
    (150, 60, 45):   21,  # Water
    (100, 170, 145): 22,  # Terrain
}

# 训练类别名称
TRAIN_CLASSES = {
    0: "Background",
    1: "Road",
    2: "RoadLine",
    3: "Ground",
    4: "Vehicles",
    5: "Pedestrian",
    6: "Pole",
}

# 可视化 RGB 颜色（BGR 格式，方便 cv2 使用）
TRAIN_COLORS = {
    0: (0, 0, 0),         # Background → 黑色
    1: (128, 64, 128),    # Road       → 紫色
    2: (50, 234, 157),    # RoadLine   → 绿色
    3: (81, 0, 81),       # Ground     → 深紫
    4: (142, 0, 0),       # Vehicles   → 蓝色
    5: (60, 20, 220),     # Pedestrian → 红色
    6: (153, 153, 153),   # Pole       → 灰色
}


class DataGatherer:
    """语义分割数据采集器"""

    def __init__(self):
        rospy.init_node("data_gatherer", anonymous=True)

        # 参数
        self.output_dir = rospy.get_param("~output_dir",
                                          os.path.expanduser("/opt/carla_ws/datasets/carla_semantic_dataset"))
        self.save_interval = rospy.get_param("~save_interval", 0.5)  # 秒
        self.max_frames = rospy.get_param("~max_frames", 0)  # 0=无限

        # 相机方向
        self.cameras = ["front", "left"]

        # 图像缓存（线程安全）
        self.bridge = CvBridge()
        self.rgb_images = {}     # {cam: np.ndarray}
        self.semantic_images = {}  # {cam: np.ndarray}
        self.lock = threading.Lock()

        # 统计
        self.saved_count = {cam: 0 for cam in self.cameras}
        self.frame_idx = {cam: 0 for cam in self.cameras}

        # 创建输出目录
        self._create_dirs()

        # 订阅话题
        self._subscribe_topics()

        rospy.loginfo("DataGatherer initialized, output: %s", self.output_dir)
        rospy.loginfo("Save interval: %.1fs, max_frames: %d", self.save_interval, self.max_frames)

    def _create_dirs(self):
        """创建输出目录结构"""
        for cam in self.cameras:
            for sub in ["rgb", "semantic", "semantic_rgb"]:
                d = os.path.join(self.output_dir, cam, sub)
                if not os.path.exists(d):
                    os.makedirs(d)
                    rospy.loginfo("Created dir: %s", d)

    def _subscribe_topics(self):
        """订阅所有相机话题"""
        self.rgb_subs = {}
        self.semantic_subs = {}

        for cam in self.cameras:
            # RGB 图像
            rgb_topic = "/carla/ego_vehicle/rgb_{}/image".format(cam)
            self.rgb_subs[cam] = rospy.Subscriber(
                rgb_topic, Image,
                lambda msg, c=cam: self._rgb_callback(msg, c),
                queue_size=1
            )

            # 语义分割图像
            semantic_topic = "/carla/ego_vehicle/semantic_segmentation_{}/image".format(cam)
            self.semantic_subs[cam] = rospy.Subscriber(
                semantic_topic, Image,
                lambda msg, c=cam: self._semantic_callback(msg, c),
                queue_size=1
            )

            rospy.loginfo("Subscribed: %s", rgb_topic)
            rospy.loginfo("Subscribed: %s", semantic_topic)

    # --------------------------------------------------------------------------
    # 回调
    # --------------------------------------------------------------------------
    def _rgb_callback(self, msg, camera):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn("[%s] RGB conversion failed: %s", camera, e)
            return
        with self.lock:
            self.rgb_images[camera] = img

    def _semantic_callback(self, msg, camera):
        """
        CARLA 语义分割图像：bgra8 编码，已着色
        需要从颜色反查标签 ID
        """
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logwarn("[%s] Semantic conversion failed: %s", camera, e)
            return

        # 取 BGR 三通道（去掉 Alpha）
        semantic_bgr = img[:, :, :3].copy()

        # 诊断
        if not hasattr(self, '_semantic_debug_printed'):
            self._semantic_debug_printed = set()
        if camera not in self._semantic_debug_printed:
            self._semantic_debug_printed.add(camera)
            rospy.loginfo("[%s] encoding=%s, shape=%s", camera, msg.encoding, img.shape)

        with self.lock:
            self.semantic_images[camera] = semantic_bgr

    # --------------------------------------------------------------------------
    # 标签重映射
    # --------------------------------------------------------------------------
    def _color_to_label(self, img_bgr):
        """
        将 CARLA 着色语义图（BGR）转为标签 ID 图
        img_bgr: numpy uint8 数组 (H, W, 3)，BGR 格式
        返回: numpy uint8 数组 (H, W)，像素值 = CARLA class ID (0-22)
        """
        label = np.zeros(img_bgr.shape[:2], dtype=np.uint8)
        for (b, g, r), tag_id in CARLA_COLOR_TO_TAG.items():
            mask = (img_bgr[:, :, 0] == b) & (img_bgr[:, :, 1] == g) & (img_bgr[:, :, 2] == r)
            label[mask] = tag_id
        return label

    def _remap_labels(self, carla_label):
        """
        将 CARLA 语义标签重映射为训练类别
        carla_label: numpy uint8 数组，像素值 = CARLA class ID (0-22)
        返回: numpy uint8 数组，像素值 = 训练 class ID (0-6)
        """
        remapped = np.zeros_like(carla_label, dtype=np.uint8)
        for carla_id, train_id in CARLA_TO_TRAIN.items():
            remapped[carla_label == carla_id] = train_id
        return remapped

    def _label_to_color(self, label):
        """
        将训练标签图（0-6）转换为 RGB 彩色图，方便查看
        label: numpy uint8 数组 (H, W)，像素值 = 训练 class ID
        返回: numpy uint8 数组 (H, W, 3)，BGR 格式
        """
        h, w = label.shape
        color = np.zeros((h, w, 3), dtype=np.uint8)
        for train_id, bgr in TRAIN_COLORS.items():
            color[label == train_id] = bgr
        return color

    # --------------------------------------------------------------------------
    # 保存
    # --------------------------------------------------------------------------
    def try_save(self, event=None):
        """定时保存已配对的 RGB + 语义标签"""
        for cam in self.cameras:
            # 检查是否达到最大帧数
            if self.max_frames > 0 and self.saved_count[cam] >= self.max_frames:
                continue

            with self.lock:
                rgb = self.rgb_images.get(cam)
                semantic = self.semantic_images.get(cam)

            if rgb is None or semantic is None:
                continue

            # 确保尺寸一致
            if rgb.shape[:2] != semantic.shape[:2]:
                rospy.logwarn("[%s] Size mismatch: rgb=%s, semantic=%s",
                              cam, rgb.shape, semantic.shape)
                continue

            idx = self.frame_idx[cam]
            self.frame_idx[cam] += 1

            # 文件名
            fname = "{:08d}.png".format(idx)
            rgb_path = os.path.join(self.output_dir, cam, "rgb", fname)
            semantic_path = os.path.join(self.output_dir, cam, "semantic", fname)
            semantic_rgb_path = os.path.join(self.output_dir, cam, "semantic_rgb", fname)

            # 颜色 → CARLA 标签 ID
            semantic = self._color_to_label(semantic)

            # 重映射语义标签为训练类别
            semantic = self._remap_labels(semantic)

            # 首次保存时打印标签统计
            if self.saved_count[cam] == 0:
                unique_raw = np.unique(self.semantic_images[cam])
                unique_remap = np.unique(semantic)
                nonzero_pct = float(np.count_nonzero(semantic)) / semantic.size * 100
                rospy.loginfo("[%s] First save: raw unique=%s, remapped unique=%s, nonzero=%.1f%%",
                              cam, unique_raw[:30], unique_remap, nonzero_pct)

            # 生成可视化 RGB 语义图
            semantic_rgb = self._label_to_color(semantic)

            # 保存
            cv2.imwrite(rgb_path, rgb)
            cv2.imwrite(semantic_path, semantic)
            cv2.imwrite(semantic_rgb_path, semantic_rgb)

            self.saved_count[cam] += 1

            if self.saved_count[cam] % 50 == 0:
                rospy.loginfo("[%s] Saved %d frames (latest: %s)",
                              cam, self.saved_count[cam], fname)

    def run(self):
        """主循环"""
        # 使用定时器按间隔保存
        save_timer = rospy.Timer(
            rospy.Duration(self.save_interval),
            self.try_save
        )

        rospy.loginfo("Data gathering started. Press Ctrl+C to stop.")
        rospy.spin()

        # 清理
        save_timer.shutdown()
        rospy.loginfo("Done. Total saved: front=%d, left=%d",
                      self.saved_count["front"], self.saved_count["left"])


def main():
    try:
        gatherer = DataGatherer()
        gatherer.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()