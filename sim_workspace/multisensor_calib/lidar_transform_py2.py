#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
多激光雷达点云变换脚本 (Python 2.7)
订阅 front_lidar, leftback_lidar, rightback_lidar 点云，
应用外参变换后发布。
"""

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import tf.transformations as tft

# ========= 配置参数 =========
# 每个激光雷达的外参（平移 + 旋转）
# 平移单位：米，旋转单位：度
LIDAR_CONFIG = {
    'front_lidar': {
        'topic_in': '/carla/ego_vehicle/front_lidar',
        'topic_out': '/carla/ego_vehicle/front_lidar_transformed',
        'translation': [0.0, 0.0, 0.0],  # x, y, z
        'rotation': [0.0, 3.0, 0.0]      # roll, pitch, yaw (角度)
    },
    'leftback_lidar': {
        'topic_in': '/carla/ego_vehicle/leftback_lidar',
        'topic_out': '/carla/ego_vehicle/leftback_lidar_transformed',
        'translation': [0.2, 0.0, 0.0],
        'rotation': [0.0, 0.0, 3.0]
    },
    'rightback_lidar': {
        'topic_in': '/carla/ego_vehicle/rightback_lidar',
        'topic_out': '/carla/ego_vehicle/rightback_lidar_transformed',
        'translation': [0.0, 0.1, 0.0],
        'rotation': [2.0, 0.0, 0.0]
    }
}

# ========= 全局变量 =========
publishers = {}
transform_matrices = {}

def create_transform_matrix(translation, rotation_deg):
    """
    创建 4x4 变换矩阵
    translation: [x, y, z]
    rotation_deg: [roll, pitch, yaw] in degrees
    """
    # 角度转弧度
    roll = np.radians(rotation_deg[0])
    pitch = np.radians(rotation_deg[1])
    yaw = np.radians(rotation_deg[2])
    
    # 创建旋转矩阵（ZYX 顺序）
    R = tft.euler_matrix(roll, pitch, yaw)
    
    # 添加平移
    T = np.eye(4)
    T[:3, :3] = R[:3, :3]
    T[0, 3] = translation[0]
    T[1, 3] = translation[1]
    T[2, 3] = translation[2]
    
    return T

def pointcloud2_to_numpy(cloud_msg):
    """
    将 PointCloud2 消息转换为 numpy 数组
    返回: points (N, 3) 或 (N, 4) 如果包含 intensity
    """
    points_list = []
    for point in point_cloud2.read_points(cloud_msg, skip_nans=True):
        points_list.append(point)
    
    if len(points_list) == 0:
        return np.array([])
    
    # 转换为 numpy 数组
    points_array = np.array(points_list)
    return points_array

def numpy_to_pointcloud2(points, header):
    """
    将 numpy 数组转换为 PointCloud2 消息
    points: (N, 3) 或 (N, 4)
    """
    # 创建 PointCloud2 消息
    cloud_out = PointCloud2()
    cloud_out.header = header
    
    # 设置字段
    fields = []
    fields.append(point_cloud2.PointField(name='x', offset=0, 
                                          datatype=point_cloud2.PointField.FLOAT32, count=1))
    fields.append(point_cloud2.PointField(name='y', offset=4, 
                                          datatype=point_cloud2.PointField.FLOAT32, count=1))
    fields.append(point_cloud2.PointField(name='z', offset=8, 
                                          datatype=point_cloud2.PointField.FLOAT32, count=1))
    
    if points.shape[1] == 4:
        fields.append(point_cloud2.PointField(name='intensity', offset=12, 
                                              datatype=point_cloud2.PointField.FLOAT32, count=1))
        point_step = 16
    else:
        point_step = 12
    
    cloud_out.fields = fields
    cloud_out.point_step = point_step
    cloud_out.height = 1
    cloud_out.width = len(points)
    cloud_out.is_bigendian = False
    cloud_out.is_dense = True
    
    # 转换数据
    data = points.astype(np.float32).tobytes()
    cloud_out.data = data
    cloud_out.row_step = point_step * cloud_out.width
    
    return cloud_out

def lidar_callback(msg, lidar_name):
    """
    激光雷达点云回调函数
    """
    if lidar_name not in transform_matrices:
        rospy.logerr("Unknown lidar: %s" % lidar_name)
        return
    
    # 获取变换矩阵
    T = transform_matrices[lidar_name]
    
    # 转换点云为 numpy 数组
    points = pointcloud2_to_numpy(msg)
    
    if len(points) == 0:
        rospy.logwarn("Empty point cloud from %s" % lidar_name)
        return
    
    # 分离 xyz 和 intensity
    xyz = points[:, :3]
    has_intensity = points.shape[1] == 4
    if has_intensity:
        intensity = points[:, 3:4]
    
    # 添加齐次坐标
    ones = np.ones((xyz.shape[0], 1))
    xyz_homogeneous = np.hstack([xyz, ones])  # (N, 4)
    
    # 应用变换: P' = T * P
    # xyz_homogeneous: (N, 4), T: (4, 4)
    xyz_transformed = np.dot(xyz_homogeneous, T.T)[:, :3]
    
    # 合并结果
    if has_intensity:
        points_transformed = np.hstack([xyz_transformed, intensity])
    else:
        points_transformed = xyz_transformed
    
    # 转换回 PointCloud2
    cloud_out = numpy_to_pointcloud2(points_transformed, msg.header)
    
    # 发布
    publishers[lidar_name].publish(cloud_out)
    
    rospy.logdebug("Published %d points from %s" % (len(points), lidar_name))

def main():
    rospy.init_node('lidar_transform_node')
    
    rospy.loginfo("Starting lidar transform node...")
    
    # 为每个激光雷达创建发布者和变换矩阵
    for lidar_name, config in LIDAR_CONFIG.items():
        # 创建发布者
        publishers[lidar_name] = rospy.Publisher(
            config['topic_out'], 
            PointCloud2, 
            queue_size=10
        )
        
        # 创建变换矩阵
        transform_matrices[lidar_name] = create_transform_matrix(
            config['translation'],
            config['rotation']
        )
        
        rospy.loginfo("Lidar '%s': translation=%s, rotation=%s" % 
                      (lidar_name, config['translation'], config['rotation']))
        rospy.loginfo("  Subscribe: %s" % config['topic_in'])
        rospy.loginfo("  Publish:   %s" % config['topic_out'])
    
    # 订阅每个激光雷达的点云
    subscribers = []
    for lidar_name, config in LIDAR_CONFIG.items():
        sub = rospy.Subscriber(
            config['topic_in'],
            PointCloud2,
            lidar_callback,
            callback_args=lidar_name,
            queue_size=10
        )
        subscribers.append(sub)
    
    rospy.loginfo("Lidar transform node started. Waiting for point clouds...")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
