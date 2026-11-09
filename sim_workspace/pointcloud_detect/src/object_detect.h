#pragma once

#include "common.h"

class ObjectDetect {
public:
    ObjectDetect();
    virtual ~ObjectDetect() {
    }
    bool Init();

    // 输入非地面点云, 进行欧氏聚类, 输出每个障碍物簇
    void ClusterObstacles(const pcl::PointCloud<PointType>::Ptr& non_ground_cloud,
                          std::vector<std::pair<pcl::PointCloud<PointType>::Ptr, pcl::PointCloud<PointType>::Ptr>>& clusters);

private:
    // 聚类参数
    double cluster_tolerance_;  // 聚类容差 (m), 两点距离小于该值视为同一簇
    int min_cluster_size_;      // 最小簇点数, 过滤噪点
    int max_cluster_size_;      // 最大簇点数, 过滤巨大物体(如整面墙)
};
