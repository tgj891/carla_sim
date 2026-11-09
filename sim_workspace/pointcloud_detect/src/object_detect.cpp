#include "object_detect.h"

ObjectDetect::ObjectDetect() {
    // 默认聚类参数
    cluster_tolerance_ = 0.5;   // 0.5m 内视为同一障碍物
    min_cluster_size_ = 10;     // 少于 20 点视为噪点
    max_cluster_size_ = 30000;  // 超过视为巨大物体(如墙面), 忽略
}

bool ObjectDetect::Init() {
    std::cout << "ObjectDetect init done, cluster_tolerance=" << cluster_tolerance_
              << " min_cluster_size=" << min_cluster_size_
              << " max_cluster_size=" << max_cluster_size_ << std::endl;
    return true;
}

void ObjectDetect::ClusterObstacles(
        const pcl::PointCloud<PointType>::Ptr& non_ground_cloud,
        std::vector<std::pair<pcl::PointCloud<PointType>::Ptr, pcl::PointCloud<PointType>::Ptr>>& clusters) {
    clusters.clear();
    if (non_ground_cloud == nullptr || non_ground_cloud->empty()) {
        std::cout << "Non-ground cloud is empty, skip clustering" << std::endl;
        return;
    }

    // 1. 构建 kd-tree
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
    tree->setInputCloud(non_ground_cloud);

    // 2. 欧氏聚类提取
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(non_ground_cloud);
    ec.extract(cluster_indices);

    std::cout << "Clustering done, found " << cluster_indices.size() << " obstacles" << std::endl;

    // 3. 每个簇单独输出, 并用不同颜色着色 (颜色由调用方通过 road_detect 的 viewer 展示)
    srand(time(NULL));
    for (size_t i = 0; i < cluster_indices.size(); i++) {
        pcl::PointCloud<PointType>::Ptr cluster(new pcl::PointCloud<PointType>());
        uint8_t r = rand() % 256;
        uint8_t g = rand() % 256;
        uint8_t b = rand() % 256;
        for (int idx : cluster_indices[i].indices) {
            PointType pt = non_ground_cloud->points[idx];
            pt.r = r;
            pt.g = g;
            pt.b = b;
            cluster->points.push_back(pt);
        }
        cluster->width = cluster->points.size();
        cluster->height = 1;

        pcl::ConvexHull<PointType> ch;
        pcl::PointCloud<PointType>::Ptr hull(new pcl::PointCloud<PointType>);
        ch.setInputCloud(cluster);
        ch.setDimension(2);
        ch.reconstruct(*hull);

        clusters.push_back(std::make_pair(cluster, hull));
        // std::cout << "  Cluster[" << i << "]: " << cluster->points.size() << " points" << std::endl;
    }
}
