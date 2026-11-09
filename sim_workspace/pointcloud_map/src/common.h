#pragma once

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>

#include <iostream>
#include <csignal>
#include <mutex>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <unordered_map>
#include <tuple>
#include <std_msgs/Float32.h>

using namespace std;
typedef pcl::PointXYZI PointType;
typedef pcl::PointXYZRGB PointRGBType;
enum MapMode {
    POINT_CLOUD = 0,
    POINT_CLOUD_WITH_SEMANTIC = 1,
    POINT_CLOUD_WITH_MARKER = 2,
};

enum class CityObjectLabel : uint8_t {
    None         = 0,
    Buildings    = 1,
    Fences       = 2,
    Other        = 3,
    Pedestrians  = 4,
    Poles        = 5,
    RoadLines    = 6,
    Road         = 7,
    Sidewalks    = 8,
    Vegetation   = 9,
    Vehicles     = 10,
    Walls        = 11,
    TrafficSigns = 12,
    Sky          = 13,
    Ground       = 14,
    Bridge       = 15,
    RailTrack    = 16,
    GuardRail    = 17,
    TrafficLight = 18,
    Static       = 19,
    Dynamic      = 20,
    Water        = 21,
    Terrain      = 22
};

// 自定义tuple哈希，用于map key
struct TupleHash
{
    size_t operator()(const std::tuple<uint8_t,uint8_t,uint8_t>& t) const
    {
        auto [r,g,b] = t;
        return (static_cast<size_t>(r) << 16) | (static_cast<size_t>(g) << 8) | b;
    }
};

using ColorMap = std::unordered_map<std::tuple<uint8_t,uint8_t,uint8_t>, CityObjectLabel, TupleHash>;

class PointCloudMap {
public:
    PointCloudMap(int mode, int interval);
    ~PointCloudMap();

    // TF 变换相关
    void UpdateTransforms();
    Eigen::Matrix4f BuildTransformMatrix(const tf::StampedTransform& transform);
    Eigen::Matrix4f HomoTransformInv(const Eigen::Matrix4f& T);

    // 订阅回调
    void CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);
    void ImageCallback(const sensor_msgs::ImageConstPtr &msg);
    void LidarCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

    // 功能函数
    bool PointsToMap(pcl::PointCloud<PointType>::Ptr& map_cloud);
    void SavePointsMap();
    void Run();
    void SemanticSegCallback(const sensor_msgs::ImageConstPtr &msg);
    bool PointsRgbToMap(pcl::PointCloud<PointRGBType>::Ptr& map_cloud);
    void PointCloudSetSemantic(pcl::PointCloud<PointRGBType>::Ptr& cloud_rgb,
        pcl::PointCloud<PointType>::Ptr& cloud, const cv::Mat& seg_img);
    void EgoVehicleSpeedCallback(const std_msgs::Float32ConstPtr &msg);

    ColorMap BuildSemanticColorTable() {
        ColorMap table;
        // {R, G, B}, Label
        table[{0, 0, 0}]     = CityObjectLabel::None;
        table[{70, 70, 70}]  = CityObjectLabel::Buildings;
        table[{100, 40, 40}] = CityObjectLabel::Fences;
        table[{55, 90, 80}]  = CityObjectLabel::Other;
        table[{220, 20, 60}] = CityObjectLabel::Pedestrians;
        table[{153, 153, 153}] = CityObjectLabel::Poles;
        table[{157, 234, 50}] = CityObjectLabel::RoadLines;
        table[{128, 64, 128}] = CityObjectLabel::Road;
        table[{244, 35, 232}] = CityObjectLabel::Sidewalks;
        table[{107, 142, 35}] = CityObjectLabel::Vegetation;
        table[{0, 0, 142}]    = CityObjectLabel::Vehicles;
        table[{102, 102, 156}] = CityObjectLabel::Walls;
        table[{220, 220, 0}]  = CityObjectLabel::TrafficSigns;
        table[{70, 130, 180}] = CityObjectLabel::Sky;
        table[{81, 0, 81}]    = CityObjectLabel::Ground;
        table[{150, 100, 100}] = CityObjectLabel::Bridge;
        table[{230, 150, 140}] = CityObjectLabel::RailTrack;
        table[{180, 165, 180}] = CityObjectLabel::GuardRail;
        table[{250, 170, 30}] = CityObjectLabel::TrafficLight;
        table[{110, 190, 160}] = CityObjectLabel::Static;
        table[{170, 120, 50}] = CityObjectLabel::Dynamic;
        table[{45, 60, 150}]  = CityObjectLabel::Water;
        table[{145, 170, 100}] = CityObjectLabel::Terrain;
        return table;
    }

    bool IsColorBelongSemanticTag(uint8_t r, uint8_t g, uint8_t b, CityObjectLabel target_label) {
        auto it = g_semantic_color_map_.find({r,g,b});
        if(it == g_semantic_color_map_.end())
            return false;
        return it->second == target_label;
    }

    void SetLinearSpeed(float speed) {
        std::lock_guard<std::mutex> lock(ego_speed_lock_);
        linear_speed_ = speed;
    }
    
    float GetLinearSpeed() {
        std::lock_guard<std::mutex> lock(ego_speed_lock_);
        return linear_speed_;
    }
        
private:
    // 相机内参（通过 camera_info 话题获取）
    cv::Mat camera_matrix_;
    int image_width_;
    int image_height_;

    // 外参变换矩阵（通过 /tf 话题获取）
    Eigen::Matrix4f lidar_to_camera_transform_;
    Eigen::Matrix4f T_ego_map_;
    Eigen::Matrix4f T_lidar_ego_;
    
    bool tf_initialized_;
    bool map_initialized_;

    // TF 监听
    tf::TransformListener *tf_listener_;

    // ROS 接口
    ros::NodeHandle pnode_;
    ros::Publisher map_pub_;
    ros::Subscriber camera_info_sub_;
    ros::Subscriber image_sub_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber semantic_seg_sub_;
    ros::Subscriber ego_speed_sub_;

    // 数据缓存
    pcl::PointCloud<PointType>::Ptr lidar_data_;
    cv::Mat camera_data_;
    cv::Mat camera_seg_data_;
    std::mutex lidar_lock_;
    std::mutex camera_lock_;
    std::mutex camera_seg_lock_;

    // 地图点云累积
    pcl::PointCloud<PointType>::Ptr points_map_ = nullptr;
    pcl::PointCloud<PointRGBType>::Ptr points_rgb_map_ = nullptr;

    // 统计
    int frame_count_;
    int projected_points_;
    int mode_;
    int interval_;

    ColorMap g_semantic_color_map_;
    std::mutex ego_speed_lock_;
    float linear_speed_;
};
