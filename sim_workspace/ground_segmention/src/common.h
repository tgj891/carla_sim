#pragma once

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/line_descriptor/descriptor.hpp>
#include <opencv2/ximgproc.hpp>
#include <cmath>

#include <iostream>
#include <csignal>
#include <mutex>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <stereo_msgs/DisparityImage.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/boundary.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <time.h>
#include <math.h>
#include <mutex>
#include <cassert>
#include <unordered_map>
#include <tuple>

using namespace std;
using namespace cv;
#define POINTXYZRGB

#ifdef POINTXYZRGB
typedef pcl::PointXYZRGB PointType;
#else
typedef pcl::PointXYZ PointType;
#endif

const double eps = 1e-8;
#define Equ(a,b) (fabs((a)-(b))<eps)
#define More(a,b) (((a)-(b))>(eps))
#define Less(a,b) (((a)-(b))<(-eps))
#define MoreEque(a,b) (((a)-(b))>(-eps))
#define LessEque(a,b) (((a)-(b))<(eps))

#define M_PI		3.14159265358979323846
#define POINTCLOUD_SEG 1
#define OBJECT_DETECT 0
#define IS_VALID( a ) ( ( std::abs( a ) > 1e8 ) ? true : false )
const int kMaxPointCloudGPUSize = 120000;

enum Mode {
    MODE_POINTCLOUD_SEG = 0,
    MODE_POINTCLOUD_SEMANTIC_SEG = 1,
    MODE_POINTCLOUD_MAP_SEG = 2,
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

class PointCloudRoadDetect {
public:
    PointCloudRoadDetect(int mode, std::string& path);
    virtual ~PointCloudRoadDetect() {
        delete tf_listener_;
    }
    void FrontSemanticImageCallback(const sensor_msgs::ImageConstPtr &msg);
    void LoadMapPcd(std::string path);
    void BuildKdTree(pcl::PointCloud<PointType>::Ptr& cloud);
    void KnnSearch(pcl::PointCloud<PointType>::Ptr& cloud,
                   const PointType& query, int k);
    bool RadiusSearch(pcl::PointCloud<PointType>::Ptr& cloud,
                      const PointType& query, double radius, std::vector<int>& indices);
    void CloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void RansacRoadDetectV2(pcl::PointCloud<PointType>::Ptr& in_cloud, bool flag);
    void SemanticSegmentation(pcl::PointCloud<PointType>::Ptr& in_cloud, bool flag);
    void PointCloudSegmentationByMap(pcl::PointCloud<PointType>::Ptr& in_cloud, bool flag);
    void UpdateTransforms(const ros::Time& stamp);
    Eigen::Matrix4f BuildTransformMatrix(const tf::StampedTransform& transform);
    Eigen::Matrix4f HomoTransformInv(const Eigen::Matrix4f& T);
    void MapPublic();
    void MapTimerCallback(const ros::TimerEvent& event);
    void CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);
    
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
private:
    int mode_;
    ros::NodeHandle pnode_;
    ros::Subscriber frontimgsub_;
    ros::Subscriber cloudsub_;
    ros::Subscriber camera_info_sub_;
    pcl::visualization::CloudViewer *viewer_;
    pcl::search::KdTree<PointType>::Ptr kdtree_;
    pcl::PointCloud<PointType>::Ptr map_cloud_;

    bool tf_initialized_;
    tf::TransformListener *tf_listener_;
    Eigen::Matrix4f lidar_to_camera_transform_;
    Eigen::Matrix4f T_ego_map_;
    Eigen::Matrix4f T_lidar_ego_;
    ros::Publisher map_pub_;
    ros::Timer map_timer_;

    std::mutex semantic_img_lock_;
    cv::Mat semantic_img_;
    cv::Mat camera_matrix_;
    int image_width_;
    int image_height_;
    ColorMap g_semantic_color_map_;
};
