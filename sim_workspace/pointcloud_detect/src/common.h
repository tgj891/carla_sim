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
#include <pcl/surface/convex_hull.h>
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
