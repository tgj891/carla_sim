#pragma once

#include "common.h"
#include "object_detect.h"

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
    void ClusterNonGroundPoints(pcl::PointCloud<PointType>::Ptr& in_cloud);
    void RansacRoadDetectV2(pcl::PointCloud<PointType>::Ptr& in_cloud, bool flag);
    void SemanticSegmentation(pcl::PointCloud<PointType>::Ptr& in_cloud, bool flag);
    void PointCloudSegmentationByMap(pcl::PointCloud<PointType>::Ptr& in_cloud, bool flag);
    void UpdateTransforms(const ros::Time& stamp);
    Eigen::Matrix4f BuildTransformMatrix(const tf::StampedTransform& transform);
    Eigen::Matrix4f HomoTransformInv(const Eigen::Matrix4f& T);
    void MapPublic();
    void MapTimerCallback(const ros::TimerEvent& event);
    void CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);
    void ProjectPointsToImage(std::vector<std::pair<pcl::PointCloud<PointType>::Ptr, pcl::PointCloud<PointType>::Ptr>>& clusters, 
    cv::Mat& bev_img);
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

    std::shared_ptr<ObjectDetect> object_detect_ = nullptr;
};
