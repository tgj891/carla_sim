#include "common.h"

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

// 保存原始终端设置, 用于退出时恢复
static struct termios g_old_term;

// 设置终端为非阻塞输入(无需回车、不显示回显)
static void set_nonblocking_input() {
    // 非终端环境(如 roslaunch 重定向输入)直接跳过
    if (!isatty(STDIN_FILENO)) {
        return;
    }
    tcgetattr(STDIN_FILENO, &g_old_term);
    struct termios t = g_old_term;
    t.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &t);
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
}

// 恢复终端设置
static void restore_terminal() {
    if (isatty(STDIN_FILENO)) {
        tcsetattr(STDIN_FILENO, TCSANOW, &g_old_term);
    }
}

// 非阻塞读取一个按键, 无输入返回 EOF
static int read_key() {
    return getchar();
}

PointCloudMap::PointCloudMap(int mode, int interval) {
    tf_initialized_ = false;
    map_initialized_ = false;
    frame_count_ = 0;
    projected_points_ = 0;
    image_width_ = 0;
    image_height_ = 0;
    lidar_data_ = nullptr;
    mode_ = mode;
    interval_ = interval;

    g_semantic_color_map_ = BuildSemanticColorTable();
    
    tf_listener_ = new tf::TransformListener();

    // 发布话题
    map_pub_ = pnode_.advertise<sensor_msgs::PointCloud2>("/carla/ego_vehicle/map", 1);

    // 订阅话题
    camera_info_sub_ = pnode_.subscribe(
        "/carla/ego_vehicle/rgb_main_front/camera_info", 1,
        &PointCloudMap::CameraInfoCallback, this);
    image_sub_ = pnode_.subscribe(
        "/carla/ego_vehicle/rgb_main_front/image", 1,
        &PointCloudMap::ImageCallback, this);
    semantic_seg_sub_ = pnode_.subscribe(
        "/carla/ego_vehicle/semantic_seg_main_front/image", 1,
        &PointCloudMap::SemanticSegCallback, this);
    lidar_sub_ = pnode_.subscribe(
        "/carla/ego_vehicle/lidar", 1,
        &PointCloudMap::LidarCallback, this);
    ego_speed_sub_ = pnode_.subscribe(
        "/carla/ego_vehicle/speedometer", 1,
        &PointCloudMap::EgoVehicleSpeedCallback, this);

    ROS_INFO("Waiting for camera_info and TF transforms...");
}

PointCloudMap::~PointCloudMap() {
    delete tf_listener_;
}

Eigen::Matrix4f PointCloudMap::BuildTransformMatrix(const tf::StampedTransform& transform) {
    // 构建 4x4 变换矩阵（局部到父坐标系）
    tf::Vector3 t = transform.getOrigin();
    tf::Matrix3x3 R = transform.getBasis();

    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            T(i, j) = R[i][j];
        }
    }
    T(0, 3) = t.x();
    T(1, 3) = t.y();
    T(2, 3) = t.z();
    return T;
}

Eigen::Matrix4f PointCloudMap::HomoTransformInv(const Eigen::Matrix4f& T) {
    // 4x4齐次变换矩阵求逆
    // 旋转矩阵的逆=转置，平移取 -R^T * t
    Eigen::Matrix4f T_inv = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f R = T.block<3, 3>(0, 0);
    Eigen::Vector3f t = T.block<3, 1>(0, 3);
    Eigen::Matrix3f R_T = R.transpose();
    T_inv.block<3, 3>(0, 0) = R_T;
    T_inv.block<3, 1>(0, 3) = -R_T * t;
    return T_inv;
}

void PointCloudMap::UpdateTransforms() {
    try {
        if (!tf_initialized_) {
            // 等待 TF 数据可用
            tf_listener_->waitForTransform(
                "ego_vehicle", "ego_vehicle/rgb_main_front",
                ros::Time(0), ros::Duration(5.0));
            tf_listener_->waitForTransform(
                "ego_vehicle", "ego_vehicle/lidar",
                ros::Time(0), ros::Duration(5.0));

            // 获取相机相对车辆的变换 (camera <- ego_vehicle)
            tf::StampedTransform cam_transform;
            tf_listener_->lookupTransform(
                "ego_vehicle", "ego_vehicle/rgb_main_front", ros::Time(0), cam_transform);

            // 获取激光雷达相对车辆的变换 (lidar <- ego_vehicle)
            tf::StampedTransform lidar_transform;
            tf_listener_->lookupTransform(
                "ego_vehicle", "ego_vehicle/lidar", ros::Time(0), lidar_transform);

            // 相机相对车辆的变换矩阵
            Eigen::Matrix4f T_camera_ego = BuildTransformMatrix(cam_transform);
            // 激光雷达相对车辆的变换矩阵
            T_lidar_ego_ = BuildTransformMatrix(lidar_transform);

            // 计算 lidar 到 camera 的变换 (camera <- lidar)
            Eigen::Matrix4f T_ego_camera = HomoTransformInv(T_camera_ego);
            lidar_to_camera_transform_ = T_ego_camera * T_lidar_ego_;

            tf_initialized_ = true;
            ROS_INFO("TF transforms initialized!");
        }

        // 获取 ego_vehicle 相对地图的变换 (map <- ego_vehicle)
        tf_listener_->waitForTransform(
            "map", "ego_vehicle", ros::Time(0), ros::Duration(5.0));
        tf::StampedTransform ego_transform;
        tf_listener_->lookupTransform("map", "ego_vehicle", ros::Time(0), ego_transform);
        T_ego_map_ = BuildTransformMatrix(ego_transform);
        map_initialized_ = true;
        // ROS_INFO("Map transform initialized!");
    } catch (tf::TransformException &e) {
        ROS_WARN_THROTTLE(2.0, "TF lookup failed: %s", e.what());
    } catch (std::exception &e) {
        ROS_WARN_THROTTLE(2.0, "TF update error: %s", e.what());
    }
}

void PointCloudMap::EgoVehicleSpeedCallback(const std_msgs::Float32ConstPtr &msg) {
    try {
        // 从 Float32 消息提取线速度
        SetLinearSpeed(msg->data);
    } catch (std::exception &e) {
        ROS_ERROR("Ego vehicle speed callback error: %s", e.what());
    }
}

void PointCloudMap::CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg) {
    try {
        // 从 CameraInfo 消息提取内参矩阵 K (3x3, 按行存储)
        camera_matrix_ = cv::Mat(3, 3, CV_64F);
        for (int i = 0; i < 9; i++) {
            camera_matrix_.at<double>(i / 3, i % 3) = msg->K[i];
        }
        image_width_ = msg->width;
        image_height_ = msg->height;

        // ROS_INFO("Camera info received: %dx%d", msg->width, msg->height);
        // ROS_INFO("  fx=%.6f, fy=%.6f, cx=%.6f, cy=%.6f",
                //  camera_matrix_.at<double>(0, 0), camera_matrix_.at<double>(1, 1),
                //  camera_matrix_.at<double>(0, 2), camera_matrix_.at<double>(1, 2));
    } catch (std::exception &e) {
        ROS_ERROR("Camera info callback error: %s", e.what());
    }
}

void PointCloudMap::ImageCallback(const sensor_msgs::ImageConstPtr &msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        std::lock_guard<std::mutex> lock(camera_lock_);
        camera_data_ = cv_ptr->image.clone();
    } catch (std::exception &e) {
        ROS_ERROR("Image callback error: %s", e.what());
    }
}

void PointCloudMap::SemanticSegCallback(const sensor_msgs::ImageConstPtr &msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        std::lock_guard<std::mutex> lock(camera_seg_lock_);
        camera_seg_data_ = cv_ptr->image.clone();
    } catch (std::exception &e) {
        ROS_ERROR("Semantic seg callback error: %s", e.what());
    }
}

void PointCloudMap::LidarCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    try {
        // 解析点云数据 (PointXYZ 兼容无 intensity 字段的话题)
        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
        pcl::fromROSMsg(*msg, *cloud);
        std::lock_guard<std::mutex> lock(lidar_lock_);
        lidar_data_ = cloud;
    } catch (std::exception &e) {
        ROS_ERROR("Lidar callback error: %s", e.what());
    }
}

void PointCloudMap::PointCloudSetSemantic(pcl::PointCloud<PointRGBType>::Ptr& cloud_rgb,
    pcl::PointCloud<PointType>::Ptr& cloud, const cv::Mat& seg_img) {
    if (seg_img.empty() || camera_matrix_.empty() || !tf_initialized_) {
        return;
    }
    float fx = camera_matrix_.at<double>(0, 0);
    float fy = camera_matrix_.at<double>(1, 1);
    float cx = camera_matrix_.at<double>(0, 2);
    float cy = camera_matrix_.at<double>(1, 2);
    int w = seg_img.cols;
    int h = seg_img.rows;

    // 遍历每个点云点, 投影到语义分割图像上, 取像素标签赋值给 intensity
    for (size_t i = 0; i < cloud->points.size(); i++) {
        auto pt = cloud->points[i];
        if (pt.x < 0.0f) {
            continue;
        }
        // 跳过相机后方的点
        Eigen::Vector4f p(pt.x, pt.y, pt.z, 1.0);
        Eigen::Vector4f p_cam = lidar_to_camera_transform_ * p;
        if (p_cam(2) <= 0.1f) {
            continue;
        }
        // 针孔投影: u = (fx * X + cx * Z) / Z, v = (fy * Y + cy * Z) / Z
        float inv_z = 1.0f / p_cam(2);
        int u = (int)(fx * p_cam(0) * inv_z + cx + 0.5f);
        int v = (int)(fy * p_cam(1) * inv_z + cy + 0.5f);
        if (u >= 0 && u < w && v >= 0 && v < h) {
            // 语义分割图像: CARLA semantic_seg 是 BGRA, 第2通道(B)含标签
            cv::Vec3b pixel = seg_img.at<cv::Vec3b>(v, u);
            if (mode_ == POINT_CLOUD_WITH_MARKER && 
                (!IsColorBelongSemanticTag(pixel[2], pixel[1], pixel[0], CityObjectLabel::Ground)
                && !IsColorBelongSemanticTag(pixel[2], pixel[1], pixel[0], CityObjectLabel::Road)
                && !IsColorBelongSemanticTag(pixel[2], pixel[1], pixel[0], CityObjectLabel::RoadLines)
                )) {
                continue;
            }
            PointRGBType pt_rgb;
            pt_rgb.x = pt.x;
            pt_rgb.y = pt.y;
            pt_rgb.z = pt.z;
            pt_rgb.r = pixel[2];
            pt_rgb.g = pixel[1];
            pt_rgb.b = pixel[0];
            cloud_rgb->points.push_back(pt_rgb);
        }
    }

    cloud_rgb->width = cloud_rgb->points.size();
    cloud_rgb->height = 1;
    cloud_rgb->is_dense = cloud->is_dense;
}

bool PointCloudMap::PointsRgbToMap(pcl::PointCloud<PointRGBType>::Ptr& map_cloud) {
    // 将3D点转换到地图坐标系
    pcl::PointCloud<PointType>::Ptr lidar_data;
    {
        std::lock_guard<std::mutex> lock(lidar_lock_);
        lidar_data = lidar_data_;
    }
    // 语义信息
    cv::Mat camera_seg_data;
    {
        std::lock_guard<std::mutex> lock(camera_seg_lock_);
        camera_seg_data = camera_seg_data_.clone();
    }
    if (lidar_data == nullptr || camera_seg_data.empty()) {
        ROS_WARN_THROTTLE(3.0, "No lidar data or camera seg data received yet...");
        return false;
    }
    if (lidar_data == nullptr) {
        ROS_WARN_THROTTLE(3.0, "No lidar data received yet...");
        return false;
    }
    if (!tf_initialized_) {
        ROS_WARN_THROTTLE(3.0, "TF (lidar/camera) not initialized...");
        return false;
    }
    if (!map_initialized_) {
        ROS_WARN_THROTTLE(3.0, "Map transform not initialized...");
        return false;
    }

    frame_count_++;
    if (frame_count_ % interval_ == 0) {
        pcl::PointCloud<PointRGBType>::Ptr lidar_rgb_data(new pcl::PointCloud<PointRGBType>());
        PointCloudSetSemantic(lidar_rgb_data, lidar_data, camera_seg_data);

        // P_map = T_ego_map * T_lidar_ego * P_lidar
        Eigen::Affine3f transform((T_ego_map_ * T_lidar_ego_).eval());
        pcl::transformPointCloud(*lidar_rgb_data, *map_cloud, transform);
        ROS_INFO_THROTTLE(5.0, "Accumulated frame %d, map points: %zu",
                          frame_count_, (size_t)map_cloud->size());
        return true;
    }
    return false;
}

bool PointCloudMap::PointsToMap(pcl::PointCloud<PointType>::Ptr& map_cloud) {
    // 将3D点转换到地图坐标系
    pcl::PointCloud<PointType>::Ptr lidar_data;
    {
        std::lock_guard<std::mutex> lock(lidar_lock_);
        lidar_data = lidar_data_;
    }
    // 语义信息
    if (lidar_data == nullptr) {
        ROS_WARN_THROTTLE(3.0, "No lidar data received yet...");
        return false;
    }
    if (lidar_data == nullptr) {
        ROS_WARN_THROTTLE(3.0, "No lidar data received yet...");
        return false;
    }
    if (!tf_initialized_) {
        ROS_WARN_THROTTLE(3.0, "TF (lidar/camera) not initialized...");
        return false;
    }
    if (!map_initialized_) {
        ROS_WARN_THROTTLE(3.0, "Map transform not initialized...");
        return false;
    }

    frame_count_++;
    if (frame_count_ % interval_ == 0) {

        // P_map = T_ego_map * T_lidar_ego * P_lidar
        Eigen::Affine3f transform((T_ego_map_ * T_lidar_ego_).eval());
        pcl::transformPointCloud(*lidar_data, *map_cloud, transform);
        ROS_INFO_THROTTLE(5.0, "Accumulated frame %d, map points: %zu",
                          frame_count_, (size_t)map_cloud->size());
        return true;
    }
    return false;
}

void PointCloudMap::SavePointsMap() {
    // 保存累积的地图点云
    if (mode_ == POINT_CLOUD_WITH_SEMANTIC || mode_ == POINT_CLOUD_WITH_MARKER) {
        if (points_rgb_map_ == nullptr || points_rgb_map_->empty()) {
            ROS_WARN("points_rgb_map is empty, nothing to save!");
            return;
        }
        // 降采样处理, 分辨率 0.05
        pcl::VoxelGrid<PointRGBType> voxel_filter;
        voxel_filter.setInputCloud(points_rgb_map_);
        voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f);
        pcl::PointCloud<PointRGBType>::Ptr downsampled_cloud(new pcl::PointCloud<PointRGBType>());
        voxel_filter.filter(*downsampled_cloud);
        if (mode_ == POINT_CLOUD_WITH_SEMANTIC) {
            pcl::io::savePCDFileBinary("points_map_semantic.pcd", *downsampled_cloud);
        } else {
            pcl::io::savePCDFileBinary("points_map_marker.pcd", *downsampled_cloud);
        }
        ROS_INFO("Saved points map: %zu points -> points_map_rgb.pcd", downsampled_cloud->size());
    } else {
        if (points_map_ == nullptr || points_map_->empty()) {
            ROS_WARN("points_map is empty, nothing to save!");
            return;
        }
        // 降采样处理, 分辨率 0.05
        pcl::VoxelGrid<PointType> voxel_filter;
        voxel_filter.setInputCloud(points_map_);
        voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f);
        pcl::PointCloud<PointType>::Ptr downsampled_cloud(new pcl::PointCloud<PointType>());
        voxel_filter.filter(*downsampled_cloud);
        pcl::io::savePCDFileBinary("points_map.pcd", *downsampled_cloud);
        ROS_INFO("Saved points map: %zu points -> points_map.pcd", downsampled_cloud->size());
    }
}

void PointCloudMap::Run() {
    ROS_INFO("Lidar-Camera fusion node started");
    ROS_INFO("Subscribed topics:");
    ROS_INFO("  - /carla/ego_vehicle/rgb_main_front/camera_info");
    ROS_INFO("  - /carla/ego_vehicle/rgb_main_front/image");
    ROS_INFO("  - /carla/ego_vehicle/lidar");
    ROS_INFO("  - /tf");
    ROS_INFO("Publishing topics:");
    ROS_INFO("  - /carla/ego_vehicle/lidar/map");
    ROS_INFO("Press 's' to save points map to pcd");

    ros::Rate rate(10);  // 10 Hz

    while (ros::ok()) {
        // 关键: 处理订阅回调 (roscpp 中回调必须由 spinOnce/spin 触发)
        ros::spinOnce();

        // 更新 TF 变换
        UpdateTransforms();

        // point to map
        if (mode_ == POINT_CLOUD_WITH_SEMANTIC || mode_ == POINT_CLOUD_WITH_MARKER) {
            pcl::PointCloud<PointRGBType>::Ptr map_rgb_cloud(new pcl::PointCloud<PointRGBType>());
            if (GetLinearSpeed() > 0.1f && PointsRgbToMap(map_rgb_cloud)) {

                // 实时发布累积地图点云, 方便 rviz 查看
                if (points_rgb_map_ == nullptr) {
                    points_rgb_map_.reset(new pcl::PointCloud<PointRGBType>());
                }
                *points_rgb_map_ += *map_rgb_cloud;
                sensor_msgs::PointCloud2 out_msg;
                pcl::toROSMsg(*points_rgb_map_, out_msg);
                out_msg.header.stamp = ros::Time::now();
                out_msg.header.frame_id = "map";
                map_pub_.publish(out_msg);
                ROS_INFO("Published points map: %zu points", points_rgb_map_->size());
            }
        } else {
            pcl::PointCloud<PointType>::Ptr map_cloud(new pcl::PointCloud<PointType>());
            if (GetLinearSpeed() > 0.1f && PointsToMap(map_cloud)) {

                // 实时发布累积地图点云, 方便 rviz 查看
                if (points_map_ == nullptr) {
                    points_map_.reset(new pcl::PointCloud<PointType>());
                }
                *points_map_ += *map_cloud;
                sensor_msgs::PointCloud2 out_msg;
                pcl::toROSMsg(*points_map_, out_msg);
                out_msg.header.stamp = ros::Time::now();
                out_msg.header.frame_id = "map";
                map_pub_.publish(out_msg);
                ROS_INFO("Published points map: %zu points", points_map_->size());
            }
        }

        // 按键保存: 按 's'/'S' 将累积地图保存为 pcd
        int key = read_key();
        if (key == 's' || key == 'S') {
            ROS_INFO("Saving points map...");
            SavePointsMap();
        }

        rate.sleep();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pointcloud_map");

    // 设置非阻塞输入(用于按键保存), 退出时恢复终端
    set_nonblocking_input();
    atexit(restore_terminal);

    // 0: point cloud, 1: point cloud with semantic, 2: point cloud with marker
    int mode = atoi(argv[1]);
    int interval = atoi(argv[2]); // 保存间隔, 单位: 帧
    PointCloudMap map_node(mode, interval);
    try {
        map_node.Run();
    } catch (ros::Exception &e) {
        ROS_ERROR("%s", e.what());
    }
    return 0;
}
