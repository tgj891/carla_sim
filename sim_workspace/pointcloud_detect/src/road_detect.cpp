#include "road_detect.h"

PointCloudRoadDetect::PointCloudRoadDetect(int mode, std::string& path) {
    mode_ = mode;
    viewer_ = new pcl::visualization::CloudViewer("pointcloud road detect");

    frontimgsub_ = pnode_.subscribe("/carla/ego_vehicle/semantic_seg_main_front/image", 100, &PointCloudRoadDetect::FrontSemanticImageCallback, this);
    cloudsub_ = pnode_.subscribe("/carla/ego_vehicle/lidar", 1, &PointCloudRoadDetect::CloudCallback, this);
    camera_info_sub_ = pnode_.subscribe(
        "/carla/ego_vehicle/rgb_main_front/camera_info", 1,
        &PointCloudRoadDetect::CameraInfoCallback, this);
        
    if (mode_ == MODE_POINTCLOUD_MAP_SEG 
        || mode_ == MODE_POINTCLOUD_SEMANTIC_SEG) {
        tf_initialized_ = false;
        g_semantic_color_map_ = BuildSemanticColorTable();
        
        tf_listener_ = new tf::TransformListener();
        if (mode_ == MODE_POINTCLOUD_MAP_SEG) {
            map_pub_ = pnode_.advertise<sensor_msgs::PointCloud2>("/carla/ego_vehicle/map", 1);
            map_cloud_.reset(new pcl::PointCloud<PointType>());
            LoadMapPcd(path);
            
            map_timer_ = pnode_.createTimer(ros::Duration(1.0), &PointCloudRoadDetect::MapTimerCallback, this);
        }
    }

    // object detect init
    object_detect_.reset(new ObjectDetect());
    if (!object_detect_->Init()) {
        std::cout << "object detect init failed" << std::endl;
    }
}

void PointCloudRoadDetect::MapPublic() {
    // 实时发布累积地图点云, 方便 rviz 查看
    sensor_msgs::PointCloud2 out_msg;
    pcl::toROSMsg(*map_cloud_, out_msg);
    out_msg.header.stamp = ros::Time::now();
    out_msg.header.frame_id = "map";
    map_pub_.publish(out_msg);
}

void PointCloudRoadDetect::MapTimerCallback(const ros::TimerEvent& event) {
    MapPublic();
}

Eigen::Matrix4f PointCloudRoadDetect::BuildTransformMatrix(const tf::StampedTransform& transform) {
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

Eigen::Matrix4f PointCloudRoadDetect::HomoTransformInv(const Eigen::Matrix4f& T) {
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

void PointCloudRoadDetect::UpdateTransforms(const ros::Time& stamp) {
    try {
        if (!tf_initialized_) {
            // 一次性初始化静态传感器外参（camera/lidar 相对 ego_vehicle 的安装位置不随时间变化）
            // 相机相对车辆的变换 (camera <- ego_vehicle)
            tf::StampedTransform cam_transform;
            tf_listener_->waitForTransform(
                "ego_vehicle", "ego_vehicle/rgb_main_front",
                ros::Time(0), ros::Duration(5.0));
            tf_listener_->lookupTransform(
                "ego_vehicle", "ego_vehicle/rgb_main_front", ros::Time(0), cam_transform);

            // 激光雷达相对车辆的变换 (lidar <- ego_vehicle)
            tf::StampedTransform lidar_transform;
            tf_listener_->waitForTransform(
                "ego_vehicle", "ego_vehicle/lidar",
                ros::Time(0), ros::Duration(5.0));
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
            ROS_INFO("TF static transforms initialized!");
        }

        // 动态变换 map <- ego_vehicle: 必须与激光雷达帧同步,
        // 用点云消息时间戳查询, 保证变换对应点云采集时刻而非处理时刻
        tf::StampedTransform ego_transform;
        bool got_transform = false;
        if (!stamp.isZero()) {
            // 优先: 用点云时间戳查询, 等待该时刻的变换可用（最多 0.2s）
            if (tf_listener_->waitForTransform(
                    "map", "ego_vehicle", stamp, ros::Duration(0.2))) {
                tf_listener_->lookupTransform(
                    "map", "ego_vehicle", stamp, ego_transform);
                got_transform = true;
            }
        }
        if (!got_transform) {
            // 回退: 时间戳过早/过新导致不可用, 退化为最新变换（近似同步）
            tf_listener_->lookupTransform(
                "map", "ego_vehicle", ros::Time(0), ego_transform);
            got_transform = true;
        }
        T_ego_map_ = BuildTransformMatrix(ego_transform);
    } catch (tf::TransformException &e) {
        ROS_WARN_THROTTLE(2.0, "TF lookup failed: %s", e.what());
    } catch (std::exception &e) {
        ROS_WARN_THROTTLE(2.0, "TF update error: %s", e.what());
    }
}

void PointCloudRoadDetect::CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg) {
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

void PointCloudRoadDetect::FrontSemanticImageCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv::Mat img(msg->height, msg->width, CV_8UC3);
    if (msg->encoding.compare("bgra8") == 0) {
        cv::Mat mat(msg->height, msg->width, CV_8UC3);
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        mat = cv_ptr->image;
        memcpy(img.data, mat.data, msg->height * msg->width * 3);
    } else {
        int size = msg->step * msg->height;
        memcpy(img.data, &(msg->data[0]), size);
    }
    std::lock_guard<std::mutex> lock(semantic_img_lock_);
    semantic_img_ = img;

    // cv::imshow("front_img",img);
    // waitKey(1);
}

void PointCloudRoadDetect::LoadMapPcd(std::string path) {
    if (pcl::io::loadPCDFile<PointType>(path.c_str(), *map_cloud_) == -1) {
        std::cout << "Couldn't read file: " << path << std::endl;
        return;
    }
    std::cout << "Loaded map PCD: " << map_cloud_->points.size() << " points" << std::endl;

    // 构建 kd-tree
    BuildKdTree(map_cloud_);
}

void PointCloudRoadDetect::BuildKdTree(pcl::PointCloud<PointType>::Ptr& cloud) {
    kdtree_.reset(new pcl::search::KdTree<PointType>());
    kdtree_->setInputCloud(cloud);
    std::cout << "KdTree built with " << cloud->points.size() << " points" << std::endl;
}

void PointCloudRoadDetect::KnnSearch(pcl::PointCloud<PointType>::Ptr& cloud,
                                     const PointType& query, int k) {
    if (kdtree_ == nullptr) {
        std::cout << "KdTree not initialized!" << std::endl;
        return;
    }
    std::vector<int> indices(k);
    std::vector<float> distances(k);
    int found = kdtree_->nearestKSearch(query, k, indices, distances);
    std::cout << "KNN search (k=" << k << "): found " << found << " neighbors" << std::endl;
    for (int i = 0; i < found; i++) {
        const auto& pt = cloud->points[indices[i]];
        std::cout << "  [" << i << "] idx=" << indices[i]
                  << " dist=" << distances[i]
                  << " pos=(" << pt.x << ", " << pt.y << ", " << pt.z << ")" << std::endl;
    }
}

bool PointCloudRoadDetect::RadiusSearch(pcl::PointCloud<PointType>::Ptr& cloud,
                                        const PointType& query, double radius, std::vector<int>& indices) {
    if (kdtree_ == nullptr) {
        std::cout << "KdTree not initialized!" << std::endl;
        return false;
    }
    std::vector<float> distances;
    int found = kdtree_->radiusSearch(query, radius, indices, distances);
    // std::cout << "Radius search (r=" << radius << "): found " << found << " points" << std::endl;
    if (found > 0) {
        return true;
    }
    return false;
}

void PointCloudRoadDetect::CloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    pcl::PointCloud<PointType>::Ptr in_cloud(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*msg, *in_cloud);

    // 用点云消息时间戳同步 TF, 保证变换对应激光雷达采集时刻
    const ros::Time& cloud_stamp = msg->header.stamp;

    if (mode_ == MODE_POINTCLOUD_SEG) {
        RansacRoadDetectV2(in_cloud, false);
        ClusterNonGroundPoints(in_cloud);
    } else if (mode_ == MODE_POINTCLOUD_SEMANTIC_SEG) {
        UpdateTransforms(cloud_stamp);
        SemanticSegmentation(in_cloud, false);
        ClusterNonGroundPoints(in_cloud);
    } else if (mode_ == MODE_POINTCLOUD_MAP_SEG) {
        if (map_cloud_ == nullptr || map_cloud_->empty()) {
            std::cout << "Map cloud is empty, skipping ground detection" << std::endl;
            return;
        }
        UpdateTransforms(cloud_stamp);
        PointCloudSegmentationByMap(in_cloud, false);
        ClusterNonGroundPoints(in_cloud);
    } else {
        std::cout << "Unknown mode: " << mode_ << std::endl;
    }
}

void PointCloudRoadDetect::ClusterNonGroundPoints(pcl::PointCloud<PointType>::Ptr& in_cloud) {
    if (in_cloud == nullptr || in_cloud->empty()) {
        return;
    }
    // 提取非地面点 (分割结果中非地面点被标记为绿色: r=0, g=255, b=0)
    pcl::PointCloud<PointType>::Ptr non_ground_cloud(new pcl::PointCloud<PointType>());
    for (const auto& pt : in_cloud->points) {
        if (pt.r == 0 && pt.g == 255 && pt.b == 0) {
            non_ground_cloud->points.push_back(pt);
        }
    }
    if (non_ground_cloud->empty()) {
        std::cout << "No non-ground points found, skip clustering" << std::endl;
        return;
    }
    non_ground_cloud->width = non_ground_cloud->points.size();
    non_ground_cloud->height = 1;

    std::cout << "Extracted " << non_ground_cloud->points.size() << " non-ground points" << std::endl;

    // 传入障碍物聚类
    std::vector<std::pair<pcl::PointCloud<PointType>::Ptr, pcl::PointCloud<PointType>::Ptr>> clusters;
    object_detect_->ClusterObstacles(non_ground_cloud, clusters);

    // 合并聚类结果(已按簇着色)到单个点云, 通过 road_detect 的 viewer 展示
    if (!clusters.empty()) {
        pcl::PointCloud<PointType>::Ptr cluster_cloud(new pcl::PointCloud<PointType>());
        for (const auto& cluster : clusters) {
            *cluster_cloud += *cluster.first;
        }
        viewer_->showCloud(cluster_cloud);

        // cv mat
        int bev_size = 1000;
        cv::Mat bev_img = cv::Mat::zeros(bev_size, bev_size, CV_8UC3);
        ProjectPointsToImage(clusters, bev_img);
        cv::imshow("bev_img", bev_img);
        cv::waitKey(1);
    }
}

void PointCloudRoadDetect::ProjectPointsToImage(std::vector<std::pair<pcl::PointCloud<PointType>::Ptr, pcl::PointCloud<PointType>::Ptr>>& clusters, 
    cv::Mat& bev_img) {
    // car 点云投影到 bev_img
    int bev_size = bev_img.rows;
    int half_size = bev_size / 2;
    float resolution = 0.1f;
    int car_width = (int)(1.84f / resolution);
    int car_length = (int)(4.6f / resolution);
    cv::Rect r(half_size - car_width/2,half_size - car_length/2, car_width, car_length);
    cv::rectangle(bev_img, r, cv::Scalar(0,255,0), -1);

    // 障碍物多边形
    Eigen::Matrix4f T_map_ego = HomoTransformInv(T_ego_map_);
    for (const auto& cluster : clusters) {
        std::vector<cv::Point> poly_points;
        for (const auto& pt : cluster.second->points) {
            Eigen::Vector4f l_pt(pt.x, pt.y, pt.z, 1.0);
            Eigen::Vector4f ego_pt = T_lidar_ego_ * l_pt;
            int py = half_size - ego_pt(0) / resolution;
            int px = half_size - ego_pt(1) / resolution;
            poly_points.emplace_back(px,py);
        }
        //第5个参数 true =自动闭合多边形
        cv::polylines(bev_img,
            poly_points,
            true,                  // true：闭合；false：不闭合
            cv::Scalar(255,0,0),
            2);
    }
}

void PointCloudRoadDetect::SemanticSegmentation(pcl::PointCloud<PointType>::Ptr& in_cloud, bool flag) {
    int pointsize = in_cloud->points.size();
    cv::Mat semantic_img;
    {       
        std::lock_guard<std::mutex> lock(semantic_img_lock_);
        semantic_img = semantic_img_.clone();
    }
    float fx = camera_matrix_.at<double>(0, 0);
    float fy = camera_matrix_.at<double>(1, 1);
    float cx = camera_matrix_.at<double>(0, 2);
    float cy = camera_matrix_.at<double>(1, 2);
    int w = semantic_img.cols;
    int h = semantic_img.rows;

    for (int i = 0; i < pointsize; i++) {
        PointType &pt = in_cloud->points[i];
        if (pt.x < 0.0f) {
            // 非fov点
            pt.r = 0;
            pt.g = 0;
            pt.b = 255;
            continue;
        }
        // 跳过相机后方的点
        Eigen::Vector4f p(pt.x, pt.y, pt.z, 1.0);
        Eigen::Vector4f p_cam = lidar_to_camera_transform_ * p;
        if (p_cam(2) <= 0.1f) {
            // 非fov点
            pt.r = 0;
            pt.g = 0;
            pt.b = 255;
            continue;
        }
        // 针孔投影: u = (fx * X + cx * Z) / Z, v = (fy * Y + cy * Z) / Z
        float inv_z = 1.0f / p_cam(2);
        int u = (int)(fx * p_cam(0) * inv_z + cx + 0.5f);
        int v = (int)(fy * p_cam(1) * inv_z + cy + 0.5f);
        if (u >= 0 && u < w && v >= 0 && v < h) {
            // 语义分割图像: CARLA semantic_seg 是 BGRA, 第2通道(B)含标签
            cv::Vec3b pixel = semantic_img.at<cv::Vec3b>(v, u);
            if (!IsColorBelongSemanticTag(pixel[2], pixel[1], pixel[0], CityObjectLabel::Ground)
                && !IsColorBelongSemanticTag(pixel[2], pixel[1], pixel[0], CityObjectLabel::Road)
                && !IsColorBelongSemanticTag(pixel[2], pixel[1], pixel[0], CityObjectLabel::RoadLines)) {
                // 非地面点
                pt.r = 0;
                pt.g = 255;
                pt.b = 0;
            } else {
                // 地面点
                pt.r = 255;
                pt.g = 0;
                pt.b = 0;
            }
        } else {
            // 非fov点
            pt.r = 0;
            pt.g = 0;
            pt.b = 255;
        }
    }

    if (flag) {
        viewer_->showCloud(in_cloud);
    }
}

void PointCloudRoadDetect::PointCloudSegmentationByMap(pcl::PointCloud<PointType>::Ptr& in_cloud, bool flag) {
    int pointsize = in_cloud->points.size();
    float lidar_height = 2.4;
    float ground_up_z_threshold = -lidar_height + 0.05;
    float ground_down_z_threshold = -lidar_height - 0.05;
    float search_radius = 0.8;
    for (int i = 0; i < pointsize; i++) {
        PointType &pt = in_cloud->points[i];
        if (pt.z < ground_up_z_threshold && pt.z > ground_down_z_threshold) {
            // 地面点
            pt.r = 255;
            pt.g = 0;
            pt.b = 0;
        } else {
            PointType query_pt;
            // P_map = T_ego_map * T_lidar_ego * P_lidar
            Eigen::Vector4f P_lidar(pt.x, pt.y, pt.z, 1.0);
            Eigen::Vector4f P_map = T_ego_map_ * T_lidar_ego_ * P_lidar;
            query_pt.x = P_map(0);
            query_pt.y = P_map(1);
            query_pt.z = P_map(2);
  
            std::vector<int> indices;
            if (RadiusSearch(map_cloud_, query_pt, search_radius, indices)) {
                if (indices.size() > 3) {
                    // 使用SVD对indices中的地图点进行平面拟合
                    Eigen::MatrixXd fit_points(indices.size(), 3);
                    for (size_t j = 0; j < indices.size(); j++) {
                        const auto& map_pt = map_cloud_->points[indices[j]];
                        fit_points(j, 0) = map_pt.x;
                        fit_points(j, 1) = map_pt.y;
                        fit_points(j, 2) = map_pt.z;
                    }

                    // 计算质心
                    Eigen::Vector3d centroid = fit_points.colwise().mean();
                    // 去中心化
                    fit_points.rowwise() -= centroid.transpose();

                    // SVD分解获取平面法向量（最小奇异值对应的列向量）
                    Eigen::JacobiSVD<Eigen::MatrixXd> svd(fit_points, Eigen::ComputeThinV);
                    Eigen::Vector3d plane_normal = svd.matrixV().col(2);

                    // 计算query_pt到拟合平面的距离
                    Eigen::Vector3d query_diff(query_pt.x - centroid(0),
                                            query_pt.y - centroid(1),
                                            query_pt.z - centroid(2));
                    double dist_to_plane = std::abs(plane_normal.dot(query_diff));

                    // 根据距离阈值判断是否为地面点
                    double dist_threshold = 0.1; // 距离阈值（米）
                    if (dist_to_plane < dist_threshold) {
                        // 地面点
                        pt.r = 255;
                        pt.g = 0;
                        pt.b = 0;
                    } else {
                        // 非地面点
                        pt.r = 0;
                        pt.g = 255;
                        pt.b = 0;
                    }
                } else {
                    // 非地面点
                    pt.r = 0;
                    pt.g = 255;
                    pt.b = 0;
                }
            } else {
                // 非地面点
                pt.r = 0;
                pt.g = 255;
                pt.b = 0;
            }
        }
    }

    if (flag) {
        viewer_->showCloud(in_cloud);
    }
}

void PointCloudRoadDetect::RansacRoadDetectV2(pcl::PointCloud<PointType>::Ptr& in_cloud, bool flag) {
        // init grid mat
    float width_threshold = 10.0;
    float length_threshold = 30.0;
    float org_height_up_threshold = 0.2;
    float ground_down_threshold = 4.0;
    float lidar_height = 2.4;
    float height_threshold = org_height_up_threshold + lidar_height + ground_down_threshold;
    float gridside = 0.1;
    int maxval = 1e3;
    int interval_num = 10;
    clock_t start = clock();
    int grid_yaxis = (int)(width_threshold / gridside + 1) * 2; //y axis
    int grid_xaxis = (int)(length_threshold / gridside + 1) * 2; //x axis
    int grid_zaxis = (int)(height_threshold / gridside + 1) * 2; //z axis
    int halfwidth = (int)(grid_yaxis / 2);
    int halflength = (int)(grid_xaxis / 2);
    int halfheight = (int)(grid_zaxis / 2);
    int grid3dnum = grid_yaxis * grid_xaxis * grid_zaxis;
    std::vector<std::vector<int>> grid3d_vec;
    grid3d_vec.reserve(grid3dnum);
    grid3d_vec.resize(grid3dnum);

    int mincount = grid_yaxis / interval_num + 1;
    std::vector<std::vector<int>> minchannel_vec(grid_xaxis, std::vector<int>(mincount, maxval));
    std::vector<std::vector<int>> mincol_vec(grid_xaxis, std::vector<int>(mincount, 0));
    int pointsize = in_cloud->points.size();
    for (int i = 0; i < pointsize; i++) {
        PointType &pt = in_cloud->points[i];
        if (std::abs(pt.x) < length_threshold && std::abs(pt.y) < width_threshold && pt.z < org_height_up_threshold && pt.z >= -lidar_height-ground_down_threshold) {
            int prow = pt.x >= 0 ? (int)(halflength - (int)(std::abs(pt.x) / gridside) - 1) : (int)(halflength + (int)(std::abs(pt.x) / gridside));
            int pcol = pt.y >= 0 ? (int)(halfwidth - (int)(std::abs(pt.y) / gridside) - 1) : (int)(halfwidth + (int)(std::abs(pt.y) / gridside));
            int pchannel = pt.z >= 0 ? (int)(halfheight + (int)(std::abs(pt.z) / gridside) - 1) : (int)(halfheight - (int)(std::abs(pt.z) / gridside));
            int id = prow * grid_yaxis * grid_zaxis + pcol * grid_zaxis + pchannel;
            int mincol = pcol / interval_num;
            if (pchannel < minchannel_vec[prow][mincol]) {
                minchannel_vec[prow][mincol] = pchannel;
                mincol_vec[prow][mincol] = pcol;
            }
            grid3d_vec[id].push_back(i);
        }
        pt.r = 0;
        pt.g = 255;
        pt.b = 0;
    }

    clock_t end1 = clock();
    std::cout << " grid time is: " << static_cast<double>(end1 - start) / CLOCKS_PER_SEC * 1000 << "ms\n";

    for (int row = 0; row < grid_xaxis; row++) {
        int min_val = *std::min_element(minchannel_vec[row].begin(), minchannel_vec[row].end());
        for (int mincol = 0; mincol < mincount; mincol++) {
            int min_channel = minchannel_vec[row][mincol];
            if (min_channel == maxval) {
                continue;
            }

            // if (min_channel > min_val + 1) {
            //     continue;
            // }
            int min_col = mincol_vec[row][mincol];
            int startcol = mincol * interval_num;
            int endcol = mincol * interval_num + interval_num;
            if (min_col == 0) {
                for (int col = 0; col < endcol; col++) {
                    bool first = false;
                    for (int channel = min_channel - 2; channel <= min_channel + 1; channel++) {
                        int id = row * grid_yaxis * grid_zaxis + col * grid_zaxis + channel;
                        for (size_t j = 0; j < grid3d_vec[id].size(); j++) {
                            PointType &pt = in_cloud->points[grid3d_vec[id][j]];
                            pt.r = 255;
                            pt.g = 0;
                            pt.b = 0;
                            if (!first) {
                                min_channel = channel;
                                first = true;
                            }
                        }

                        if (first){
                            // for (size_t j = 0; j < grid3d_vec[id+1].size(); j++) {
                            //     PointType &pt = in_cloud->points[grid3d_vec[id+1][j]];
                            //     pt.r = 255;
                            //     pt.g = 0;
                            //     pt.b = 0;
                            // }
                            break;
                        }
                    }
                }
            } else {
                for (int col = min_col; col < endcol; col++) {
                    bool first = false;
                    for (int channel = min_channel - 2; channel <= min_channel + 1; channel++) {
                        int id = row * grid_yaxis * grid_zaxis + col * grid_zaxis + channel;
                        for (size_t j = 0; j < grid3d_vec[id].size(); j++) {
                            PointType &pt = in_cloud->points[grid3d_vec[id][j]];
                            pt.r = 255;
                            pt.g = 0;
                            pt.b = 0;
                            if (!first) {
                                min_channel = channel;
                                first = true;
                            }
                        }
                        if (first){
                            // for (size_t j = 0; j < grid3d_vec[id+1].size(); j++) {
                            //     PointType &pt = in_cloud->points[grid3d_vec[id+1][j]];
                            //     pt.r = 255;
                            //     pt.g = 0;
                            //     pt.b = 0;
                            // }
                            break;
                        }
                    }
                }

                for (int col = min_col - 1; col >= startcol; col--) {
                    bool first = false;
                    for (int channel = min_channel - 2; channel <= min_channel + 1; channel++) {
                        int id = row * grid_yaxis * grid_zaxis + col * grid_zaxis + channel;
                        for (size_t j = 0; j < grid3d_vec[id].size(); j++) {
                            PointType &pt = in_cloud->points[grid3d_vec[id][j]];
                            pt.r = 255;
                            pt.g = 0;
                            pt.b = 0;
                            if (!first) {
                                min_channel = channel;
                                first = true;
                            }
                        }
                        if (first){
                            // for (size_t j = 0; j < grid3d_vec[id+1].size(); j++) {
                            //     PointType &pt = in_cloud->points[grid3d_vec[id+1][j]];
                            //     pt.r = 255;
                            //     pt.g = 0;
                            //     pt.b = 0;
                            // }
                            break;
                        }
                    }
                }
            }
        }
    }
    clock_t end = clock();
    std::cout << " all time is: " << static_cast<double>(end - start) / CLOCKS_PER_SEC * 1000 << "ms\n";
    if (flag) {
        viewer_->showCloud(in_cloud);
    }
}
