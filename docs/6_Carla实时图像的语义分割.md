# 6_Carla实时图像的语义分割
# 1 启动容器

1. 启动容器
```
cd carla_sim
bash docker/scripts/dev_start.sh
```
2. 进入镜像
```
bash docker/scripts/dev_into.sh
```

# 2 启动Carla UE4

```
cd /opt/carla_ws
bash ./CarlaUe4.sh
```
![图片](../data/pictures/ue4_run.png)

# 3 启动ros bridge car
1. 新开一个终端
```
cd carla_sim
bash docker/scripts/dev_into.sh
```
2. source ros bridge_ws环境
```
source ros_bridge_env.sh
cd /opt/carla_ws/ros_bridge_ws
source devel/setup.bash
```
3. 启动ros bridge car
```
cd /opt/carla_ws
roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch
```
![图片](../data/pictures/ros_bridge_car.png)

# 4 发布相机图像
新开一个终端
```
#ros图像转tcp消息
cd /opt/carla_ws
python tcp_bridge/tcp_bridge_server_multi_camera_py2.py # 启动环视四路相机发布

python tcp_bridge/tcp_bridge_server_py2.py # 只启动前视相机发布
```

# 5 创建NPC
新开一个终端
```
python scripts/spawn_random_npc_py2.py _vehicle_count:=20 _walker_count:=10 #自定义添加数量
```
![图片](../data/pictures/ue4_npc.png)

# 6 启动语义分割模型
新开一个终端
1.采用segformer模型
```
cd /opt/carla_ws/sim_workspace/image_seg

python image_seg_segformer_online.py #在线实时语义分割
python image_seg_segformer.py #离线语义分割
```
![图片](../data/pictures/semantic_segment_segformer.png)


2.采用yolopv2模型,这个包含车道线和地面的语义分割，以及目标检测功能，如下图。
```
cd /opt/carla_ws/sim_workspace/image_seg

python image_seg_yolopv2_online.py #在线实时语义分割
python image_seg_yolopv2.py #离线语义分割
```
![图片](../data/pictures/semantic_segment_yolopv2.png)

测试了2个模型，分别是segformer和yolopv2，都能够正常工作，但是效果上yolopv2的语义分割效果要好很多。