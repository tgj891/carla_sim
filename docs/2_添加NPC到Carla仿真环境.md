# 2_添加NPC到Carla仿真环境
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

# 4 添加NPC到仿真环境
1. 新开一个终端
```
cd carla_sim    
bash docker/scripts/dev_into.sh
```
2. 启动可视化
```
cd /opt/carla_ws
python scripts/spawn_random_npc.py # 默认添加10个车5个人
python scripts/spawn_random_npc.py _vehicle_count:=20 _walker_count:=10 #自定义添加
```
![图片](../data/pictures/ue4_npc.png)