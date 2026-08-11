# Carla Tutorial - Carla 仿真 + ROS + 深度学习实战

> 自动驾驶感知仿真系列教程配套代码
> 公众号：感知技术life | B站/CSDN 同步更新

---

## 项目简介

基于 **Carla 仿真环境 + ROS1 + 深度学习** 的自动驾驶感知实战系列，从环境搭建到目标检测、BEV拼接、语义分割，一步步带你跑通完整的感知数据流。

## 系列目录

| 章节 | 内容 | 状态 |
|:----|:-----|:----:|
| 01 | Carla + ROS 仿真环境搭建 | 已完成 |
| 02 | 添加 NPC 交通流 | 已完成 |
| 03 | YOLOv8 目标检测（TCP跨环境通信） | 已完成 |
| 04 | Tesla 四路相机安装 | 已完成 |
| 05 | 四路相机标定与 BEV 拼接 | 已完成 |
| 06 | 实时语义分割（SegFormer / YOLOPv2） | 已完成 |

## 环境要求

- Ubuntu 18.04+
- Docker（推荐）
- NVIDIA GPU（6GB+ 显存）
- Carla V1.3 镜像

## 快速开始

```bash
# 1. 克隆仓库
git clone https://gitee.com/buffalo891/carla-tutorial.git
cd carla-tutorial

# 2. 启动容器
cd docker
bash scripts/dev_start.sh
bash scripts/dev_into.sh

# 3. 启动 Carla UE4
cd /opt/carla_ws
bash ./CarlaUe4.sh
```

详细步骤见各章节目录下的 README。

## 目录结构

```
carla-tutorial/
├── docker/               # Docker 环境配置
│   └── scripts/          # 启动脚本
├── 01_env_setup/         # 环境搭建
├── 02_spawn_npc/         # NPC 生成
├── 03_yolo_detect/       # YOLO 检测
├── 04_four_camera/       # 四路相机
├── 05_bev_surround/      # BEV 拼接
└── 06_semantic_seg/      # 语义分割
```

## 开源协议

本项目代码遵循 MIT License 开源。

**注意：** Docker 镜像包为付费产品，不包含在本仓库中。需要开箱即用的完整环境，请关注公众号或访问面包多店铺。

## 联系我们

- 公众号：感知技术life
- B站：感知技术life
- 技术交流群：公众号回复"加群"
