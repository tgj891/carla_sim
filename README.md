# Carla Sim - CARLA + ROS1 + Perception Algorithms in Practice

> Source code for the autonomous driving perception simulation tutorial series
> WeChat Official Account: 感知技术life (Chinese readers)

---

## Introduction

A hands-on perception tutorial series built on **CARLA simulator + ROS1**, covering the complete perception pipeline step by step: environment setup, object detection, BEV surround view, semantic segmentation, and more.

## Tutorial Series (1-9 Episodes)

| Episode | Topic | Status |
|:-------|:------|:------:|
| 01 | CARLA + ROS Simulation Environment Setup | Done |
| 02 | Adding NPC Traffic Flow | Done |
| 03 | YOLOv8 Object Detection (TCP Cross-Environment) | Done |
| 04 | Tesla Four-Camera Installation | Done |
| 05 | Four-Camera Calibration & BEV Stitching | Done |
| 06 | Real-Time Semantic Segmentation (SegFormer / YOLOPv2) | Done |
| 07 | BEV + Semantic Segmentation Fusion | Done |
| 08 | Point Cloud & Image Fusion | Done |
| 09 | Monocular Obstacle Distance Estimation | Done |

## Requirements

- Ubuntu 18.04 or higher
- Docker
- NVIDIA GPU driver
- nvidia-container-toolkit
- nvidia-docker2

## Quick Start

### 1. Clone the repository

```bash
git clone https://gitee.com/buffalo891/carla_sim.git
cd carla_sim
```

### 2. Get the Docker image

The pre-built Docker image (26.7GB, includes CARLA + ROS + conda environments + all tutorial code) is distributed via a community channel. For the download link, please join our community (WeChat Official Account: 感知技术life, reply "社群" to join the group; the link is in the group announcement).

```bash
# Extract (if downloaded as .gz)
gzip -d carla_sim.tar.gz

# Load into Docker
docker load -i carla_sim.tar

# Verify
docker images | grep carla
carlasim/carla :v1.0-beta   5493d6abf29e   2 days ago      26.7GB
```

### 3. Start the container

```bash
cd carla_sim
bash docker/scripts/dev_start.sh  # start container
bash docker/scripts/dev_into.sh   # enter container
```

Detailed steps are in the `docs/` directory for each episode.

## Project Structure

```
carla_sim/
├── docker/scripts/       # Container scripts
├── scripts/              # Python2 helper scripts (NPC/chessboard)
├── tcp_bridge/           # TCP image transfer (Py2 -> Py3)
├── sim_workspace/        # Algorithm workspace
│   ├── bev_surround/     # BEV surround view stitching
│   ├── image_detect/     # YOLOv8 object detection
│   ├── image_seg/        # Semantic segmentation (SegFormer/YOLOPv2)
│   ├── image_pointcloud/ # Point cloud & image fusion
│   └── pointcloud_road_detect/ # Point cloud road detection
├── docs/                 # Tutorial docs (1-9)
├── data/pictures/        # Screenshots
├── ros_bridge_ws/src/    # CARLA ROS Bridge source
├── CarlaUe4.sh           # Launch CARLA UE4
└── ros_bridge_env.sh     # ROS environment config
```

## Models

Model weights are **not included** in this repository (several hundred MB each). Download them separately:

- YOLOv8: `yolov8s.pt` -> `sim_workspace/image_detect/weights/`
- YOLOPv2: `yolopv2.pt` -> `sim_workspace/image_seg/weight/`
- SegFormer: `segformer-b2-finetuned-ade-512-512` -> `sim_workspace/image_seg/weight/`

## License

This project's **code** is open-sourced under the **MIT License**, but it does **not** include any model weight files.

YOLOv8 / YOLOPv2 / SegFormer pretrained models are copyrighted by their respective authors and subject to their own licenses:
- YOLOv8 (ultralytics): AGPL-3.0, commercial use requires a commercial license or open-sourcing your derivative code
- YOLOPv2: GPL
- SegFormer (NVIDIA): Apache-2.0 / MIT

For commercial use, please verify and obtain the appropriate commercial licenses yourself. This project assumes no liability for license risks arising from model usage.

## Contact

- WeChat Official Account: 感知技术life (for Chinese readers)
- Bilibili: https://space.bilibili.com/1990924908
