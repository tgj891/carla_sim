#! /bin/bash

colmap database_creator --database_path ./carla_colmap_data/database.db
echo "database_creator ok"

colmap feature_extractor \
  --database_path ./carla_colmap_data/database.db \
  --image_path ./carla_colmap_data/images \
  --ImageReader.single_camera 1 \
  --ImageReader.camera_model PINHOLE \
  --ImageReader.camera_params "640,640,640,360"

echo "feature_extractor ok"

colmap sequential_matcher --database_path ./carla_colmap_data/database.db
echo "sequential_matcher ok"

colmap mapper \
  --database_path ./carla_colmap_data/database.db \
  --image_path ./carla_colmap_data/images \
  --export_path ./carla_colmap_data/sparse
echo "colmap success"
