#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import sys, os
import Queue as queue

sys.path.append('/opt/ue4_ws/PythonAPI/carla/dist/carla-0.9.13-py2.7-linux-x86_64.egg')
sys.path.append('/opt/ue4_ws/PythonAPI/carla')
import carla

client = carla.Client("localhost", 2000)
client.set_timeout(10.0)
world = client.get_world()
bp_lib = world.get_blueprint_library()

# ========= 配置参数（可修改）=========
target_actor_id = 109  # 目标 actor 的 ID，相机将围绕它旋转
radius = 4.0
cam_height_offset = 0.6  # 相机相对于目标中心的高度偏移（米）
total_frames = 360
delta_angle_per_tick = (2 * math.pi) / total_frames

image_save_path = "./carla_colmap_data/images/"
sparse_save_path = "./carla_colmap_data/sparse/"

for path_str in [image_save_path, sparse_save_path]:
    if not os.path.exists(path_str):
        os.makedirs(path_str)

# ========= 切换到同步模式 =========
original_settings = world.get_settings()
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05  # 50ms per tick
world.apply_settings(settings)

# ========= 通过 ID 获取目标 actor 作为中心 =========
target_actor = world.get_actor(target_actor_id)
if target_actor is None:
    print("[ERROR] Actor with id %d not found!" % target_actor_id)
    sys.exit(1)

target_loc = target_actor.get_location()
center = carla.Location(x=target_loc.x, y=target_loc.y, z=target_loc.z)
print("[INFO] Target actor found: id=%d, type=%s, pos=(%.2f, %.2f, %.2f)" %
      (target_actor_id, target_actor.type_id, center.x, center.y, center.z))

# 相机绝对高度 = 目标中心高度 + 偏移
cam_height_abs = center.z + cam_height_offset
# 计算俯仰角: 让相机对准目标中心（正 pitch = 向下看）
pitch_deg = math.degrees(math.atan2(cam_height_abs - center.z, radius))
print("[INFO] Camera height=%.2f, pitch=%.1f° (looking at target center)" %
      (cam_height_abs, pitch_deg))

# ========= 图像队列 =========
image_queue = queue.Queue()

def image_callback(image):
    image_queue.put(image)

# RGB相机 blueprint（只复用，不保存实例）
cam_bp = bp_lib.find("sensor.camera.rgb")
cam_bp.set_attribute("image_size_x", "1280")
cam_bp.set_attribute("image_size_y", "720")
cam_bp.set_attribute("fov", "90")

angle = 0.0
saved_count = 0
camera = None

try:
    print("===== Start Carla orbit camera collection (SYNC MODE) =====")
    print("Target: collect %d images, radius=%sm, center=(%.1f,%.1f)" %
          (total_frames, radius, center.x, center.y))
    while True:
        if saved_count >= total_frames:
            print("\n✅ Already collected %d frames, exit loop." % saved_count)
            break

        # 1. 计算新位姿
        angle += delta_angle_per_tick
        x = center.x + radius * math.cos(angle)
        y = center.y + radius * math.sin(angle)
        cam_loc = carla.Location(x, y, cam_height_abs)
        dx = center.x - x
        dy = center.y - y
        yaw_deg = math.degrees(math.atan2(dy, dx))
        cam_rot = carla.Rotation(yaw=yaw_deg, pitch=pitch_deg)
        new_transform = carla.Transform(cam_loc, cam_rot)

        # 2. 销毁旧相机，在新位姿重新生成
        if camera is not None:
            camera.destroy()
        camera = world.spawn_actor(cam_bp, new_transform)
        camera.listen(image_callback)

        # 3. 清空队列残留
        while not image_queue.empty():
            try:
                image_queue.get_nowait()
            except queue.Empty:
                break

        # 4. tick 让相机在新位姿采集一帧
        world.tick()

        # 5. 获取图像（超时 5s）
        try:
            image = image_queue.get(timeout=5.0)
            frame_id = image.frame
            img_name = "%d.png" % frame_id
            img_full_path = os.path.join(image_save_path, img_name)
            image.save_to_disk(img_full_path)
            saved_count += 1
            print("[SAVE] Frame=%6d | num: %d/%d | angle=%6.2f°, pos=(%.2f,%.2f)"
                  % (frame_id, saved_count, total_frames, math.degrees(angle), x, y))
        except queue.Empty:
            print("[WARN] Frame timeout, no image received!")

except KeyboardInterrupt:
    print("\n⌨️ User pressed Ctrl+C, stop collection")
finally:
    print("Cleaning up...")
    if camera is not None:
        camera.destroy()
    world.apply_settings(original_settings)
    print("Done.")