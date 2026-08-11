#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import sys
import math
import time
import signal
import argparse

sys.path.append('/opt/ue4_ws/PythonAPI/carla/dist/carla-0.9.13-py2.7-linux-x86_64.egg')
sys.path.append('/opt/ue4_ws/PythonAPI/carla')

import carla

client = None
world = None
ego_vehicle = None
exit_flag = False

def handle_sigint(signum, frame):
    global exit_flag
    print("\n[Ctrl+C] 开始清理资源...")
    exit_flag = True

signal.signal(signal.SIGINT, handle_sigint)


def draw_ego_local_grid(world, ego_transform, grid_len=10.0, grid_wid=6.0, cell=0.5, z=0.05):
    """
    ego局部坐标系网格，跟随车辆yaw旋转
    X：车头向前（长10m），Y：车辆左侧（宽6m）
    grid_len: X方向长度 10m
    grid_wid: Y方向宽度 6m
    cell：单个格子边长0.5m
    """
    yaw_rad = math.radians(ego_transform.rotation.yaw)
    center = ego_transform.location
    half_len = grid_len / 2.0
    half_wid = grid_wid / 2.0
    step_x = int(grid_len / cell)
    step_y = int(grid_wid / cell)

    def local_to_world(lx, ly):
        wx = lx * math.cos(yaw_rad) - ly * math.sin(yaw_rad) + center.x
        wy = lx * math.sin(yaw_rad) + ly * math.cos(yaw_rad) + center.y
        return carla.Location(wx, wy, z)

    # 竖线（沿X方向）：固定局部X值，从Y=-half_wid到Y=+half_wid
    for i in range(step_x + 1):
        lx = -half_len + i * cell
        p1 = local_to_world(lx, -half_wid)
        p2 = local_to_world(lx, half_wid)
        world.debug.draw_line(p1, p2, thickness=0.01,
                             color=carla.Color(0, 0, 0), life_time=0.5)
    # 横线（沿Y方向）：固定局部Y值，从X=-half_len到X=+half_len
    for i in range(step_y + 1):
        ly = -half_wid + i * cell
        p1 = local_to_world(-half_len, ly)
        p2 = local_to_world(half_len, ly)
        world.debug.draw_line(p1, p2, thickness=0.01,
                             color=carla.Color(0, 0, 0), life_time=0.5)


def find_ego_vehicle(world, target_id=-1):
    if target_id > 0:
        vehicle = world.get_actor(target_id)
        if vehicle is not None:
            print("Found vehicle by ID: {} ({})".format(vehicle.id, vehicle.type_id))
            return vehicle
        print("Vehicle ID {} not found".format(target_id))
        return None

    vehicles = world.get_actors().filter("vehicle.*")
    if not vehicles:
        print("No vehicles found in the world")
        return None

    for v in vehicles:
        if "tesla" in v.type_id.lower() or "model3" in v.type_id.lower():
            print("Found Tesla vehicle: {} (id={})".format(v.type_id, v.id))
            return v

    v = vehicles[0]
    print("Found first vehicle: {} (id={})".format(v.type_id, v.id))
    return v


def main(args):
    global client, world, ego_vehicle
    try:
        client = carla.Client("127.0.0.1", 2000)
        client.set_timeout(10.0)
        world = client.get_world()

        ego_vehicle = find_ego_vehicle(world, args.id)
        if ego_vehicle is None:
            print("No ego vehicle found, exit")
            return

        # 主循环
        while not exit_flag:
            ego_tf = ego_vehicle.get_transform()
            draw_ego_local_grid(world, ego_tf, grid_len=10.0, grid_wid=6.0, cell=0.5, z=0.05)
            time.sleep(0.05)

    except Exception as e:
        print("Exception:", e)
    finally:
        if ego_vehicle is not None:
            print("Ego vehicle destroyed")
        print("Program exit")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="BEV环视标定板生成工具 (CARLA 0.9.13)")
    parser.add_argument('--id', type=int, default=-1, 
                        help='指定车辆ID，-1表示自动查找')
    
    args = parser.parse_args()
    main(args)