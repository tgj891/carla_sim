#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
在CARLA中生成BEV环视标定用的棋盘格标定板（CARLA 0.9.13兼容版）
只在车辆前后左右4个方向生成标定板
使用 world.debug.draw_box 每帧重绘，确保可见
"""
import sys
import argparse
import time

sys.path.append('/opt/ue4_ws/PythonAPI/carla/dist/carla-0.9.13-py2.7-linux-x86_64.egg')
sys.path.append('/opt/ue4_ws/PythonAPI/carla')

import carla


def draw_checkerboard(world, center_loc, center_yaw, board_size=(1.0, 1.0), grid=(4, 4), thickness=0.005):
    """
    使用 debug.draw_box 绘制棋盘格标定板
    :param world: carla.World
    :param center_loc: 标定板中心位置 carla.Location
    :param center_yaw: 标定板朝向角度（度）
    :param board_size: 标定板总尺寸 (宽, 高) 米
    :param grid: 棋盘格行列数 (列数, 行数)
    :param thickness: 标定板厚度（Z轴）
    """
    w, h = board_size
    cols, rows = grid
    cell_w = w / cols
    cell_h = h / rows
    
    for i in range(cols):
        for j in range(rows):
            x = (i - cols/2 + 0.5) * cell_w
            y = (j - rows/2 + 0.5) * cell_h
            z = thickness / 2

            loc = carla.Location(
                x=center_loc.x + x,
                y=center_loc.y + y,
                z=center_loc.z + z
            )

            # color = carla.Color(0, 0, 0) if (i+j) % 2 == 0 else carla.Color(255, 255, 255)
            color = carla.Color(0, 0, 0)
            
            bbox = carla.BoundingBox(loc, carla.Vector3D(cell_w/2, cell_h/2, thickness/2))
            world.debug.draw_box(
                box=bbox,
                rotation=carla.Rotation(yaw=center_yaw),
                thickness=0,
                color=color,
                life_time=0.1
            )


class CheckerboardCalibrator:
    def __init__(self, host='localhost', port=2000):
        self.client = carla.Client(host, port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.ego_vehicle = None
        self.running = True
        # offset是棋盘格中心相对于车辆中心的外参
        self.configs = [
            {"name": "front", "offset": (3.25,  0.0, 0.05), "yaw": 0.0},
            {"name": "back",  "offset": (-3.25, 0.0, 0.05), "yaw": 180.0},
            {"name": "left",  "offset": (0.0,  -1.75, 0.05), "yaw": -90.0},
            {"name": "right", "offset": (0.0, 1.75, 0.05), "yaw": 90.0},
        ]

    def find_ego_vehicle(self, filter_pattern='vehicle.*'):
        """查找自车车辆"""
        actors = self.world.get_actors().filter(filter_pattern)
        for actor in actors:
            print("Found vehicle: {} (id={})".format(actor.type_id, actor.id))
            return actor
        return None
    
    def get_board_transform(self, cfg):
        """计算标定板的变换"""
        if self.ego_vehicle:
            vehicle_transform = self.ego_vehicle.get_transform()
            relative_loc = carla.Location(x=cfg["offset"][0], 
                                          y=cfg["offset"][1], 
                                          z=cfg["offset"][2])
            board_loc = vehicle_transform.transform(relative_loc)
            board_yaw = vehicle_transform.rotation.yaw + cfg["yaw"]
        else:
            board_loc = carla.Location(x=cfg["offset"][0], 
                                      y=cfg["offset"][1], 
                                      z=cfg["offset"][2])
            board_yaw = cfg["yaw"]
        return board_loc, board_yaw
    
    def draw_all_boards(self):
        """绘制所有标定板"""
        for cfg in self.configs:
            board_loc, board_yaw = self.get_board_transform(cfg)
            grid_size = (4, 4)
            board_size = (1.0, 1.0)
            draw_checkerboard(
                world=self.world,
                center_loc=board_loc,
                center_yaw=board_yaw,
                board_size=board_size,
                grid=grid_size,
                thickness=0.005
            )
    
    def run(self):
        """主循环，持续绘制标定板"""
        print("Starting checkerboard drawing loop...")
        frame_count = 0
        
        while self.running:
            self.draw_all_boards()
            
            frame_count += 1
            if frame_count % 30 == 0:
                if self.ego_vehicle:
                    v_loc = self.ego_vehicle.get_transform().location
                    print("Frame {} | Vehicle at ({:.2f}, {:.2f}, {:.2f})".format(
                        frame_count, v_loc.x, v_loc.y, v_loc.z))
                else:
                    print("Frame {} | No vehicle found".format(frame_count))
            
            time.sleep(0.05)


def main():
    parser = argparse.ArgumentParser(description="BEV环视标定板生成工具 (CARLA 0.9.13)")
    parser.add_argument('--host', default='localhost', help='Carla服务器地址')
    parser.add_argument('--port', type=int, default=2000, help='Carla服务器端口')
    parser.add_argument('--vehicle-id', type=int, default=-1, 
                        help='指定车辆ID，-1表示自动查找')
    
    args = parser.parse_args()
    
    calibrator = CheckerboardCalibrator(args.host, args.port)
    
    if args.vehicle_id > 0:
        calibrator.ego_vehicle = calibrator.world.get_actor(args.vehicle_id)
        if calibrator.ego_vehicle:
            print("Using specified vehicle: {} (id={})".format(
                calibrator.ego_vehicle.type_id, calibrator.ego_vehicle.id))
        else:
            print("Vehicle with id {} not found, trying auto-detection".format(args.vehicle_id))
            calibrator.ego_vehicle = calibrator.find_ego_vehicle()
    else:
        calibrator.ego_vehicle = calibrator.find_ego_vehicle()
 
    if calibrator.ego_vehicle:
        v_transform = calibrator.ego_vehicle.get_transform()
        print("Vehicle transform: pos=({:.2f}, {:.2f}, {:.2f}), yaw={:.1f}".format(
            v_transform.location.x, v_transform.location.y, v_transform.location.z,
            v_transform.rotation.yaw))
    else:
        print("⚠️ 未找到车辆，将在固定位置生成标定板")
        print("固定位置: front(5,0), back(-5,0), left(0,3), right(0,-3)")
    
    print("\n✅ 开始绘制4个标定板（前、后、左、右）")
    print("按 Ctrl+C 退出")
    
    try:
        calibrator.run()
    except KeyboardInterrupt:
        calibrator.running = False
        print("\n退出")


if __name__ == '__main__':
    main()
