#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import sys
import numpy as np
import cv2

sys.path.append('/opt/ue4_ws/PythonAPI/carla/dist/carla-0.9.13-py2.7-linux-x86_64.egg')
sys.path.append('/opt/ue4_ws/PythonAPI/carla')

import carla


class CarlaBirdViewVisualizer:
    def __init__(self, world, carla_map, img_width=1400, img_height=1000):
        self.world = world
        self.map = carla_map
        self.img_w = img_width
        self.img_h = img_height

        # 俯视视角参数
        self.center_x = 0.0
        self.center_y = 0.0
        self.view_range = 120.0
        self.scale = self.img_w / (self.view_range * 2)

        # 尝试将初始视角移动到地图中心点
        spawn_points = carla_map.get_spawn_points()
        if spawn_points:
            xs = [sp.location.x for sp in spawn_points]
            ys = [sp.location.y for sp in spawn_points]
            self.center_x = (min(xs) + max(xs)) / 2.0
            self.center_y = (min(ys) + max(ys)) / 2.0

        # 鼠标交互
        self.mouse_drag = False
        self.drag_start_x = 0
        self.drag_start_y = 0
        self.drag_world_x = 0.0
        self.drag_world_y = 0.0

        # 跟随模式相关【功能B】
        self.follow_mode = True
        self.follow_actor_id = None

        # ID 输入模式
        self.input_mode = False
        self.input_buffer = ""
        self._init_follow_target()

        # 车道边界存储
        self.lane_borders = []          # [np.array, ...]
        self._border_cache = []         # 像素坐标缓存
        self._cache_center = (0.0, 0.0)
        self._cache_scale = 0.0
        self._parse_opendrive_lanes()

    def _world2pixel(self, wx, wy):
        px = self.img_w / 2 - (wx - self.center_x) * self.scale
        py = self.img_h / 2 - (wy - self.center_y) * self.scale
        return int(px), int(py)

    def _pixel2world(self, px, py):
        wx = self.center_x - (px - self.img_w / 2) / self.scale
        wy = self.center_y - (py - self.img_h / 2) / self.scale
        return wx, wy

    def _parse_opendrive_lanes(self):
        print("解析OpenDRIVE车道边界...")
        topology = self.map.get_topology()
        sampled_dist = 1.0
        visited_keys = set()

        for wp_start, _ in topology:
            wp = wp_start
            segment_left = []
            segment_right = []
            while True:
                key = (wp.road_id, wp.section_id, wp.lane_id, round(wp.s, 1))
                if key in visited_keys:
                    break
                visited_keys.add(key)

                offset = wp.lane_width / 2.0
                yaw_rad = np.radians(wp.transform.rotation.yaw)
                c, s = np.cos(yaw_rad), np.sin(yaw_rad)

                left_x = wp.transform.location.x - offset * s
                left_y = wp.transform.location.y + offset * c
                right_x = wp.transform.location.x + offset * s
                right_y = wp.transform.location.y - offset * c

                segment_left.append([left_x, left_y])
                segment_right.append([right_x, right_y])

                next_wps = wp.next(sampled_dist)
                if not next_wps:
                    break
                wp = next_wps[0]

            # 至少 2 个点才能画线
            if len(segment_left) >= 2:
                self.lane_borders.append(np.array(segment_left, np.float32))
            if len(segment_right) >= 2:
                self.lane_borders.append(np.array(segment_right, np.float32))

        print("车道边界采样点加载完成，线段数量：%d" % len(self.lane_borders))

    def draw_road_border(self, canvas):
        # 仅当视角变化时重建像素坐标缓存
        if (self.center_x, self.center_y, self.scale) != (self._cache_center[0], self._cache_center[1], self._cache_scale):
            self._cache_center = (self.center_x, self.center_y)
            self._cache_scale = self.scale
            self._border_cache = []
            for pts in self.lane_borders:
                pix_list = []
                for (wx, wy) in pts:
                    px, py = self._world2pixel(wx, wy)
                    pix_list.append([px, py])
                self._border_cache.append(np.array(pix_list, np.int32))
        for arr in self._border_cache:
            cv2.polylines(canvas, [arr], False, (120, 120, 120), thickness=1)

    def draw_rotated_rect(self, canvas, x0, y0, yaw, half_len, half_wid, color, thick=2):
        yaw_rad = np.radians(yaw)
        c, s = np.cos(yaw_rad), np.sin(yaw_rad)
        corners = [
            [ half_len*c - half_wid*s,  half_len*s + half_wid*c],
            [-half_len*c - half_wid*s, -half_len*s + half_wid*c],
            [-half_len*c + half_wid*s, -half_len*s - half_wid*c],
            [ half_len*c + half_wid*s,  half_len*s - half_wid*c],
        ]
        pix_corner = []
        for dx, dy in corners:
            px, py = self._world2pixel(x0 + dx, y0 + dy)
            pix_corner.append([px, py])
        cv2.polylines(canvas, [np.array(pix_corner, np.int32)], True, color, thick)
        return pix_corner[0]

    def draw_all_actors(self, canvas):
        veh_half_l = 2.3
        veh_half_w = 0.9
        walk_half = 0.45

        actors = self.world.get_actors()
        vehicles = actors.filter("vehicle.*")
        walkers = actors.filter("walker.*")

        # 遍历车辆
        for v in vehicles:
            vid = v.id
            t = v.get_transform()
            x0, y0 = t.location.x, t.location.y
            yaw = t.rotation.yaw

            # 速度 km/h
            vel = v.get_velocity()
            speed_m_s = np.sqrt(vel.x**2 + vel.y**2)
            speed_kmh = speed_m_s * 3.6

            # Ego车判定【功能E】：可以自行修改ego筛选条件
            # 方案1：通过角色ID；方案2：根据角色属性区分
            is_ego = ("ego" in v.attributes.get("role_name", ""))

            if is_ego:
                color = (255, 120, 0)   # 蓝色
                thick = 3
            else:
                color = (0, 0, 255)     # 红色NPC车辆
                thick = 2

            # 绘制旋转矩形
            text_pos = self.draw_rotated_rect(canvas, x0, y0, yaw, veh_half_l, veh_half_w, color, thick)
            # 打印ID与速度【功能A】
            txt = "ID:%d %.1f km/h" % (vid, speed_kmh)
            cv2.putText(canvas, txt, (text_pos[0]+6, text_pos[1]-6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255,255,255), 1)
            # 运动方向箭头（从车头向前延伸）
            arrow_len = 3.5
            yaw_rad = np.radians(yaw)
            ax = x0 + np.cos(yaw_rad) * arrow_len
            ay = y0 + np.sin(yaw_rad) * arrow_len
            p_start = self._world2pixel(x0, y0)
            p_end = self._world2pixel(ax, ay)
            cv2.arrowedLine(canvas, p_start, p_end, (0, 255, 255), 2, tipLength=0.3)

        # 行人
        for w in walkers:
            t = w.get_transform()
            x0, y0 = t.location.x, t.location.y
            yaw = t.rotation.yaw
            self.draw_rotated_rect(canvas, x0, y0, yaw, walk_half, walk_half, (0, 220, 0), 2)

        # 红绿灯（三角形）
        traffic_lights = actors.filter("traffic.traffic_light*")
        for tl in traffic_lights:
            t = tl.get_transform()
            x0, y0 = t.location.x, t.location.y
            px, py = self._world2pixel(x0, y0)
            state = tl.get_state()
            if state == carla.TrafficLightState.Red:
                color = (0, 0, 255)       # 红色
            elif state == carla.TrafficLightState.Yellow:
                color = (0, 255, 255)     # 黄色
            elif state == carla.TrafficLightState.Green:
                color = (0, 255, 0)       # 绿色
            else:
                color = (128, 128, 128)   # 灰色（未知）
            # 等边三角形，顶点朝上
            r = 5
            pts = np.array([
                [px, py - r],
                [px - int(r * 0.866), py + int(r * 0.5)],
                [px + int(r * 0.866), py + int(r * 0.5)]
            ], np.int32)
            cv2.fillPoly(canvas, [pts], color)
            cv2.polylines(canvas, [pts], True, (0, 0, 0), 1)  # 黑色描边

    def _init_follow_target(self):
        """初始化跟随目标，优先选择 ego 车辆"""
        vehicles = list(self.world.get_actors().filter("vehicle.*"))
        ego_candidates = [v for v in vehicles if "ego" in v.attributes.get("role_name", "")]
        if ego_candidates:
            self.follow_actor_id = ego_candidates[0].id
            print("默认跟随 ego 车辆, actor_id = %d" % self.follow_actor_id)
        elif vehicles:
            self.follow_actor_id = vehicles[0].id
            print("未找到 ego 车辆, 默认跟随 actor_id = %d" % self.follow_actor_id)
        else:
            print("场景内无车辆，跟随模式等待中...")

    def update_follow_view(self):
        """跟随模式更新视角【功能B】"""
        if not self.follow_mode:
            return
        # 如果目标尚未设置，尝试动态查找 ego
        if self.follow_actor_id is None:
            self._init_follow_target()
        if self.follow_actor_id is None:
            return
        target = self.world.get_actor(self.follow_actor_id)
        if target is not None:
            self.center_x = target.get_location().x
            self.center_y = target.get_location().y

    def switch_follow_target(self):
        """切换下一个跟随目标"""
        vehicles = list(self.world.get_actors().filter("vehicle.*"))
        if len(vehicles) == 0:
            print("场景内无车辆可供跟随！")
            return
        # 优先寻找ego车
        ego_candidates = [v for v in vehicles if "ego" in v.attributes.get("role_name","")]
        if ego_candidates:
            ego_ids = [v.id for v in ego_candidates]
            if self.follow_actor_id in ego_ids:
                idx = ego_ids.index(self.follow_actor_id)
                new_idx = (idx + 1) % len(ego_ids)
                self.follow_actor_id = ego_ids[new_idx]
            else:
                self.follow_actor_id = ego_ids[0]
        else:
            veh_ids = [v.id for v in vehicles]
            if self.follow_actor_id in veh_ids:
                idx = veh_ids.index(self.follow_actor_id)
                new_idx = (idx + 1) % len(veh_ids)
                self.follow_actor_id = veh_ids[new_idx]
            else:
                self.follow_actor_id = veh_ids[0]
        print("跟随目标切换为 actor_id = %d" % self.follow_actor_id)

    def render(self):
        canvas = np.zeros((self.img_h, self.img_w, 3), dtype=np.uint8)
        self.update_follow_view()
        self.draw_road_border(canvas)
        self.draw_all_actors(canvas)

        # 状态文字
        follow_state = "ON" if self.follow_mode else "OFF"
        cv2.putText(canvas, "Follow:%s | TargetID:%s" % (follow_state, str(self.follow_actor_id)),
                    (20,75), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (220,220,0), 2)
        cv2.putText(canvas, "f: check follow | n: switch target | ESC: exit",
                    (20,110), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180,180,180),1)
        # ID 输入提示
        if self.input_mode:
            prompt = "Input Target ID: %s_" % self.input_buffer
            cv2.putText(canvas, prompt, (20, self.img_h - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        return canvas

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.mouse_drag = True
            self.drag_start_x = x
            self.drag_start_y = y
            self.drag_world_x, self.drag_world_y = self._pixel2world(x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self.mouse_drag:
            wx, wy = self._pixel2world(x, y)
            self.center_x += (self.drag_world_x - wx)
            self.center_y += (self.drag_world_y - wy)
            # 手动拖拽自动关闭跟随
            self.follow_mode = False
        elif event == cv2.EVENT_LBUTTONUP:
            self.mouse_drag = False
        elif event == cv2.EVENT_MOUSEWHEEL:
            # flags > 0 滚轮向上 → 放大; flags < 0 滚轮向下 → 缩小
            if flags > 0:
                self.scale *= 1.1
            else:
                self.scale *= 0.9
            self.scale = np.clip(self.scale, 0.2, 15)


def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()

    vis = CarlaBirdViewVisualizer(world, carla_map, img_width=1400, img_height=1000)
    win_name = "CARLA World BirdView"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(win_name, vis.mouse_callback)

    fps_counter = 0
    t_start = cv2.getTickCount()
    fps = 0.0

    try:
        while True:
            frame = vis.render()
            # FPS计算
            fps_counter += 1
            now_t = cv2.getTickCount()
            dt = (now_t - t_start) / cv2.getTickFrequency()
            if dt > 1.0:
                fps = fps_counter / dt
                fps_counter = 0
                t_start = now_t
            cv2.putText(frame, "FPS:%.1f" % fps, (20,40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255),2)

            cv2.imshow(win_name, frame)
            key = cv2.waitKey(1) & 0xFF
            if vis.input_mode:
                if key == 27:  # ESC 取消
                    vis.input_mode = False
                    vis.input_buffer = ""
                elif key == 13:  # Enter 确认
                    try:
                        new_id = int(vis.input_buffer)
                        vis.follow_actor_id = new_id
                        vis.follow_mode = True
                        print("跟随目标切换为 actor_id = %d" % new_id)
                    except ValueError:
                        print("无效 ID: %s" % vis.input_buffer)
                    vis.input_mode = False
                    vis.input_buffer = ""
                elif key == 8:  # Backspace
                    vis.input_buffer = vis.input_buffer[:-1]
                elif ord('0') <= key <= ord('9'):
                    vis.input_buffer += chr(key)
            elif key == 27:
                break
            elif key == ord('f'):
                vis.follow_mode = not vis.follow_mode
                print("跟随模式: %s" % ("开启" if vis.follow_mode else "关闭"))
            elif key == ord('n'):
                vis.input_mode = True
                vis.input_buffer = ""
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
