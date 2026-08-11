import cv2
import numpy as np
from config import *

class ManualCornerPicker:
    def __init__(self, image):
        self.image = image.copy()
        self.draw_image = image.copy()
        self.pattern_size = GRID_SIZE
        self.target_corners = (GRID_SIZE[0]-1) * (GRID_SIZE[1]-1)
        self.corners = []
        self.world_corners = []
        self.current_idx = 0
        self.selected_idx = 0
        self.done = False
        
        self.window_name = "Manual Corner Picker"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 800, 600)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)
        
        self.show_instructions()
    
    def show_instructions(self):
        print("=" * 60)
        print("手动角点选择工具")
        print("=" * 60)
        print("棋盘格规格: {}x{} = {} 个角点".format(
            self.pattern_size[0]-1, self.pattern_size[1]-1, self.target_corners))
        print("点击顺序: 从左上角开始，逐行从左到右点击")
        print("-" * 60)
        print("操作说明:")
        print("  左键点击: 选择角点")
        print("  右键点击: 删除最后一个角点")
        print("  r 键: 重置所有角点")
        print("  空格/回车: 完成选择")
        print("  ESC: 退出")
        print("=" * 60)
    
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.current_idx < self.target_corners:
                self.corners.append((x, y))
                self.current_idx += 1
                self.redraw()
                print("角点 {}: ({}, {})".format(self.current_idx, x, y))
        
        elif event == cv2.EVENT_RBUTTONDOWN:
            if self.current_idx > 0:
                removed = self.corners.pop()
                self.current_idx -= 1
                self.redraw()
                print("撤销角点 {}: ({}, {})".format(self.current_idx + 1, removed[0], removed[1]))
    
    def redraw(self):
        self.draw_image = self.image.copy()
        
        for i, (x, y) in enumerate(self.corners):
            color = (0, 255, 0) if i == self.current_idx - 1 else (0, 0, 255)
            cv2.circle(self.draw_image, (x, y), 6, color, -1)
            cv2.putText(self.draw_image, str(i + 1), (x + 8, y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        cv2.putText(self.draw_image,
                    "已选择: {}/{}".format(self.current_idx, self.target_corners),
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        cv2.imshow(self.window_name, self.draw_image)
    
    def run(self):
        cv2.imshow(self.window_name, self.draw_image)
        
        while not self.done:
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('r'):
                self.corners = []
                self.current_idx = 0
                self.redraw()
                print("已重置所有角点")
            
            elif key == ord(' ') or key == 13:
                if self.current_idx == self.target_corners:
                    self.done = True
                    print("\n角点选择完成!")
                else:
                    print("还需要选择 {} 个角点".format(self.target_corners - self.current_idx))
            
            elif key == 27:
                print("用户退出")
                self.corners = []
                self.done = True
        
        cv2.destroyWindow(self.window_name)
        return self.corners


def build_chessboard_world_points(camera_name):
    objp = []
    HALF_COL = GRID_SIZE[0] / 2
    HALF_ROW = GRID_SIZE[1] / 2
    bev_car_w = CAR_WIDTH / RESOLUTION
    bev_car_l = CAR_LENGTH / RESOLUTION
    bev_half_h = BEV_HEIGHT / 2
    bev_half_w = BEV_WIDTH / 2
    bev_car_half_w = bev_car_w / 2
    bev_car_half_l = bev_car_l / 2
    grid_side_length = float(GRID_WIDTH) / float(GRID_SIZE[0])
    for row in range(GRID_SIZE[1]-1):
        for col in range(GRID_SIZE[0]-1):
            if camera_name == "front":
                x = GRIDS_CONFIGS["front"][0] + grid_side_length * (1 - row)
                y = GRIDS_CONFIGS["front"][1] + grid_side_length * (1 - col)
                z = 0.0
                bev_x = bev_half_w - y / RESOLUTION
                bev_y = bev_half_h - x / RESOLUTION
                objp.append([bev_x, bev_y, z])
            elif camera_name == "back":
                x = GRIDS_CONFIGS["back"][0] + grid_side_length * (row - 1)
                y = GRIDS_CONFIGS["back"][1] + grid_side_length * (col - 1)
                z = 0.0
                
                bev_x = bev_half_w - y / RESOLUTION
                bev_y = bev_half_h - x / RESOLUTION
                objp.append([bev_x, bev_y, z])
            elif camera_name == "left":
                x = GRIDS_CONFIGS["left"][0] + grid_side_length * (col - 1)
                y = GRIDS_CONFIGS["left"][1] + grid_side_length * (1 - row)
                z = 0.0
                bev_x = bev_half_w - y / RESOLUTION
                bev_y = bev_half_h - x / RESOLUTION
                objp.append([bev_x, bev_y, z])
            elif camera_name == "right":
                x = GRIDS_CONFIGS["right"][0] + grid_side_length * (1 - col)
                y = GRIDS_CONFIGS["right"][1] + grid_side_length * (row - 1)
                z = 0.0
                bev_x = bev_half_w - y / RESOLUTION
                bev_y = bev_half_h - x / RESOLUTION
                objp.append([bev_x, bev_y, z])

    return objp


def bev_by_homography(img, img_corners, world_points, bev_w=800, bev_h=600):
    H, mask = cv2.findHomography(img_corners, world_points, cv2.RANSAC)
    bev_img = cv2.warpPerspective(img, H, (bev_w, bev_h))
    return bev_img, H

# ========== 保存H矩阵 ==========
def save_homography(H, save_path):
    np.save(save_path, H)
    print(f"H矩阵已保存至: {save_path}")

# ========== 加载H矩阵 ==========
def load_homography(load_path="homography.npy"):
    H = np.load(load_path)
    print(f"H矩阵加载完成\n{H}")
    return H

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="手动角点选择工具")
    parser.add_argument("--image", default="grid_board.png", help="输入图像路径")
    parser.add_argument("--name", type=str, default="front", help="相机名称")
    parser.add_argument("--output", default=None, help="角点坐标输出文件")
    args = parser.parse_args()
    
    img = cv2.imread(args.image)
    if img is None:
        raise Exception("图片读取失败: {}".format(args.image))
    
    picker = ManualCornerPicker(img)
    corners = picker.run()
    
    if corners:
        corners_np = np.array(corners, dtype=np.float32).reshape(-1, 1, 2)
        world_corners = build_chessboard_world_points(args.name)
        world_corners_np = np.array(world_corners, dtype=np.float32).reshape(-1, 1, 3)
        print("\n" + "=" * 60)
        print("角点坐标列表 (图像像素坐标):")
        print("=" * 60)
        for i, (x, y) in enumerate(corners):
            print("角点 {:2d}: ({:6.1f}, {:6.1f})".format(i + 1, x, y))
            print("world: {}".format(world_corners[i]))
        
        print("\n" + "=" * 60)
        print("OpenCV格式 (N, 1, 2):")
        print("=" * 60)
        print(corners_np)
        
        if args.output:
            np.save(args.output, corners_np)
            print("\n角点坐标已保存到: {}".format(args.output))
        
        bev_img, H = bev_by_homography(img, corners_np, world_corners_np, bev_w=BEV_WIDTH, bev_h=BEV_HEIGHT)
        
        print("\n" + "=" * 60)
        print("H矩阵 (单应性矩阵):")
        print("=" * 60)
        print(H)
        #保存H矩阵
        save_homography(H, f"data/{args.name}_homography.npy")
        
        draw_img = img.copy()
        for pt in corners_np:
            x, y = pt[0]
            cv2.circle(draw_img, (int(x), int(y)), 5, (0, 0, 255), -1)
        
        cv2.namedWindow("Selected Corners", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Selected Corners", 800, 600)
        cv2.imshow("Selected Corners", draw_img)
        
        cv2.namedWindow("BEV Result", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("BEV Result", 800, 600)
        cv2.imshow("BEV Result", bev_img)
        
        cv2.waitKey(0)
        cv2.destroyAllWindows()
