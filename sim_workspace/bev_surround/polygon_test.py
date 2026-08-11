import cv2
import numpy as np

def copy_polygon_roi_same_size(img_src, img_dst, polygon):
    """
    【两张图像尺寸完全相同】多边形区域复制
    :param img_src: 源图像 BGR
    :param img_dst: 目标图像 BGR
    :param polygon: 多边形顶点 np.array(N,2) int32, 闭合轮廓
    :return: 复制完成后的目标图像（新副本，不修改原输入）
    """
    # 保护：创建副本，避免修改原始图片
    dst_out = img_dst.copy()
    h, w = img_src.shape[:2]

    # 校验两张图尺寸一致
    assert img_src.shape[:2] == img_dst.shape[:2], "源图与目标图像宽高必须一致！"

    # 创建掩码
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.fillPoly(mask, [polygon.astype(np.int32)], 255)

    # 源图提取多边形区域
    roi_region = cv2.bitwise_and(img_src, img_src, mask=mask)

    # 将多边形区域覆盖写入目标图
    # mask==255 的位置，使用源图像像素；其余位置保持原图不变
    dst_out[mask > 0] = roi_region[mask > 0]
    return dst_out


if __name__ == "__main__":
    # 1. 读取两张【相同分辨率】图片
    imgA = cv2.imread("front.png")
    imgB = cv2.imread("back.png")

    # 2. 定义共用多边形顶点（四边形/任意多边形均可）
    poly_points = np.array([
        [320, 210],
        [640, 200],
        [660, 720],
        [290, 740]
    ], dtype=np.int32)

    # 3. 执行复制
    result = copy_polygon_roi_same_size(imgA, imgB, poly_points)

    cv2.imshow("source", imgA)
    cv2.imshow("target original", imgB)
    cv2.imshow("result", result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    cv2.imwrite("output.png", result)