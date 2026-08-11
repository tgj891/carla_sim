import cv2
import numpy as np
from PIL import Image
import torch
from config import *

from transformers import SegformerImageProcessor, SegformerForSemanticSegmentation
np.set_printoptions(threshold=np.inf)

# 加载预训练SegFormer (ADE20K 语义分割)
processor = SegformerImageProcessor.from_pretrained("./weight/segformer-b2-finetuned-ade-512-512")
model = SegformerForSemanticSegmentation.from_pretrained("./weight/segformer-b2-finetuned-ade-512-512").cuda()
model.eval()

def predict_semantic_seg(rgb_frame):
    """
    rgb_frame: BGR numpy数组(OpenCV读取) H,W,3
    return: seg_mask(类别索引图), seg_vis(彩色可视化图)
    """
    # 转RGB
    image_rgb = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2RGB)
    inputs = processor(images=image_rgb, return_tensors="pt").to("cuda")

    with torch.no_grad():
        outputs = model(**inputs)

    # 核心：argmax 获取类别索引 0~36
    logits = outputs.logits
    pred_mask = torch.argmax(logits, dim=1).squeeze().cpu().numpy()

    # 截断兜底（防止异常数值，杜绝索引越界）
    pred_mask = np.clip(pred_mask, 0, ADE_PALETTE.shape[0]-1)

    h,w = rgb_frame.shape[:2]
    pred_mask = cv2.resize(pred_mask.astype(np.float32), (w,h), interpolation=cv2.INTER_NEAREST)
    pred_mask = pred_mask.astype(np.int32)

    # 上色可视化
    seg_vis = ADE_PALETTE[pred_mask]
    seg_vis = seg_vis.astype(np.uint8)
    
    return pred_mask, seg_vis

# ============ 选择模式 ============
MODE = "image"  # camera / image

if MODE == "image":
    img = cv2.imread("four_images.png")
    mask, vis = predict_semantic_seg(img)
    cv2.imwrite("semantic_output.png", vis)
    cv2.imshow("semantic", vis)
    cv2.waitKey(0)
else:
    cap = cv2.VideoCapture(0)
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret: break
        seg_mask, seg_vis = predict_semantic_seg(frame)
        combine = np.hstack([frame, seg_vis])
        cv2.imshow("Semantic SegFormer", combine)
        if cv2.waitKey(1) & 0xFF == 27:
            break
    cap.release()
cv2.destroyAllWindows()