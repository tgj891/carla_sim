import socket
import numpy as np
import cv2
import torch
from utils.utils import \
    time_synchronized,select_device, increment_path,\
    scale_coords,xyxy2xywh,non_max_suppression,split_for_trace_model,\
    driving_area_mask,lane_line_mask,plot_one_box,show_seg_result,\
    letterbox

FOURE_FUSION = True 
TCP_PORT = 9999
HOST = "127.0.0.1"
# 四合一的图像尺寸
IMG_HEIGHT = 960
IMG_WIDTH = 1280

# 相机内参
image_height = 480
image_width = 640
cx = image_width / 2
cy = image_height / 2
fx = 184.7520861406803
fy = 184.7520861406803
#

# load model
stride =32
imgsz = 640
agnostic_nms = True
classes = None
iou_thres = 0.45
conf_thres = 0.3
weights  = 'weight/yolopv2/yolopv2.pt'
model  = torch.jit.load(weights)
device = select_device('0')
half = device.type != 'cpu'  # half precision only supported on CUDA
model = model.to(device)

if half:
    model.half()  # to FP16  
model.eval()

if device.type != 'cpu':
    dummy = torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters()))  # run once
    with torch.no_grad():
        _ = model(dummy)
    del dummy
def recv_all(sock, length):
    buffer = b""
    while len(buffer) < length:
        chunk = sock.recv(length - len(buffer))
        if not chunk:
            raise ConnectionError("Server disconnected")
        buffer += chunk
    return buffer


def connect_socket():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((HOST, TCP_PORT))
    print("Connected to server at {}:{}".format(HOST, TCP_PORT))
    return sock
def cv2_crop(img, x1, y1, x2, y2):
    """
    OpenCV图像裁剪
    :param img: cv2读取的图像
    :param x1,y1: 左上角坐标
    :param x2,y2: 右下角坐标
    :return: 裁剪后图像
    """
    return img[y1:y2, x1:x2]

def main():
    while True:
        try:
            sock = connect_socket()
            while True:
                header_buf = recv_all(sock, 16)
                data_len = int(header_buf.strip())

                buffer = recv_all(sock, data_len)

                if FOURE_FUSION:
                    img = np.frombuffer(buffer, dtype=np.uint8).reshape(IMG_HEIGHT, IMG_WIDTH, 3).copy()
                    front_image = img
                else:
                    img = np.frombuffer(buffer, dtype=np.uint8).reshape(image_height, image_width, 3).copy()
                    front_image = img

                # Padded resize
                front_image = cv2.resize(front_image, (1280,720), interpolation=cv2.INTER_LINEAR)
                infer_image = letterbox(front_image, imgsz, stride=stride)[0]
        
                # Convert
                infer_image = infer_image[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
                infer_image = np.ascontiguousarray(infer_image)
                infer_tensor = torch.from_numpy(infer_image).to(device)
                infer_tensor = infer_tensor.half() if half else infer_tensor.float()  # uint8 to fp16/32
                infer_tensor /= 255.0  # 0 - 255 to 0.0 - 1.0
        
                if infer_tensor.ndimension() == 3:
                    infer_tensor = infer_tensor.unsqueeze(0)
                    
                with torch.no_grad():
                    [pred,anchor_grid],seg,ll= model(infer_tensor)
                    
                pred = split_for_trace_model(pred,anchor_grid)
                pred = non_max_suppression(pred, conf_thres, iou_thres, classes=classes, agnostic=agnostic_nms)
                
                da_seg_mask = driving_area_mask(seg)
                ll_seg_mask = lane_line_mask(ll)
        
                # Process detections
                for i, det in enumerate(pred):  # detections per image
                    if len(det):
                        # Rescale boxes from img_size to im0 size
                        det[:, :4] = scale_coords(infer_tensor.shape[2:], det[:, :4], front_image.shape).round()
        
                        # Write results
                        for *xyxy, conf, cls in reversed(det):
                            plot_one_box(xyxy, front_image, line_thickness=3)
        
                    # Print time (inference)
                    show_seg_result(front_image, (da_seg_mask,ll_seg_mask), is_demo=True)
        
                cv2.imshow("Carla RGB", front_image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    cv2.destroyAllWindows()
                    sock.close()
                    return

                del infer_tensor, pred, seg, ll, da_seg_mask, ll_seg_mask
                if device.type != "cpu":
                    torch.cuda.empty_cache()
                    
        except ConnectionError:
            print("Connection lost, reconnecting...")
            cv2.destroyAllWindows()
            import time
            time.sleep(1)
        except Exception as e:
            print("Error:", e)
            break

    cv2.destroyAllWindows()
    torch.cuda.empty_cache()

if __name__ == "__main__":
    main()