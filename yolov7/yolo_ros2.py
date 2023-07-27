import time
import cv2
import torch
import numpy as np

from numpy import random
from cv_bridge import CvBridge
from models.experimental import attempt_load
from utils.datasets import letterbox
from utils.general import check_img_size, check_requirements, non_max_suppression, scale_coords
from utils.plots import plot_one_box
from utils.torch_utils import select_device, time_synchronized

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray

WEIGHTS = 'weights/delivery_t.pt'
IMG_SIZE = 640
DEVICE = ''
AUGMENT = False
CONF_THRES = 0.70
IOU_THRES = 0.45
CLASSES = None
AGNOSTIC_NMS = False

# Initialize
device = select_device(DEVICE)
half = device.type != 'cpu'  # half precision only supported on CUDA
print('device:', device)

# Load model
model = attempt_load(WEIGHTS, map_location=device)  # load FP32 model
stride = int(model.stride.max())  # model stride
imgsz = check_img_size(IMG_SIZE, s=stride)  # check img_size
if half:
    model.half()  # to FP16

# Get names and colors
names = model.module.names if hasattr(model, 'module') else model.names
colors = [[random.randint(0, 255) for _ in range(3)] for _ in names]

# Run inference
if device.type != 'cpu':
    model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))  # run once

class YOLOv7(Node):
    def __init__(self):

        super().__init__('yolo_ros2')

        self.detected_pub = self.create_publisher(Image, "/detected_image", 10)
        self.yolo_pub = self.create_publisher(Int32MultiArray, "/delivery_sign", 10) # [id, xpose]

        self.image_sub = self.create_subscription(Image, "/image_raw", self.image_cb, 10)

    def image_cb(self, img):
        check_requirements(exclude=('pycocotools', 'thop'))
        with torch.no_grad():
            bridge = CvBridge()
            cap = bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")

            result = self.detect(cap)
            image_message = bridge.cv2_to_imgmsg(result, encoding="bgr8")
            image_message.header.stamp = self.get_clock().now().to_msg()
            self.detected_pub.publish(image_message)

    # Detect function
    def detect(self, frame):
        # Load image
        img0 = frame

        # Padded resize
        img = letterbox(img0, imgsz, stride=stride)[0]

        # Convert
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
        img = np.ascontiguousarray(img)

        img = torch.from_numpy(img).to(device)
        img = img.half() if half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)


        # Inference
        t0 = time_synchronized()
        pred = model(img, augment=AUGMENT)[0]

        # Apply NMS
        pred = non_max_suppression(pred, CONF_THRES, IOU_THRES, classes=CLASSES, agnostic=AGNOSTIC_NMS)

        # Process detections
        det = pred[0]
        numClasses = len(det)

        s = ''
        s += '%gx%g ' % img.shape[2:]  # print string

        boundingBoxes = Int32MultiArray()
        boundingBoxes.data = []
        box = []

        if numClasses:
            # Rescale boxes from img_size to img0 size
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], img0.shape).round()

            # Print results
            for c in det[:, -1].unique():
                n = (det[:, -1] == c).sum()  # detections per class
                s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

            # Write results
            for *xyxy, conf, cls in reversed(det):
                label = f'{names[int(cls)]} {conf:.2f}'
                plot_one_box(xyxy, img0, label=label, color=colors[int(cls)], line_thickness=3)
                xmin, ymin, xmax, ymax = [int(tensor.item()) for tensor in xyxy]

                box_ = [int(cls), xmin, ymin, xmax, ymax]
                for b_ in range(len(box_)):
                    box.append(box_[b_])

            boundingBoxes.data = box

        self.yolo_pub.publish(boundingBoxes)

        # return results
        return img0

def main(args=None):
    rclpy.init(args=args)

    try:
        yolo = YOLOv7()

        rclpy.spin(yolo)

    finally:
        yolo.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
