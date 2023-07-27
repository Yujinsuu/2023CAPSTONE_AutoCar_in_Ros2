import time
import cv2
import torch
import statistics
import numpy as np

from numpy import random
from cv_bridge import CvBridge
from PIL import Image as IM
from models.experimental import attempt_load
from utils.datasets import letterbox
from utils.general import check_img_size, check_requirements, non_max_suppression, scale_coords
from utils.plots import plot_one_box
from utils.torch_utils import select_device, time_synchronized

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, String

WEIGHTS = 'weights/delivery_t.pt'
IMG_SIZE = 640
DEVICE = ''
AUGMENT = False
CONF_THRES = 0.60
IOU_THRES = 0.45
CLASSES = None
AGNOSTIC_NMS = False

QUEUE_SIZE = 13
CLASS_MAP = (
    ("id_0", "id_1", "id_2"),
    ("A1",),
    ("A2",),
    ("A3",),
    ("B1",),
    ("B2",),
    ("B3",)
    )

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


# Tesseract Parameter
from PIL import Image as IMG
import argparse

import torch
import torch.backends.cudnn as cudnn
import torch.utils.data
import torch.nn.functional as F

from util import CTCLabelConverter, AttnLabelConverter
from dataset import EasyData, AlignCollate
from model import Model

OPT = argparse.Namespace()
OPT.FeatureExtraction = 'VGG'
OPT.PAD = False
OPT.Prediction = 'CTC'
OPT.SequenceModeling = 'BiLSTM'
OPT.Transformation = 'None'
OPT.batch_max_length = 2
OPT.batch_size = 192
OPT.character = 'A123AB123AB123AB123AB123AB123AB123AB'
# OPT.character = 'a1','a2','a3','b1','b2','b3','a1','a2','a3','b1','b2','b3','a1','a2','a3','b1','b2','b3','a1','a2','a3','b1','b2','b3','a1','a2','a3','b1','b2','b3','a1','a2','a3','b1','b2','b3'
OPT.hidden_size = 256
OPT.imgH = 32
OPT.imgW = 100
OPT.input_channel = 1
OPT.num_fiducial = 20
OPT.output_channel = 512
OPT.rgb = False
OPT.saved_model = 'saved_models/best_accuracy.pth'
OPT.sensitive = False
OPT.workers = 4
cudnn.benchmark = True
cudnn.deterministic = True
OPT.num_gpu = torch.cuda.device_count()
converter = CTCLabelConverter(OPT.character)
OPT.num_class = len(converter.character)

# model configuration
ocr_model = Model(OPT)
ocr_model = torch.nn.DataParallel(ocr_model).to(device)

# load model
print('loading pretrained model from %s' % OPT.saved_model)
ocr_model.load_state_dict(torch.load(OPT.saved_model, map_location = device))

class YOLOv7(Node):
    def __init__(self):

        super().__init__('delivery')

        self.detected_pub = self.create_publisher(Image, "/detected_image", 10)
        self.delivery_pub = self.create_publisher(Int32MultiArray, "/delivery_sign", 10)

        self.mode_sub = self.create_subscription(String, "/yolo_mode", self.mode_cb, 10)
        self.image_sub = self.create_subscription(Image, "/image_raw", self.image_cb, 10)

        self.mode = 'global'
        self.num = 0
        self.text = []

        # [[A queue], [B1 queue], [B2 queue], ...]
        self.queue_list = [[-1 for i in range(QUEUE_SIZE)] for j in range(len(CLASS_MAP))]

        # [[A1 to A queue], [A2 to A queue], [A3 to A queue], [B1 to B1 queue], [B2 to B2 queue], [B3 to B3 queue], ...]
        self.id_to_queue_list = [self.queue_list[i] for i in range(len(CLASS_MAP)) for _ in range(len(CLASS_MAP[i]))]

        self.timer = self.create_timer(0.1, self.yolo_pub)

    def mode_cb(self, msg):
        self.mode = msg.data

    def image_cb(self, img):
        check_requirements(exclude=('pycocotools', 'thop'))
        with torch.no_grad():
            bridge = CvBridge()
            cap = bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")

            if self.mode == 'delivery':
                result = self.detect(cap)
                image_message = bridge.cv2_to_imgmsg(result, encoding="bgr8")
            else:
                image_message = bridge.cv2_to_imgmsg(cap, encoding="bgr8")

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

        if numClasses:
            # Rescale boxes from img_size to img0 size
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], img0.shape).round()

            # Print results
            for c in det[:, -1].unique():
                n = (det[:, -1] == c).sum()  # detections per class
                s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

            images = []
            # Write results
            for *xyxy, conf, cls in reversed(det):
                id = int(cls)
                label = f'{names[id]} {conf:.2f}'
                plot_one_box(xyxy, img0, label=label, color=colors[id], line_thickness=3)
                xmin, ymin, xmax, ymax = [int(tensor.item()) for tensor in xyxy]

                xmean = (xmin + xmax) / 2
                height = max(ymax - ymin, 1)
                width = max(xmax - xmin, 1)

                if xmean > 50 and xmean < IMG_SIZE - 50 and height >= 100 and width / height > 0.8:

                    self.id_to_queue_list[id + 3].append(int(xmean))

                    roi = frame[ymin:ymax, xmin:xmax]
                    reimg = cv2.resize(roi, (OPT.imgW, OPT.imgH))
                    gray = cv2.cvtColor(reimg, cv2.COLOR_BGR2GRAY)

                    # Reshape roi_img if necessary
                    # ocr_img = np.expand_dims(gray, axis=2)  # Add batch dimension if missing
                    ocr_img = IM.fromarray(gray)
                    images.append(ocr_img)

                    if id in (0, 1, 2):
                        self.id_to_queue_list[0].append(id)

                else:
                    for queue in self.queue_list:
                        if len(queue) == QUEUE_SIZE: # append -1 to an undetected classes
                            queue.append(-1)

            if len(images) != 0:
                AlignCollate_demo = AlignCollate(imgH=OPT.imgH, imgW=OPT.imgW, keep_ratio_with_pad=OPT.PAD)
                demo_data = EasyData(images)
                demo_loader = torch.utils.data.DataLoader(
                    demo_data, batch_size=OPT.batch_size,
                    shuffle=False,
                    num_workers=int(OPT.workers),
                    collate_fn=AlignCollate_demo, pin_memory=True)
                self.easyOCR(demo_loader)

        else:
            for queue in self.queue_list:
                if len(queue) == QUEUE_SIZE: # append -1 to an undetected classes
                    queue.append(-1)

        # return results
        return img0

    # Tesseract
    def easyOCR(self, loader):
        with torch.no_grad():
            for image_tensors, img_list in loader:
                batch_size = image_tensors.size(0)
                image = image_tensors.to(device)

                text_for_pred = torch.LongTensor(batch_size, OPT.batch_max_length + 1).fill_(0).to(device)

                preds = ocr_model(image, text_for_pred)

                preds_size = torch.IntTensor([preds.size(1)] * batch_size)
                _, preds_index = preds.max(2)
                preds_str = converter.decode(preds_index, preds_size)

                preds_prob = F.softmax(preds, dim=2)
                preds_max_prob, _ = preds_prob.max(dim=2)

                for img_index, pred, pred_max_prob in zip(img_list, preds_str, preds_max_prob):
                    confidence_score = pred_max_prob.cumprod(dim=0)[-1]
                    score = confidence_score * 100

                    print(f'class : {pred.upper()}, score : {score:0.2f}%')



    # CLASS ==========================================================================
    # 0 : A1  1 : A2  2 : A3  3 : B1  4 : B2  5: B3
    # ================================================================================

    # OUTPUT =========================================================================
    # [(A1.id, A2.id, A3.id), (A1.x), (A2.x), (A3.x), (B1.x), (B2.x), (B3.x)]
    # ================================================================================
    def delivery_vote(self, queue):
        if queue.count(-1) > int(QUEUE_SIZE / 2):
            return -1
        else:
            for element in queue: # get latest x_min
                if element != -1:
                    val = element
            return val


    def hard_vote(self, queue):
        return statistics.mode(queue)


    def yolo_pub(self):
        final_check = Int32MultiArray()

        for queue in self.queue_list:
            while len(queue) != QUEUE_SIZE: # delete first element
                del queue[0]

        queue_list = self.queue_list

        # queue voting
        for idx in range(len(queue_list)):
            if idx == 0: # find A number
                final_check.data.append(self.hard_vote(queue_list[idx]))
            else:
                final_check.data.append(self.delivery_vote(queue_list[idx]))

        self.delivery_pub.publish(final_check)


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
