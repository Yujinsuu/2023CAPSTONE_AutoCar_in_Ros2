import rospy
import cv2
import time
import torch
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Pose, PoseArray

from numpy import random
from models.experimental import attempt_load
from utils.datasets import letterbox
from utils.general import check_img_size, check_requirements, non_max_suppression, scale_coords
from utils.plots import plot_one_box
from utils.torch_utils import select_device, time_synchronized

WEIGHTS = 'weights/rubber_new.pt'
IMG_SIZE = 640
DEVICE = ''
AUGMENT = False
CONF_THRES = 0.65
IOU_THRES = 0.45
CLASSES = None
AGNOSTIC_NMS = False


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
if device.type != 'cpu' :
    model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))  # run once


# 이미지 메시지를 받는 콜백 함수
def image_callback(image_msg):
        # main
    check_requirements(exclude=('pycocotools', 'thop'))
    with torch.no_grad():
        bridge = CvBridge()
        cap = bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        # 여기에서 cap을 사용하여 원하는 작업을 수행할 수 있습니다.
        result = detect(cap)
        image_message = bridge.cv2_to_imgmsg(result, encoding="bgr8")
        image_message.header.stamp = rospy.Time.now()
        cam_pub.publish(image_message)

        #cv2.imshow("Image", result)
        #cv2.waitKey(1)

# Detect function
def detect(frame):
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


        poses = PoseArray()
        poses.header.stamp = rospy.Time.now()
        poses.header.frame_id = 'map'
        # Write results
        for *xyxy, conf, cls in reversed(det):
            label = f'{names[int(cls)]} {conf:.2f}'
            plot_one_box(xyxy, img0, label=label, color=colors[int(cls)], line_thickness=3)
            xmin, ymin, xmax, ymax = [int(tensor.item()) for tensor in xyxy]

            pose = Pose()
            # Set the position of the pose
            pose.position.x = int(cls) # 0 or 1
            pose.position.y = 0
            pose.position.z = 0

            pose.orientation.x = xmin
            pose.orientation.y = ymin
            pose.orientation.z = xmax
            pose.orientation.w = ymax

            ##box_ = [int(cls), xmin, ymin, xmax, ymax]
            #for b_ in range(len(box_)):
            #    box.append(box_[b_])

            poses.poses.append(pose)
        #boundingBoxes.data = box
        #yolo_pub.publish(boundingBoxes)
        pose_array_pub.publish(poses)

    # return results
    return img0

#def bounding_boxes_callback(msg):
#    poses = PoseArray()
#    poses.header.stamp = rospy.Time.now()
#    poses.header.frame_id = 'map'
#
#    for obj in msg.objects:
#        corners = obj.bounding_box_2d.corners
#        pose = Pose()
#
#        # Set the position of the pose
#        pose.position.x = obj.label_id # 0 or 1
#        pose.position.y = 0
#        pose.position.z = 0
#
#        # Set the orientation of the pose
#        pose.orientation.x = corners[0].kp[0]*1000 + corners[0].kp[1] #789123
#        pose.orientation.y = corners[1].kp[0]*1000 + corners[1].kp[1]
#        pose.orientation.z = corners[2].kp[0]*1000 + corners[2].kp[1]
#        pose.orientation.w = corners[3].kp[0]*1000 + corners[3].kp[1]
#
#        # Add the pose to the list of poses
#        poses.poses.append(pose)
#
#    # Publish the list of poses
#    pose_array_pub.publish(poses)
#    #print(msg.poses.size(), poses.poses.size())

if __name__ == '__main__':
    rospy.init_node('yolo')
    # 이미지 메시지를 받을 때마다 image_callback 함수 호출
    cam_pub = rospy.Publisher("/yolo_visualization",Image,queue_size =1)
    pose_array_pub= rospy.Publisher('/bounding_boxes', PoseArray, queue_size=10)
    image_subscriber = rospy.Subscriber("/combined_image", Image, image_callback)
    rospy.spin()
