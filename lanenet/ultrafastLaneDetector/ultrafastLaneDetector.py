import cv2
import torch
import scipy.special
import numpy as np
import torchvision
import torchvision.transforms as transforms
from PIL import Image
from enum import Enum
from scipy.spatial.distance import cdist
from sklearn.linear_model import RANSACRegressor

from ultrafastLaneDetector.model import parsingNet

lane_colors = [(0,0,255),(0,255,0),(255,0,0),(0,255,255)]# 파란색, 녹색, 빨간색, 청록색

tusimple_row_anchor = [ 64,  68,  72,  76,  80,  84,  88,  92,  96, 100, 104, 108, 112,
			116, 120, 124, 128, 132, 136, 140, 144, 148, 152, 156, 160, 164,
			168, 172, 176, 180, 184, 188, 192, 196, 200, 204, 208, 212, 216,
			220, 224, 228, 232, 236, 240, 244, 248, 252, 256, 260, 264, 268,
			272, 276, 280, 284]
culane_row_anchor = [121, 131, 141, 150, 160, 170, 180, 189, 199, 209, 219, 228, 238, 248, 258, 267, 277, 287]

filtered_angle = 0.0

def process_lane(lane_index, lanes_points, visualization_img):
	arctan = calculate_arctan(lanes_points[lane_index])
	dev = np.round(arctan - np.mean(arctan))
	var = np.mean(dev**2)
	#print("L{}_dev".format(lane_index), dev, "L{}_var".format(lane_index), var)

	if var > 300 :
		cross_x = 820
		pass
		#draw_polyfit_line(visualization_img, lanes_points[lane_index], (0, 0, 255))
	elif var > 200 :
		lanes_points[lane_index] = remove_outlier(lanes_points[lane_index], dev)
		slope, intercept = draw_polyfit_line(visualization_img, lanes_points[lane_index], (0, 0, 255))
		cross_x = find_point_over_dcsion_line(slope, intercept, 430)
	else :
		slope, intercept = draw_polyfit_line(visualization_img, lanes_points[lane_index], (0, 0, 255))
		cross_x = find_point_over_dcsion_line(slope, intercept, 430)
	return cross_x

def remove_outlier(lanes_points, data):
    data_mean = np.mean(np.abs(data))
    outlier_index = np.where(np.abs(data)< data_mean)
    print(outlier_index)
    cleaned_lanes_points = np.delete(lanes_points, outlier_index, axis=0)
    return cleaned_lanes_points

def draw_polyfit_line(visualization_img, points, color):
	points_x = points[:, 0]
	points_y = points[:, 1]

	# RANSAC 모델 생성
	ransac = RANSACRegressor()

	# X와 y를 2D 배열로 변환
	X = points_x.reshape(-1, 1)
	y = points_y.reshape(-1, 1)

	# RANSAC을 사용하여 선형 회귀 모델 학습
	ransac.fit(X, y)

	# # RANSAC 모델로부터 추정된 선형 회귀식의 기울기와 y 절편 가져오기
	slope = ransac.estimator_.coef_[0][0]
	intercept = ransac.estimator_.intercept_[0]

	cv2.line(visualization_img, (int((590-intercept)/slope),590) , (int((295-intercept)/slope),295)  , color, 2)
	return slope, intercept

#def draw_polyfit_line(visualization_img, points, color):
#	points_x = points[:, 0]
#	points_y = points[:, 1]
#	coefficients = np.polyfit(points_x, points_y, 1)
#	f1 = np.poly1d(coefficients)
#
#	start_x = points[0, 0]
#	end_x = points[-1, 0]
#	start_y = f1(start_x)
#	end_y = f1(end_x)
#	start_point = (int(start_x), int(start_y))
#	end_point = (int(end_x), int(end_y))
#	cv2.line(visualization_img, start_point, end_point, color, 2)

def find_point_over_dcsion_line(slope, intercept, dcsion_y):
	dcsion_x = (dcsion_y - intercept) / slope
	return dcsion_x

def calculate_arctan(points):
    x, y = np.transpose(points)
    dx = np.diff(x)
    dy = np.diff(y)
    arctan_values = np.round(np.degrees(np.arctan2(dy, dx)), 1)
    return arctan_values

class ModelType(Enum):
	TUSIMPLE = 0
	CULANE = 1

class ModelConfig():

	def __init__(self, model_type):

		if model_type == ModelType.TUSIMPLE :
			self.init_tusimple_config()
		else:
			self.init_culane_config()

		self.filtered_angle = 0.0

	def init_tusimple_config(self):
		self.img_w = 1280
		self.img_h = 720
		self.row_anchor = tusimple_row_anchor
		self.griding_num = 100
		self.cls_num_per_lane = 56

	def init_culane_config(self):
		self.img_w = 1640
		self.img_h = 590
		self.row_anchor = culane_row_anchor
		self.griding_num = 200
		self.cls_num_per_lane = 18

class UltrafastLaneDetector():

	def __init__(self, model_path, model_type=ModelType.TUSIMPLE, use_gpu=False):

		self.use_gpu = use_gpu

		# Load model configuration based on the model type
		self.cfg = ModelConfig(model_type)

		# Initialize model
		self.model = self.initialize_model(model_path, self.cfg, use_gpu)

		# Initialize image transformation
		self.img_transform = self.initialize_image_transform()


	@staticmethod
	def initialize_model(model_path, cfg, use_gpu):

		# Load the model architecture
		net = parsingNet(pretrained = False, backbone='18', cls_dim = (cfg.griding_num+1,cfg.cls_num_per_lane,4),
						use_aux=False) # we dont need auxiliary segmentation in testing


		# Load the weights from the downloaded model
		if use_gpu:
			net = net.cuda()
			state_dict = torch.load(model_path, map_location='cuda')['model'] # CUDA
		else:
			state_dict = torch.load(model_path, map_location='cpu')['model'] # CPU

		compatible_state_dict = {}
		for k, v in state_dict.items():
			if 'module.' in k:
				compatible_state_dict[k[7:]] = v
			else:
				compatible_state_dict[k] = v

		# Load the weights into the model
		net.load_state_dict(compatible_state_dict, strict=False)
		net.eval()

		return net

	@staticmethod
	def initialize_image_transform():
		# Create transfom operation to resize and normalize the input images
		img_transforms = transforms.Compose([
			transforms.Resize((288, 800)),
			transforms.ToTensor(),
			transforms.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225)),
		])

		return img_transforms

	def detect_lanes(self, image, draw_points=True):

		input_tensor = self.prepare_input(image)

		# Perform inference on the image
		output = self.inference(input_tensor)

		# Process output data
		self.lanes_points, self.lanes_detected = self.process_output(output, self.cfg)

		# Draw depth image
		visualization_img, filtered_angle  = self.draw_lanes(image, self.lanes_points, self.lanes_detected, self.cfg, draw_points)

		return visualization_img, filtered_angle

	def prepare_input(self, img):
		# Transform the image for inference
		img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
		img_pil = Image.fromarray(img)
		input_img = self.img_transform(img_pil)
		input_tensor = input_img[None, ...]

		if self.use_gpu:
			input_tensor = input_tensor.cuda()

		return input_tensor

	def inference(self, input_tensor):
		with torch.no_grad():
			output = self.model(input_tensor)

		return output



	@staticmethod
	def process_output(output, cfg):
		# Parse the output of the model
		processed_output = output[0].data.cpu().numpy()
		processed_output = processed_output[:, ::-1, :]
		prob = scipy.special.softmax(processed_output[:-1, :, :], axis=0)
		idx = np.arange(cfg.griding_num) + 1
		idx = idx.reshape(-1, 1, 1)
		loc = np.sum(prob * idx, axis=0)
		processed_output = np.argmax(processed_output, axis=0)
		loc[processed_output == cfg.griding_num] = 0
		processed_output = loc


		col_sample = np.linspace(0, 800 - 1, cfg.griding_num)
		col_sample_w = col_sample[1] - col_sample[0]

		lanes_points = []
		lanes_detected = []

		max_lanes = processed_output.shape[1]
		for lane_num in range(max_lanes):
			lane_points = []
			# Check if there are any points detected in the lane
			if np.sum(processed_output[:, lane_num] != 0) > 2:

				lanes_detected.append(True)

				# Process each of the points for each lane
				for point_num in range(processed_output.shape[0]):
					if processed_output[point_num, lane_num] > 0:
						lane_point = [int(processed_output[point_num, lane_num] * col_sample_w * cfg.img_w / 800) - 1, int(cfg.img_h * (cfg.row_anchor[cfg.cls_num_per_lane-1-point_num]/288)) - 1 ]
						lane_points.append(lane_point)
			else:
				lanes_detected.append(False)

			lanes_points.append(lane_points)
		return np.array(lanes_points, dtype=object), np.array(lanes_detected, dtype=object)

	@staticmethod
	def draw_lanes(input_img, lanes_points, lanes_detected, cfg, draw_points=True):
		global filtered_angle
		# Write the detected line points in the image
		#print(lanes_points)
		visualization_img = cv2.resize(input_img, (cfg.img_w, cfg.img_h), interpolation = cv2.INTER_AREA)
		lanes_points = list(map(np.array, lanes_points))
		#print(lanes_points)
		#cv2.line(visualization_img, (820,590), (820,400), (0, 255, 255), 3)
				# ct error 결정 기준선에 대한 파라미터
		center_x, dcsion_y, offset_x, dcsion_line_w = 1640 // 2, 430, 300, 250

		L1_x_left, L1_x_right = int(center_x - offset_x - dcsion_line_w/2), int(center_x - offset_x + dcsion_line_w/2)
		L2_x_left, L2_x_right = int(center_x + offset_x - dcsion_line_w/2), int(center_x + offset_x + dcsion_line_w/2)

		cv2.line(visualization_img, (center_x , dcsion_y + 50), (center_x , dcsion_y - 50), (0, 255, 255), 2)
		cv2.line(visualization_img, (L1_x_left, dcsion_y), (L1_x_right, dcsion_y), (255, 0, 0), 2)
		cv2.line(visualization_img, (L2_x_left, dcsion_y), (L2_x_right, dcsion_y), (255, 0, 0), 2)

		# if not lanes_detected[1] and not lanes_detected[2] :
		# 	filtered_angle = # 이전값들
		#filtered_angle = 0

		if not lanes_detected[1] and not lanes_detected[2] : # 아무것도 안잡히면 지정한 steer angle을 저장해줌
			filtered_angle =  0

		if lanes_detected[1]: # 왼쪽만 잡혔을 때
			L1_x = process_lane(1, lanes_points, visualization_img)
			#filtered_angle = 20000
			if not lanes_detected[2] :
				ct_error= int(( L1_x + 1106 ) / 2) - center_x
				filtered_angle = int(filtered_angle*0.9 + ct_error * 0.1)
				print(filtered_angle)

		if lanes_detected[2]: # 오른쪽만 잡혔을 때
			L2_x = process_lane(2, lanes_points, visualization_img)
			#filtered_angle = 20000
			if not lanes_detected[1] :
				ct_error = int(( 552 + L2_x ) / 2) - center_x
				filtered_angle = int(filtered_angle*0.9 + ct_error * 0.1)
				print(filtered_angle)

		check = False
		if lanes_detected[1]  and lanes_detected[2] :
			ct_error = int((L1_x + L2_x ) / 2) - center_x
			filtered_angle = int(filtered_angle*0.9 + ct_error * 0.1)

			cv2.putText(visualization_img, f"L1_x : {L1_x}", (220, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
			cv2.putText(visualization_img, f"L2_x : {L2_x}", (620, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

			cv2.putText(visualization_img, f"ct_error: {ct_error}", (220, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
			cv2.putText(visualization_img, f"LPF_error: {filtered_angle}", (620, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
			cv2.line(visualization_img, (center_x, dcsion_y ), (center_x + filtered_angle, dcsion_y), (0, 255, 0), 5)
			cv2.line(visualization_img, (center_x, dcsion_y+30 ), (center_x  + ct_error , dcsion_y+30), (122, 122, 255), 5)

			check = True


		if(draw_points):
			for lane_num,lane_points in enumerate(lanes_points) :
				for lane_point in lane_points :
					cv2.circle(visualization_img, (lane_point[0],lane_point[1]), 4, lane_colors[lane_num], -1)

		return check, visualization_img, filtered_angle
