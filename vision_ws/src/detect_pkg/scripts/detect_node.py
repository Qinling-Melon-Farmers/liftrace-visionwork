#!/home/cxy/anaconda3/envs/yolov5/bin/python3.8
import rospy
import cv2
import torch
import torch.backends.cudnn as cudnn
import numpy as np
from cv_bridge import CvBridge
import os
from sensor_msgs.msg import Image
from detect_pkg.msg import detected, qrcode  # 导入新消息类型
from models.common import DetectMultiBackend
from utils.general import non_max_suppression, scale_boxes
from utils.plots import Annotator, colors
from utils.torch_utils import select_device
from utils.augmentations import letterbox


@torch.no_grad()
class Yolov5Detector:
    def __init__(self):
        self.conf_thres = 0.75
        self.iou_thres = 0.45
        self.agnostic_nms = True
        self.max_det = 1000
        self.classes = None
        self.line_thickness = 3
        self.view_image = True

        self.ifdetected = 0
        self.qr_classes = []  # 用于存储二维码的类别信息

        current_path = os.path.dirname(__file__)
        weights = current_path + "/cifar.pt"
        self.device = select_device("0")
        self.model = DetectMultiBackend(weights, device=self.device, dnn=True, data=current_path + "/data/cifardata.yaml")
        self.stride, self.names = self.model.stride, self.model.names
        self.img_size = [640, 640]
        self.half = False
        self.half &= (self.model.pt or self.model.jit or self.model.onnx or self.model.engine) and self.device.type != "cpu"
        if self.model.pt or self.model.jit:
            self.model.model.half() if self.half else self.model.model.float()

        cudnn.benchmark = True
        self.model.warmup()

        # 发布新检测消息
        self.pred_pub = rospy.Publisher("detect", detected, queue_size=10)
        self.bridge = CvBridge()

    def qrcode_callback(self, msg):
        self.qr_classes = [msg.class1, msg.class2]

    def preprocess(self, img):
        img0 = img.copy()
        img = np.array([letterbox(img, self.img_size, stride=self.stride, auto=self.model.pt)[0]])
        img = img[..., ::-1].transpose((0, 3, 1, 2))
        img = np.ascontiguousarray(img)
        return img, img0

    def convert_to_tensor(self, im):
        im = torch.from_numpy(im).to(self.device)
        im = im.half() if self.half else im.float()
        im /= 255
        if len(im.shape) == 3:
            im = im[None]
        return im

    def callback(self, image_msg):
        im = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        im, im0 = self.preprocess(im)
        im = self.convert_to_tensor(im)
        pred = self.model(im, augment=False, visualize=False)
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det)
        det = np.array(pred[0].cpu())

        self.ifdetected = 0  # 默认未检测到匹配目标
        result_msg = detected()
        result_msg.ifdetected = 0
        result_msg.classname = ""
        result_msg.probability = 0.0

        if len(det):
            det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im.shape[1:]).round()
            for *xyxy, conf, cls in reversed(det):
                c = int(cls)
                class_name = self.names[c]
                if class_name in self.qr_classes:
                    self.ifdetected = 1
                    result_msg.ifdetected = 1
                    result_msg.classname = class_name
                    result_msg.probability = float(conf)
                    break  # 一旦匹配，退出

        self.pred_pub.publish(result_msg)

        if self.view_image:
            annotator = Annotator(im0, line_width=self.line_thickness, example=str(self.names))
            for *xyxy, conf, cls in reversed(det):
                c = int(cls)
                label = f"{self.names[c]} {conf:.2f}"
                annotator.box_label(xyxy, label, color=colors(c, True))
            im0 = annotator.result()
            cv2.imshow("Detection", im0)
            cv2.waitKey(1)


if __name__ == "__main__":
    rospy.init_node("yolov5", anonymous=True)
    detector = Yolov5Detector()
    rospy.Subscriber("webcam_imgmsg", Image, detector.callback)
    # 订阅二维码信息
    rospy.Subscriber("qr_info", qrcode, detector.qrcode_callback)
    rospy.spin()
