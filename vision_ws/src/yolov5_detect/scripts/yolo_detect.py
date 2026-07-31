#! /usr/bin/python3

import rospy
import cv2
import numpy
import std_msgs.msg
import sensor_msgs.msg
import cv_bridge
import ultralytics
import ultralytics.engine.results
import typing
from yolov5_detect.srv import image2center, image2centerRequest


class Yolo:
    def __init__(self, path_of_model1: str, path_of_model2: str, first_detect_image: str):
        self.model: ultralytics.YOLO = ultralytics.YOLO(path_of_model1, task='detect')
        self.model_tank: ultralytics.YOLO = ultralytics.YOLO(path_of_model2, task='detect')
        self.bridge = cv_bridge.CvBridge()
        self.seq_tank: int = 0
        self.flag_sub: rospy.Subscriber = rospy.Subscriber("/detect/class_control", std_msgs.msg.Bool, self.flag_callback, queue_size=1)
        self.tank_flag: bool = False
        self.tank_flag_sub = rospy.Subscriber("/detect/tank_control", std_msgs.msg.Bool, self.tank_flag_callback)
        self.img_sub = rospy.Subscriber("/camera/color/image_raw", sensor_msgs.msg.Image, self.image_callback, queue_size=1)
        self.img_pub = rospy.Publisher("/yolo_detect", std_msgs.msg.String, queue_size=1)
        self.seq: int = 0
        self.flag: bool = False
        self.class_dict: dict = {"bridge": 0, "panzer": 0, "pillbox": 0, "tent": 0}
        self.tank_detect_num: int = 0
        self.service_client = rospy.ServiceProxy("/visual/service", image2center)
        self.service_client.wait_for_service()
        rospy.loginfo("load model successfully")
        temp_image1 = cv2.imread(first_detect_image + 'bridge.jpg')
        temp_image2 = cv2.imread(first_detect_image + 'panzer.jpg')
        temp_image3 = cv2.imread(first_detect_image + 'pillbox.jpg')
        temp_image4 = cv2.imread(first_detect_image + 'tent.jpg')
        temp_image5 = cv2.imread(first_detect_image + 'tank.jpg')
        _ = self.model(temp_image1)
        _ = self.model(temp_image2)
        _ = self.model(temp_image3)
        _ = self.model(temp_image4)
        _ = self.model_tank(temp_image5)

    def tank_flag_callback(self, msg: std_msgs.msg.Bool):
        self.tank_flag = True
    
    def flag_callback(self, msg: std_msgs.msg.Bool):
        self.flag = True
    
    def image_callback(self, img: sensor_msgs.msg.Image):
        
        try:
            image: numpy.ndarray = self.bridge.imgmsg_to_cv2(img, 'bgr8')
        except cv_bridge.CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return
        
        if not self.flag:
            self.class_dict = {"bridge": 0, "panzer": 0, "pillbox": 0, "tent": 0}
            self.img_pub.publish(std_msgs.msg.String())
            self.seq = 0

        else:
            results: typing.List[ultralytics.engine.results.Results] = self.model(image)
            if len(results) == 0:
                rospy.logwarn("No results found in the image")
                return
            elif len(results) > 1:
                rospy.logwarn("More than one result found, using the first one")
            self.seq += 1
            for result in results:
                # 绘制检测框和类别名
                if result.boxes is not None and result.boxes.xyxy is not None:
                    boxes = result.boxes.xyxy.cpu().numpy()
                    classes = result.boxes.cls.cpu().numpy()
                    for box, cls_id in zip(boxes, classes):
                        x1, y1, x2, y2 = map(int, box)
                        class_name = self.model.names[int(cls_id)]
                        self.class_dict[class_name] += 1
                        # cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        # cv2.putText(image, class_name, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            if self.seq >= 3:
                if (self.class_dict['bridge'] + self.class_dict['panzer'] + self.class_dict['pillbox'] + self.class_dict['tent']) == 0:
                    self.img_pub.publish(std_msgs.msg.String("Nothing"))
                    return
                self.img_pub.publish(std_msgs.msg.String(max(self.class_dict, key=self.class_dict.get)))
            else:
                self.img_pub.publish(std_msgs.msg.String())

        if not self.tank_flag:
            self.seq_tank = 0
            self.tank_detect_num = 0

        else:
            results_tank: typing.List[ultralytics.engine.results.Results] = self.model_tank(image)
            for result in results_tank:
                # 绘制检测框和类别名
                boxes = result.boxes.xyxy.cpu().numpy()
                classes = result.boxes.cls.cpu().numpy()
                for box, cls_id in zip(boxes, classes):
                    x1, y1, x2, y2 = map(int, box)
                    class_name = self.model_tank.names[int(cls_id)]
                    
                    if class_name == 'tank':
                        request = image2centerRequest()
                        request.x.data = (x1 + x2) // 2
                        request.y.data = (y1 + y2) // 2
                        self.service_client.call(request)
                        

        # cv2.imshow("Video Frame", image)
        # cv2.waitKey(1)
        
    
        
    def loop(self):
        rospy.spin()


def main() -> None:
    rospy.init_node("yolo_detect_node", anonymous=False)
    path_of_model1: str = str(rospy.get_param("/path_of_model1"))
    path_of_model2: str = str(rospy.get_param("/path_of_model2"))
    first_detect_image = str(rospy.get_param("/path_of_image"))
    model: Yolo = Yolo(path_of_model1, path_of_model2, first_detect_image)
    model.loop()


    

if __name__ == "__main__":
    main()
