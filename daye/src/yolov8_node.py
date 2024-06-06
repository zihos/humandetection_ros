#!/usr/bin/env python3
from typing import List, Dict
import numpy as np 
from typing import List, Tuple
import rospy
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import Header

from ultralytics import YOLO
from ultralytics.engine.results import Results
from ultralytics.engine.results import Boxes
from ultralytics.engine.results import Masks
from ultralytics.engine.results import Keypoints
from torch import cuda

from sensor_msgs.msg import Image
from my_pkg.msg import Point2D
from my_pkg.msg import BoundingBox2D
from my_pkg.msg import Mask
from my_pkg.msg import KeyPoint2D
from my_pkg.msg import KeyPoint2DArray
from my_pkg.msg import Detection
from my_pkg.msg import DetectionArray
from std_srvs.srv import SetBool, SetBoolResponse

class Yolov8Node:
    def __init__(self, real_time):
        rospy.init_node('yolov8_node', anonymous=True)
        self.model = rospy.get_param('~model', 'yolov8n.pt')
        self.device = rospy.get_param('~device', 'cpu')
        self.threshold = rospy.get_param('~threshold', 0.5)
        self.enable = rospy.get_param('~enable', True)
        
        self.publisher = rospy.Publisher('detections', DetectionArray, queue_size=10)

        self.service = rospy.Service('enable', SetBool, self.enable_cb)
        self.image_w_bb = rospy.Publisher('/visualization_img_2bb', Image, queue_size=10)
        
        # YOLO 모델 초기화 및 설정
        self.yolo = YOLO(self.model)
        self.yolo.fuse()
        rospy.loginfo('Yolov8Node created and configured')
        
        if real_time == "True":
            self.subscriber = rospy.Subscriber('/camera/color/image_raw', Image, self.image_cb)
        else:
            self.input_image_topic = rospy.get_param('~input_image_topic', '/camera/rgb/image_raw')
            self.subscriber = rospy.Subscriber(self.input_image_topic, Image, self.image_cb)
        self.cv_bridge = CvBridge()
        

    def enable_cb(self, request):
        self.enable = request.data
        return SetBoolResponse(success=True)
    
    def parse_hypothesis(self, results: Results) -> List[Dict]:

        hypothesis_list = []

        box_data: Boxes
        for box_data in results.boxes:
            hypothesis = {
                "class_id": int(box_data.cls),
                "class_name": self.yolo.names[int(box_data.cls)],
                "score": float(box_data.conf)
            }
            hypothesis_list.append(hypothesis)

        return hypothesis_list

    def parse_boxes(self, results: Results) -> List[BoundingBox2D]:

        boxes_list = []

        box_data: Boxes
        for box_data in results.boxes:

            msg = BoundingBox2D()

            # get boxes values
            box = box_data.xywh[0]
            msg.center.position.x = float(box[0])
            msg.center.position.y = float(box[1])
            msg.size.x = float(box[2])
            msg.size.y = float(box[3])

            # append msg
            boxes_list.append(msg)

        return boxes_list

    def parse_masks(self, results: Results) -> List[Mask]:

        masks_list = []

        def create_point2d(x: float, y: float) -> Point2D:
            p = Point2D()
            p.x = x
            p.y = y
            return p

        mask: Masks
        for mask in results.masks:

            msg = Mask()

            msg.data = [create_point2d(float(ele[0]), float(ele[1]))
                        for ele in mask.xy[0].tolist()]
            msg.height = results.orig_img.shape[0]
            msg.width = results.orig_img.shape[1]

            masks_list.append(msg)

        return masks_list

    def parse_keypoints(self, results: Results) -> List[KeyPoint2DArray]:

        keypoints_list = []

        points: Keypoints
        for points in results.keypoints:

            msg_array = KeyPoint2DArray()

            if points.conf is None:
                continue

            for kp_id, (p, conf) in enumerate(zip(points.xy[0], points.conf[0])):

                if conf >= self.threshold:
                    msg = KeyPoint2D()

                    msg.id = kp_id + 1
                    msg.point.x = float(p[0])
                    msg.point.y = float(p[1])
                    msg.score = float(conf)

                    msg_array.data.append(msg)

            keypoints_list.append(msg_array)

        return keypoints_list
    
    def draw_box(self, cv_image:np.array, detection: List, color: Tuple[int]) -> np.array:
        min_pt = (round(detection[0]), round(detection[1]))
        max_pt = (round(detection[2]), round(detection[3]))

        #draw box 
        cv2.rectangle(cv_image, min_pt, max_pt, color, 2)
        return cv_image
    
    def image_cb(self, msg: Image) -> None:
        # if self.enable:
        if self.enable and hasattr(self, 'yolo'):
            # rospy.loginfo('Image callback triggered')
            # convert image + predict
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            
            img2 = cv_image.copy() # for 2D-bb image 
            
            results = self.yolo.predict(
                source=cv_image,
                verbose=False,
                stream=False,
                classes=[0],
                conf=self.threshold,
                device=self.device
            )
            
            results: Results = results[0].cpu()

            if results.boxes:
                hypothesis = self.parse_hypothesis(results)
                boxes = self.parse_boxes(results)

            if results.masks:
                masks = self.parse_masks(results)

            if results.keypoints:
                keypoints = self.parse_keypoints(results)

            # create detection msgs
            detections_msg = DetectionArray()

            for i in range(len(results)):

                aux_msg = Detection()

                if results.boxes:
                    aux_msg.class_id = hypothesis[i]["class_id"]
                    aux_msg.class_name = hypothesis[i]["class_name"]
                    aux_msg.score = hypothesis[i]["score"]

                    aux_msg.bbox = boxes[i]

                if results.masks:
                    aux_msg.mask = masks[i]

                if results.keypoints:
                    aux_msg.keypoints = keypoints[i]

                detections_msg.detections.append(aux_msg)

                xmin = boxes[i].center.position.x - (boxes[i].size.x /2)
                ymin = boxes[i].center.position.y - (boxes[i].size.y /2)
                xmax = boxes[i].center.position.x + (boxes[i].size.x /2)
                ymax = boxes[i].center.position.y + (boxes[i].size.y /2)

                img2 = self.draw_box(img2, [int(xmin), int(ymin), int(xmax), int(ymax)], (0, 255,0))
            # publish detections
            detections_msg.header = msg.header
            self.publisher.publish(detections_msg)
            
            bb_cv_image = self.cv_bridge.cv2_to_imgmsg(img2, "bgr8")
            # Set the header frame ID, must be set to the same of visualizer Marker message for rviz 
            bb_cv_image.header = Header()
            bb_cv_image.header.frame_id = "camera_link"
            bb_cv_image.header.stamp = rospy.Time.now()

            self.image_w_bb.publish(bb_cv_image)

            del results
            del cv_image


def get_args():
    import sys
    import argparse
    parser = argparse.ArgumentParser(
        description=""
    )

    # Required arguments
    parser.add_argument("--real_time",
                        type=str,
                        default="True",
                        help="Whether it is a real-time experiment with a RGB-D camera")

    return parser.parse_args(rospy.myargv()[1:])    
       

def main():
    try:
        opt = get_args()
        node = Yolov8Node(opt.real_time)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()