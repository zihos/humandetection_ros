#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from message_filters import Subscriber, ApproximateTimeSynchronizer

from typing import List, Dict, Tuple
from std_msgs.msg import Header

from ultralytics import YOLO
from ultralytics.engine.results import Results
from ultralytics.engine.results import Boxes
from ultralytics.engine.results import Masks
from ultralytics.engine.results import Keypoints
from torch import cuda

from sensor_msgs.msg import Image
from my_pkg_bev.msg import Point2D
from my_pkg_bev.msg import BoundingBox2D
from my_pkg_bev.msg import Mask
from my_pkg_bev.msg import KeyPoint2D
from my_pkg_bev.msg import KeyPoint2DArray
from my_pkg_bev.msg import Detection
from my_pkg_bev.msg import DetectionArray
from std_srvs.srv import SetBool, SetBoolResponse

class RGBDProcessor:
    def __init__(self):
        rospy.init_node('rgbd_distance_calculator', anonymous=True)
        self.bridge = CvBridge()

        # Subscribers for color and depth images
        self.color_sub = Subscriber("/camera/color/image_raw", Image)
        self.depth_sub = Subscriber("/camera/depth/image_rect_raw", Image)
        self.publisher = rospy.Publisher('detections', DetectionArray, queue_size=10)

        # ApproximateTimeSynchronizer to synchronize the topics
        self.ts = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], queue_size=10, slop=0.5)
        self.ts.registerCallback(self.callback)

        self.color_image = None
        self.depth_image = None

        self.red = (0, 0, 255)  # 빨간색
        self.green = (0, 255, 0) # green
        self.thickness = 1

        rospy.loginfo("RGBDProcessor Initialized")

        # YOLO 모델 초기화 및 설정
        self.model = rospy.get_param('~model', 'yolov8n.pt')
        self.yolo = YOLO(self.model)
        self.device = rospy.get_param('~device', 'cpu')
        self.threshold = rospy.get_param('~threshold', 0.5)
        self.enable = rospy.get_param('~enable', True)
        self.yolo.fuse()
        rospy.loginfo('Yolov8Node created and configured')

    def callback(self, color_msg, depth_msg):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
            self.show_images()
        except CvBridgeError as e:
            rospy.logerr(f"Callback error: {e}")

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

    def show_images(self):
        if self.color_image is not None and self.depth_image is not None:
            try:
                # Resize color_image to match depth_image size
                resized_color_image = cv2.resize(self.color_image, (self.depth_image.shape[1], self.depth_image.shape[0]))

                # Normalize the depth image for display purposes
                depth_display = cv2.normalize(self.depth_image, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
                depth_display = cv2.applyColorMap(depth_display, cv2.COLORMAP_JET)

                results = self.yolo.predict(
                source=resized_color_image,
                verbose=False,
                stream=False,
                classes=[0],
                conf=self.threshold,
                device=self.device
                )

                results: Results = results[0].cpu()

                distance_list = []

                # create detection msgs
                detections_msg = DetectionArray()

                for box in results.boxes:
                    aux_msg = Detection()

                    aux_msg.class_id = int(box.cls)
                    aux_msg.class_name = self.yolo.names[int(box.cls)]
                    aux_msg.score = float(box.conf)
                    
                    xyxy = box.xyxy[0]
                    xywh = box.xywh[0]

                    msg = BoundingBox2D()
                    msg.center.position.x = xywh[0]
                    msg.center.position.y = xywh[1]
                    msg.size.x = xywh[2]
                    msg.size.y = xywh[3]
                    aux_msg.bbox = msg

                    # Ensure the coordinates are within the image dimensions
                    xmin, ymin, xmax, ymax = int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])
                    
                    height, width = self.depth_image.shape
                    
                    xmin = max(0, xmin)
                    ymin = max(0, ymin)
                    xmax = min(width, xmax)
                    ymax = min(height, ymax)

                    start_point = (xmin, ymin)
                    end_point = (xmax, ymax)
                    cv2.rectangle(resized_color_image, start_point, end_point, self.red, self.thickness)

                    # Get minimum distance in the ROI
                    # Extract the region of interest from the depth image
                    roi_depth = self.depth_image[ymin:ymax, xmin:xmax]

                    # Find the minimum distance within the ROI
                    min_distance = np.min(roi_depth[roi_depth > 0]) / 1000.0
                    distance_list.append(round(min_distance, 2))
                    aux_msg.distance = min_distance

                    detections_msg.detections.append(aux_msg)

                    # min_distance = self.get_min_distance_in_roi(self.depth_image, start_point[0], start_point[1], end_point[0], end_point[1])
                    bbox_text = f"{min_distance}m"
                    cv2.putText(resized_color_image, bbox_text, (start_point[0], start_point[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.green, 1)

                self.publisher.publish(detections_msg)

                # Combine color and depth images side by side
                combined_image = np.vstack((resized_color_image, depth_display))
                rospy.loginfo(distance_list)

                # Display the combined images
                cv2.imshow("RGB and Depth Image", combined_image)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    rospy.signal_shutdown("User requested shutdown")
                

            except Exception as e:
                rospy.logerr(f"Error in show_images: {e}")

def main():
    processor = RGBDProcessor()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()