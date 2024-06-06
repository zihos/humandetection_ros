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
from my_pkg.msg import Point2D
from my_pkg.msg import BoundingBox2D
from my_pkg.msg import Mask
from my_pkg.msg import KeyPoint2D
from my_pkg.msg import KeyPoint2DArray
from my_pkg.msg import Detection
from my_pkg.msg import DetectionArray
from std_srvs.srv import SetBool, SetBoolResponse

class RGBDProcessor:
    def __init__(self):
        rospy.init_node('rgbd_distance_calculator', anonymous=True)
        self.bridge = CvBridge()

        # Subscribers for color and depth images
        self.color_sub = Subscriber("/camera/color/image_raw", Image)
        self.depth_sub = Subscriber("/camera/depth/image_rect_raw", Image)

        # ApproximateTimeSynchronizer to synchronize the topics
        self.ts = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], queue_size=10, slop=0.5)
        self.ts.registerCallback(self.callback)

        self.color_image = None
        self.depth_image = None

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
    
    def get_min_distance_in_roi(self, depth_image: np.array, xmin: int, ymin: int, xmax: int, ymax: int) -> float:
        roi = depth_image[ymin:ymax, xmin:xmax]
        min_distance = np.min(roi)
        return min_distance
    
    def get_distance_at_point(self, x, y):
        if self.depth_image is not None:
            distance = self.depth_image[y, x] / 1000.0
            # rospy.loginfo(f"Distance at ({x}, {y}): {distance:.2f} meters")
            return distance
        else:
            rospy.logwarn("Depth image not available")
            return None


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

                if results.boxes:
                    hypothesis = self.parse_hypothesis(results)
                    boxes = self.parse_boxes(results)

                if results.masks:
                    masks = self.parse_masks(results)

                if results.keypoints:
                    keypoints = self.parse_keypoints(results)

                for box in results.boxes:
                    xyxy = box.xyxy[0]
                    xywh = box.xywh[0]

                    start_point = (int(xyxy[0]), int(xyxy[1]))
                    end_point = (int(xyxy[2]), int(xyxy[3]))
                    color = (0, 0, 255)  # 빨간색
                    thickness = 2
                    cv2.rectangle(resized_color_image, start_point, end_point, color, thickness)

                    center_distance = self.get_distance_at_point(int(xywh[0]), int(xywh[1]))

                    # 빨간색 점 그리기 (중앙에)
                    color = (0, 0, 255)  # 빨간색 (BGR 형식)
                    thickness = -1  # 채워진 원
                    radius = 5  # 점의 반지름

                    cv2.circle(resized_color_image, (int(xywh[0]), int(xywh[1])), radius, color, thickness)

                    xmin, ymin, xmax, ymax = int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])
                    # Ensure the coordinates are within the image dimensions
                    height, width = self.depth_image.shape
                    
                    xmin = max(0, xmin)
                    ymin = max(0, ymin)
                    xmax = min(width, xmax)
                    ymax = min(height, ymax)

                    # Get minimum distance in the ROI
                    # Extract the region of interest from the depth image
                    roi_depth = self.depth_image[ymin:ymax, xmin:xmax]

                    # Find the minimum distance within the ROI
                    min_distance = np.min(roi_depth[roi_depth > 0]) / 1000.0

                    # Compute the average distance, ignoring NaNs and invalid values
                    valid_depths = roi_depth[np.isfinite(roi_depth)]
                    if valid_depths.size > 0:
                        average_distance = np.mean(valid_depths) / 1000.0
                    # min_distance = self.get_min_distance_in_roi(self.depth_image, start_point[0], start_point[1], end_point[0], end_point[1])
                    bbox_text = f"avg: {average_distance:.2f}m, center: {center_distance:.2f}m, min: {min_distance:.2f}m"
                    rospy.loginfo(bbox_text)
                    cv2.putText(resized_color_image, bbox_text, (start_point[0], start_point[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                    
                # Combine color and depth images side by side
                combined_image = np.vstack((resized_color_image, depth_display))

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