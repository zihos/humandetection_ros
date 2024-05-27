#!/usr/bin/env python3
from typing import List, Dict

import message_filters
import rospy
import rospkg
import tensorflow as tf
import numpy as np
from bbox3d_utils import *


import os
from cv_bridge import CvBridge

from ultralytics.engine.results import Results
from ultralytics.engine.results import Boxes
from ultralytics.engine.results import Masks
from ultralytics.engine.results import Keypoints
from torch import cuda

from tensorflow.keras.applications import *
from tensorflow.keras.models import load_model
from tensorflow.keras.optimizers import *
from tensorflow.keras.layers import *

from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from my_pkg.msg import Point2D
from my_pkg.msg import BoundingBox2D
from my_pkg.msg import Prediction
from my_pkg.msg import PredictionArray
from my_pkg.msg import Mask
from my_pkg.msg import KeyPoint2D
from my_pkg.msg import KeyPoint2DArray
from my_pkg.msg import Detection
from my_pkg.msg import DetectionArray
from std_srvs.srv import SetBool, SetBoolResponse

from geometry_msgs.msg import Point, Vector3


os.environ["CUDA_VISIBLE_DEVICES"] = "-1"
P2 = np.array([[718.856, 0.0, 607.1928, 45.38225], [0.0, 718.856, 185.2157, -0.1130887], [0.0, 0.0, 1.0, 0.003779761]])
dims_avg = {'Car': np.array([1.52131309, 1.64441358, 3.85728004]),
'Van': np.array([2.18560847, 1.91077601, 5.08042328]),
'Truck': np.array([3.07044968,  2.62877944, 11.17126338]),
'Pedestrian': np.array([1.75562272, 0.67027992, 0.87397566]),
'Person_sitting': np.array([1.28627907, 0.53976744, 0.96906977]),
'Cyclist': np.array([1.73456498, 0.58174006, 1.77485499]),
'Tram': np.array([3.56020305,  2.40172589, 18.60659898])}
yolo_classes = ['Pedestrian', 'Cyclist', 'Car', 'motorcycle', 'airplane', 'Van', 'train', 'Truck', 'boat']



@tf.keras.utils.register_keras_serializable()
def orientation_loss(y_true, y_pred):
    # Find number of anchors
    anchors = tf.reduce_sum(tf.square(y_true), axis=2)
    anchors = tf.greater(anchors, tf.constant(0.5))
    anchors = tf.reduce_sum(tf.cast(anchors, tf.float32), 1)
    
    # Define the loss
    loss = -(y_true[:,:,0]*y_pred[:,:,0] + y_true[:,:,1]*y_pred[:,:,1])
    loss = tf.reduce_sum(loss, axis=1)
    epsilon = 1e-5  ##small epsilon value to prevent division by zero.
    anchors = anchors + epsilon
    loss = loss / anchors
    loss = tf.reduce_mean(loss)
    loss = 2 - 2 * loss 

    return loss

class Detection3DNode:
    def __init__(self):
        rospy.init_node('detection_3d_node', anonymous=True)
        
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('my_pkg')
        self.model_path = os.path.join(package_path, 'model', 'mobilenetv2_weights.h5')
        self.device = rospy.get_param('~device', 'cpu')
        self.threshold = rospy.get_param('~threshold', 0.5)
        self.enable = rospy.get_param('~enable', True)
        self.input_image_topic = rospy.get_param('~input_image_topic', '/detections')

        self.publisher = rospy.Publisher('prediction_3d', PredictionArray, queue_size=10)
        self.service = rospy.Service('enable', SetBool, self.enable_cb)

        # self.subscriber = rospy.Subscriber(self.input_image_topic, DetectionArray, self.detection_array_cb)
        self.cv_bridge = CvBridge()
        
        try:
            self.bbox3d_model = load_model(self.model_path,
                                           custom_objects={"orientation_loss": orientation_loss})
            # print(self.bbox3d_model.summary())
            rospy.loginfo('Model loaded successfully')
        except Exception as e:
            rospy.logerr('Failed to load model: {}'.format(e))
            exit(1)  # 모델 로딩 실패 시 노드 종료

        rospy.loginfo('Detection3DNode created and configured')

        # 메세지 싱크 맞추기
        self.image_sub = message_filters.Subscriber('/camera/rgb/image_raw', Image)
        self.detection_sub = message_filters.Subscriber('/detections', DetectionArray)
        self.subscriber = message_filters.ApproximateTimeSynchronizer(
            (self.image_sub, self.detection_sub), 10, 0.5)
        self.subscriber.registerCallback(self.detections_cb)


    def detections_cb(self, img_msg: Image, detections_msg: DetectionArray) -> None:
        # rospy.loginfo('Image')
        cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg)
        rospy.loginfo(cv_image.shape)
        # rospy.loginfo('Detection')
        rospy.loginfo(f"length: {len(detections_msg.detections)}")

        DIMS = []
        bboxes_3d = []

        prediction_msg = PredictionArray()

        for detection in detections_msg.detections:
            aux_msg = Prediction()
            padding = 0  # Set the padding value
            # xmin, ymin, xman, ymax 계산!
            xmin = max(0, detection.bbox.center.position.x - detection.bbox.size.x/2 - padding)
            ymin = max(0, detection.bbox.center.position.y - detection.bbox.size.y/2 - padding)
            # 우선 임의로 frame.shape 찍어서 하드코딩으로 넣어둠 (480, 854, 3)
            xmax = min(854, detection.bbox.center.position.x + detection.bbox.size.x/2 + padding)
            ymax = min(480, detection.bbox.center.position.y + detection.bbox.size.y/2 + padding)

            aux_msg.class_id = int(detection.class_id)

            # rospy.loginfo(f"xmin: {xmin}, ymin: {ymin}, xmax: {xmax}, ymax: {ymax}")
            crop = cv_image[int(ymin) : int(ymax), int(xmin) : int(xmax)]
            rospy.loginfo(f"Cropped Image Shape: {crop.shape}")

            patch = tf.convert_to_tensor(crop, dtype=tf.float32)
            patch /= 255.0  # Normalize to [0,1]
            patch = tf.image.resize(patch, (224, 224))  # Resize to 224x224
            patch = tf.expand_dims(patch, axis=0)  # Equivalent to reshape((1, *crop.shape))
            prediction = self.bbox3d_model.predict(patch, verbose = 0)

            # rospy.loginfo(prediction)

            dim = prediction[0][0]
            bin_anchor = prediction[1][0]
            bin_confidence = prediction[2][0]
            bin_size = 6 # 원본 코드에서 6이었음
            objID = 0 # 그런게 우리 2d bbox에는 없음...

            ###refinement dimension
            # 여기서는 person이 아니라 'Pedestrian'으로 찍힘
            try:
                # rospy.loginfo(dims_avg[str(yolo_classes[int(detection.class_id)])])
                dim += dims_avg[str(yolo_classes[int(detection.class_id)])] + dim
                DIMS.append(dim)
            except:
                dim = DIMS[-1]

            bbox_ = [int(xmin), int(ymin), int(xmax), int(ymax)]
            theta_ray = calc_theta_ray(cv_image, bbox_, P2)
            # update with predicted alpha, [-pi, pi]
            alpha = recover_angle(bin_anchor, bin_confidence, bin_size)
            alpha = alpha - theta_ray

            # calculate the location   # plot 3d bbox
            location, x = calc_location_(dimension=dim, proj_matrix=P2, box_2d=bbox_, alpha=alpha, theta_ray=theta_ray)
            bboxes_3d.append([bbox_, dim, alpha, theta_ray, bin_anchor, bin_confidence, detection.class_id, location, objID])

            aux_msg.class_name = yolo_classes[int(detection.class_id)]
            aux_msg.bbox = bbox_
            aux_msg.position = Point(location[0], location[1], location[2])
            aux_msg.dimensions = Vector3(dim[0], dim[1], dim[2])
            aux_msg.alpha = alpha
            aux_msg.theta_ray = theta_ray
            aux_msg.bin_anchor = [float(x) for x in bin_anchor.flatten().tolist()]
            aux_msg.bin_confidence = [float(x) for x in bin_confidence.flatten().tolist()]

            prediction_msg.predictions.append(aux_msg)


        # publish detections
        # detections_msg.header = msg.header
        self.publisher.publish(prediction_msg)
        # rospy.loginfo(f"EX of the 3D bboxes: {bboxes_3d}")
        # for i in bboxes_3d[-1]:
        #     print(type(i), i)


        


    def enable_cb(self, request):
        self.enable = request.data
        return SetBoolResponse(success=True)
    
    def detection_array_cb(self, msg: DetectionArray) -> None:
        # testing
        detections_msg = DetectionArray()

        if self.enable and msg.detections:
            for detection in msg.detections:
                padding = 0  # Set the padding value

                # xmin, ymin, xman, ymax 계산!
                xmin = max(0, detection.bbox.center.position.x - detection.bbox.size.x/2 - padding)
                ymin = max(0, detection.bbox.center.position.y - detection.bbox.size.y/2 - padding)
                # 우선 임의로 frame.shape 찍어서 하드코딩으로 넣어둠 (480, 854, 3)
                xmax = min(854, detection.bbox.center.position.x + detection.bbox.size.x/2 + padding)
                ymax = min(480, detection.bbox.center.position.y + detection.bbox.size.y/2 + padding)

                aux_msg = Detection()
                aux_msg.class_id = detection.class_id
                aux_msg.score = detection.score
                # self.publisher.publish(detection.class_id)
                # self.publisher.publish(detection.score)
                detections_msg.detections.append(aux_msg)
        
        # publish detections
            detections_msg.header = msg.header
            self.publisher.publish(detections_msg)
        
def main():
    try:
        node = Detection3DNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
    
    