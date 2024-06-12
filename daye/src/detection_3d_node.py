#!/usr/bin/env python3
from typing import List, Dict

import rospy
from std_msgs.msg import Header
from cv_bridge import CvBridge
import rospkg
import tensorflow as tf
import os
import numpy as np


from tensorflow.keras.applications import *
from tensorflow.keras.models import load_model
from tensorflow.keras.optimizers import *
from tensorflow.keras.layers import *

# from ultralytics.engine.results import Results
# from ultralytics.engine.results import Boxes
# from ultralytics.engine.results import Masks
# from ultralytics.engine.results import Keypoints

from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from daye.msg import Point2D
from daye.msg import BoundingBox2D
from daye.msg import VisBoundingBox3D
from daye.msg import VisBoundingBox3DArray
# from daye.msg import BoundingBox3D
# from vision_msgs.msg import BoundingBox3D
# from geometry_msgs.msg import Pose, Vector3

from daye.msg import Mask
from daye.msg import KeyPoint2D
from daye.msg import KeyPoint2DArray
from daye.msg import Detection
from daye.msg import DetectionArray
from std_srvs.srv import SetBool, SetBoolResponse

import message_filters 
import threading



yolo_classes = ['Pedestrian', 'Cyclist', 'Car', 'motorcycle', 'airplane', 'Van', 'train', 'Truck', 'boat']
P2 = np.array([[718.856, 0.0, 607.1928, 45.38225], [0.0, 718.856, 185.2157, -0.1130887], [0.0, 0.0, 1.0, 0.003779761]])
dims_avg = {'Car': np.array([1.52131309, 1.64441358, 3.85728004]),
'Van': np.array([2.18560847, 1.91077601, 5.08042328]),
'Truck': np.array([3.07044968,  2.62877944, 11.17126338]),
'Pedestrian': np.array([1.75562272, 0.67027992, 0.87397566]),
'Person_sitting': np.array([1.28627907, 0.53976744, 0.96906977]),
'Cyclist': np.array([1.73456498, 0.58174006, 1.77485499]),
'Tram': np.array([3.56020305,  2.40172589, 18.60659898])}


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

def euler_to_quaternion(roll, pitch, yaw):
    import tf.transformations
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    return quaternion

def calc_theta_ray(img, box_2d, proj_matrix):
    """
    Calculate global angle of object, see paper
    """
    width = img.shape[1]
    # Angle of View: fovx (rad) => 3.14
    fovx = 2 * np.arctan(width / (2 * proj_matrix[0][0]))
    center = (box_2d[1] + box_2d[0]) / 2
    dx = center - (width/2)

    mult = 1
    if dx < 0:
        mult = -1
    dx = abs(dx)
    angle = np.arctan((2*dx*np.tan(fovx/2)) / width)
    angle = angle * mult

    return angle

def recover_angle(bin_anchor, bin_confidence, bin_num):
    # select anchor from bins
    max_anc = np.argmax(bin_confidence)
    anchors = bin_anchor[max_anc]
    # compute the angle offset
    if anchors[1] > 0:
        angle_offset = np.arccos(anchors[0])
    else:
        angle_offset = -np.arccos(anchors[0])

    # add the angle offset to the center ray of each bin to obtain the local orientation
    wedge = 2 * np.pi / bin_num
    angle = angle_offset + max_anc * wedge

    # angle - 2pi, if exceed 2pi
    angle_l = angle % (2 * np.pi)

    # change to ray back to [-pi, pi]
    angle = angle_l - np.pi / 2
    if angle > np.pi:
        angle -= 2 * np.pi
    angle = round(angle, 2)
    return angle

# using this math: https://en.wikipedia.org/wiki/Rotation_matrix
def rotation_matrix(yaw, pitch=0, roll=0):
    tx = roll
    ty = yaw
    tz = pitch

    Rx = np.array([[1,0,0], [0, np.cos(tx), -np.sin(tx)], [0, np.sin(tx), np.cos(tx)]])
    Ry = np.array([[np.cos(ty), 0, np.sin(ty)], [0, 1, 0], [-np.sin(ty), 0, np.cos(ty)]])
    Rz = np.array([[np.cos(tz), -np.sin(tz), 0], [np.sin(tz), np.cos(tz), 0], [0,0,1]])


    return Ry.reshape([3,3])
    # return np.dot(np.dot(Rz,Ry), Rx)


# this is based on the paper. Math!
# calib is a 3x4 matrix, box_2d is [(xmin, ymin), (xmax, ymax)]
# Math help: http://ywpkwon.github.io/pdf/bbox3d-study.pdf
def calc_location(dimension, proj_matrix, box_2d, alpha, theta_ray):
    #global orientation
    orient = alpha + theta_ray
    R = rotation_matrix(orient)

    # format 2d corners
    xmin = box_2d[0]
    ymin = box_2d[1]
    xmax = box_2d[2]
    ymax = box_2d[3]

    # left top right bottom
    box_corners = [xmin, ymin, xmax, ymax]

    # get the point constraints
    constraints = []

    left_constraints = []
    right_constraints = []
    top_constraints = []
    bottom_constraints = []

    # using a different coord system
    dx = dimension[2] / 2
    dy = dimension[0] / 2
    dz = dimension[1] / 2

    # below is very much based on trial and error

    # based on the relative angle, a different configuration occurs
    # negative is back of car, positive is front
    left_mult = 1
    right_mult = -1

    # about straight on but opposite way
    if alpha < np.deg2rad(92) and alpha > np.deg2rad(88):
        left_mult = 1
        right_mult = 1
    # about straight on and same way
    elif alpha < np.deg2rad(-88) and alpha > np.deg2rad(-92):
        left_mult = -1
        right_mult = -1
    # this works but doesnt make much sense
    elif alpha < np.deg2rad(90) and alpha > -np.deg2rad(90):
        left_mult = -1
        right_mult = 1

    # if the car is facing the oppositeway, switch left and right
    switch_mult = -1
    if alpha > 0:
        switch_mult = 1

    # left and right could either be the front of the car ot the back of the car
    # careful to use left and right based on image, no of actual car's left and right
    for i in (-1,1):
        left_constraints.append([left_mult * dx, i*dy, -switch_mult * dz])
    for i in (-1,1):
        right_constraints.append([right_mult * dx, i*dy, switch_mult * dz])

    # top and bottom are easy, just the top and bottom of car
    for i in (-1,1):
        for j in (-1,1):
            top_constraints.append([i*dx, -dy, j*dz])
    for i in (-1,1):
        for j in (-1,1):
            bottom_constraints.append([i*dx, dy, j*dz])

    # now, 64 combinations
    for left in left_constraints:
        for top in top_constraints:
            for right in right_constraints:
                for bottom in bottom_constraints:
                    constraints.append([left, top, right, bottom])

    # filter out the ones with repeats
    constraints = filter(lambda x: len(x) == len(set(tuple(i) for i in x)), constraints)

    # create pre M (the term with I and the R*X)
    pre_M = np.zeros([4,4])
    # 1's down diagonal
    for i in range(0,4):
        pre_M[i][i] = 1

    best_loc = None
    best_error = [1e09]
    best_X = None

    # loop through each possible constraint, hold on to the best guess
    # constraint will be 64 sets of 4 corners
    count = 0
    for constraint in constraints:
        # each corner
        Xa = constraint[0]
        Xb = constraint[1]
        Xc = constraint[2]
        Xd = constraint[3]

        X_array = [Xa, Xb, Xc, Xd]

        # M: all 1's down diagonal, and upper 3x1 is Rotation_matrix * [x, y, z]
        Ma = np.copy(pre_M)
        Mb = np.copy(pre_M)
        Mc = np.copy(pre_M)
        Md = np.copy(pre_M)

        M_array = [Ma, Mb, Mc, Md]

        # create A, b
        A = np.zeros([4,3])
        b = np.zeros([4,1])

        indicies = [0,1,0,1]
        for row, index in enumerate(indicies):
            X = X_array[row]
            M = M_array[row]

            # create M for corner Xx
            RX = np.dot(R, X)
            M[:3,3] = RX.reshape(3)

            M = np.dot(proj_matrix, M)

            A[row, :] = M[index,:3] - box_corners[row] * M[2,:3]
            b[row] = box_corners[row] * M[2,3] - M[index,3]

        # solve here with least squares, since over fit will get some error
        loc, error, rank, s = np.linalg.lstsq(A, b, rcond=None)

        # found a better estimation
        if error < best_error:
            count += 1 # for debugging
            best_loc = loc
            best_error = error
            best_X = X_array

    # return best_loc, [left_constraints, right_constraints] # for debugging
    best_loc = [best_loc[0][0], best_loc[1][0], best_loc[2][0]]
    return best_loc, best_X

# class Detection3DNode:
#     def __init__(self, real_time):
#         rospy.init_node('detection_3d_node', anonymous=True)
        
#         rospack = rospkg.RosPack()
#         package_path = rospack.get_path('daye')
#         self.model_path = os.path.join(package_path, 'model', 'mobilenetv2_weights.h5')
#         self.device = rospy.get_param('~device', 'cpu')
#         self.threshold = rospy.get_param('~threshold', 0.5)
#         self.enable = rospy.get_param('~enable', True)
#         self.input_image_topic = rospy.get_param('~input_image_topic', '/detections')

#         self.publisher = rospy.Publisher('detections_3d', VisBoundingBox3DArray, queue_size=10)
#         self.service = rospy.Service('enable', SetBool, self.enable_cb)
        
#         # sub
#         # synchronize two different subscriber (two different msg types)
#         self.detections_sub = message_filters.Subscriber("detections", DetectionArray)
        
#         if real_time == "True":
#             self.image_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
#         else:
#             self.image_sub = message_filters.Subscriber("/camera/rgb/image_raw", Image)
        
#         self._synchronizer = message_filters.ApproximateTimeSynchronizer(
#             [self.detections_sub, self.image_sub], 10, 0.5)

#         self._synchronizer.registerCallback(self.callback)

#         self.cv_bridge = CvBridge()
#         # self.lock = threading.Lock()
#         # self.processing_thread = threading.Thread(target=self.processing_loop)
#         # self.processing_thread.start()
        
#         try:
#             self.bbox3d_model = load_model(self.model_path,
#                                            custom_objects={"orientation_loss": orientation_loss})
#             rospy.loginfo('3d detection Model loaded successfully')
#         except Exception as e:
#             rospy.logerr('Failed to load model: {}'.format(e))
#             exit(1)  # 모델 로딩 실패 시 노드 종료

#         rospy.loginfo('Detection3DNode created and configured')
        
#     def enable_cb(self, request):
#         self.enable = request.data
#         response_message = "Enabled" if self.enable else "Disabled"
#         rospy.loginfo(response_message)
#         return SetBoolResponse(success=True, message=response_message)
        
    
#     def process_detections(self, img: Image, detections_msg: DetectionArray):
#         # self.bbox3d_model.summary()
#         # Convert ROS Image message to OpenCV image
#         try:
#             cv_image = self.cv_bridge.imgmsg_to_cv2(img, "bgr8")
#         except CvBridgeError as e:
#             print(e)
#             return []

#         DIMS = []
#         bboxes = [] 
#         results = []
  
#         for msg in detections_msg.detections:


#             xmin = msg.bbox.center.position.x - (msg.bbox.size.x /2)
#             ymin = msg.bbox.center.position.y - (msg.bbox.size.y /2)
#             xmax = msg.bbox.center.position.x + (msg.bbox.size.x /2)
#             ymax = msg.bbox.center.position.y + (msg.bbox.size.y /2)
#             objID = msg.class_id
            

            
#             crop = cv_image[int(ymin) : int(ymax), int(xmin) : int(xmax)]
#             patch = tf.convert_to_tensor(crop, dtype=tf.float32)
#             patch /= 255.0  # Normalize to [0,1]
#             patch = tf.image.resize(patch, (224, 224))  # Resize to 224x224
#             patch = tf.expand_dims(patch, axis=0)  # Equivalent to reshape((1, *crop.shape))
#             prediction = self.bbox3d_model.predict(patch, verbose = 0)

#             dim = prediction[0][0]
#             bin_anchor = prediction[1][0]
#             bin_confidence = prediction[2][0]
#             bin_size = 6

#             ###refinement dimension
#             try:
#                 # dim += dims_avg[str(yolo_classes[int(objID.cpu().numpy())])] + dim
#                 dim += dims_avg[str(yolo_classes[int(objID)])] + dim
#                 DIMS.append(dim)
#             except:
#                 dim = DIMS[-1]
#             # rospy.loginfo(dim)
#             bbox_ = [int(xmin), int(ymin), int(xmax), int(ymax)]
            
#             frame = cv_image.copy()
#             ###### 여기서 frame이 "전체" 이미지 맞겠지? 
#             theta_ray = calc_theta_ray(frame, bbox_, P2)
#             # update with predicted alpha, [-pi, pi]
#             alpha = recover_angle(bin_anchor, bin_confidence, bin_size)
#             alpha = alpha - theta_ray

#             # calculate the location   # plot 3d bbox
#             location, x = calc_location(dimension=dim, proj_matrix=P2, box_2d=bbox_, alpha=alpha, theta_ray=theta_ray)
#             bboxes.append([bbox_, dim, alpha, theta_ray, bin_anchor, bin_confidence, objID, location, objID])
            
            
#             # 3D position and orientation of the bounding box center: Pose center
#             # total size of the bounding box, in meters, surrounding the object's center Vector3 size
#             # frame reference string frame_id
#             bbox3d = VisBoundingBox3D()

#             bbox3d.header = Header()
#             bbox3d.header.frame_id = "base_link"
#             bbox3d.header.stamp = rospy.Time.now()
#             bbox3d.bbox = bbox_
#             bbox3d.dim = dim
#             bbox3d.alpha = alpha
#             bbox3d.theta_ray = theta_ray
#             bbox3d.orient = bin_anchor.flatten()
#             bbox3d.conf = bin_confidence
#             bbox3d.classes = objID
#             bbox3d.location = location
#             bbox3d.objID = objID

#             results.append(bbox3d)

#         return results


#     def callback(self, detections_msg: DetectionArray, img:Image):
        
#         self.current_detections_msg = detections_msg
#         self.current_img = img
        
#         rospy.loginfo('Callback received detections and image')

#         rate = rospy.Rate(10)  # Adjust the rate as necessary
#         while not rospy.is_shutdown():
#             if self.enable and hasattr(self, 'current_detections_msg') and hasattr(self, 'current_img'):
#                 results = self.process_detections(self.current_img, self.current_detections_msg)
#                 # rospy.loginfo(f"results: {results}")
#                 if results:
#                     bbox3d_array = VisBoundingBox3DArray()
#                     bbox3d_array.header = Header()
#                     bbox3d_array.header.stamp = rospy.Time.now()
#                     bbox3d_array.header.frame_id = "base_link"
#                     bbox3d_array.boxes = results
#                     self.publisher.publish(bbox3d_array)
#                     rospy.loginfo("Published 3D bounding boxes")
#             rate.sleep()

class Detection3DNode:
    def __init__(self, real_time):
        rospy.init_node('detection_3d_node', anonymous=True)
        
        self.current_detections_msg = None
        self.current_img = None
        self.enable = True
        
        # self.tf_listener = tf.TransformListener()        
        
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('daye')

        self.model_path = os.path.join(package_path, 'model', 'mobilenetv2_weights.h5')
        self.device = rospy.get_param('~device', 'cpu')
        self.threshold = rospy.get_param('~threshold', 0.5)
        self.enable = rospy.get_param('~enable', True)
        self.input_image_topic = rospy.get_param('~input_image_topic', '/detections')

        self.publisher = rospy.Publisher('detections_3d', VisBoundingBox3DArray, queue_size=10)
        self.service = rospy.Service('enable', SetBool, self.enable_cb)
        
        # sub
        # synchronize two different subscriber (two different msg types)
        # self.detections_sub = message_filters.Subscriber("detections", DetectionArray)
        
        # if real_time == "True":
        #     self.image_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
        # else:
        #     self.image_sub = message_filters.Subscriber("/camera/rgb/image_raw", Image)
        # self._synchronizer = message_filters.ApproximateTimeSynchronizer(
        #     [self.detections_sub, self.image_sub], 10, 0.5)

        # self._synchronizer.registerCallback(self.callback)

        rospy.Subscriber('/detections', DetectionArray, self.detections_callback)
        if real_time =="True":
            rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        else:
            rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)


        self.cv_bridge = CvBridge()
        # self.lock = threading.Lock()
        # self.processing_thread = threading.Thread(target=self.processing_loop)
        # self.processing_thread.start()
        
        try:
            self.bbox3d_model = load_model(self.model_path,
                                           custom_objects={"orientation_loss": orientation_loss})
            rospy.loginfo('3d detection Model loaded successfully')
        except Exception as e:
            rospy.logerr('Failed to load model: {}'.format(e))
            exit(1)  # 모델 로딩 실패 시 노드 종료

        rospy.loginfo('Detection3DNode created and configured')
    
    def detections_callback(self, detections_msg):
        self.current_detections_msg = detections_msg 
    
    def image_callback(self, img):
        self.current_img=img

    def enable_cb(self, request):
        self.enable = request.data
        response_message = "Enabled" if self.enable else "Disabled"
        rospy.loginfo(response_message)
        return SetBoolResponse(success=True, message=response_message)
    

    def process_detections(self, img: Image, detections_msg: DetectionArray):
        # self.bbox3d_model.summary()
        # Convert ROS Image message to OpenCV image
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)
            return []

        DIMS = []
        bboxes = [] 
        results = []
  
        for msg in detections_msg.detections:


            xmin = msg.bbox.center.position.x - (msg.bbox.size.x /2)
            ymin = msg.bbox.center.position.y - (msg.bbox.size.y /2)
            xmax = msg.bbox.center.position.x + (msg.bbox.size.x /2)
            ymax = msg.bbox.center.position.y + (msg.bbox.size.y /2)
            objID = msg.class_id
            

            
            crop = cv_image[int(ymin) : int(ymax), int(xmin) : int(xmax)]
            patch = tf.convert_to_tensor(crop, dtype=tf.float32)
            patch /= 255.0  # Normalize to [0,1]
            patch = tf.image.resize(patch, (224, 224))  # Resize to 224x224
            patch = tf.expand_dims(patch, axis=0)  # Equivalent to reshape((1, *crop.shape))
            prediction = self.bbox3d_model.predict(patch, verbose = 0)

            dim = prediction[0][0]
            bin_anchor = prediction[1][0]
            bin_confidence = prediction[2][0]
            bin_size = 6

            ###refinement dimension
            try:
                # dim += dims_avg[str(yolo_classes[int(objID.cpu().numpy())])] + dim
                dim += dims_avg[str(yolo_classes[int(objID)])] + dim
                DIMS.append(dim)
            except:
                dim = DIMS[-1]
            # rospy.loginfo(dim)
            bbox_ = [int(xmin), int(ymin), int(xmax), int(ymax)]
            
            frame = cv_image.copy()
            ###### 여기서 frame이 "전체" 이미지 맞겠지? 
            theta_ray = calc_theta_ray(frame, bbox_, P2)
            # update with predicted alpha, [-pi, pi]
            alpha = recover_angle(bin_anchor, bin_confidence, bin_size)
            alpha = alpha - theta_ray

            # calculate the location   # plot 3d bbox
            location, x = calc_location(dimension=dim, proj_matrix=P2, box_2d=bbox_, alpha=alpha, theta_ray=theta_ray)
            bboxes.append([bbox_, dim, alpha, theta_ray, bin_anchor, bin_confidence, objID, location, objID])
            
            
            # 3D position and orientation of the bounding box center: Pose center
            # total size of the bounding box, in meters, surrounding the object's center Vector3 size
            # frame reference string frame_id
            bbox3d = VisBoundingBox3D()

            bbox3d.header = Header()
            bbox3d.header.frame_id = "base_link"
            bbox3d.header.stamp = rospy.Time.now()
            bbox3d.bbox = bbox_
            bbox3d.dim = dim
            bbox3d.alpha = alpha
            bbox3d.theta_ray = theta_ray
            bbox3d.orient = bin_anchor.flatten()
            bbox3d.conf = bin_confidence
            bbox3d.classes = objID
            bbox3d.location = location
            bbox3d.objID = objID

            results.append(bbox3d)
            # rospy.loginfo(bbox3d)

        return results


    def run(self):
        rate = rospy.Rate(10)  # Adjust the rate as necessary
        while not rospy.is_shutdown():
            if self.enable and self.current_detections_msg is not None and self.current_img is not None:
                results = self.process_detections(self.current_img, self.current_detections_msg)
                if results:
                    bbox3d_array = VisBoundingBox3DArray()
                    bbox3d_array.header = Header()
                    bbox3d_array.header.stamp = rospy.Time.now()
                    bbox3d_array.header.frame_id = "base_link"
                    bbox3d_array.boxes = results
                    self.publisher.publish(bbox3d_array)
                    # rospy.loginfo("Published 3D bounding boxes")
            rate.sleep()


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
        node = Detection3DNode(opt.real_time)
        node.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
    
    