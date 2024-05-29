#!/usr/bin/env python3 
import rospy 
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from my_pkg_bev.msg import VisBoundingBox3D
from my_pkg_bev.msg import VisBoundingBox3DArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Header

import numpy as np 
import message_filters 

import os
import cv2
import numpy as np
# from enum import Enum
# import itertools
from typing import List, Tuple
import random

from matplotlib.path import Path
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.gridspec import GridSpec
import matplotlib.patches as mpatches
from collections import deque

# P2 = proj_matrix 
P2 = np.array([[718.856, 0.0, 607.1928, 45.38225], [0.0, 718.856, 185.2157, -0.1130887], [0.0, 0.0, 1.0, 0.003779761]])
yolo_classes = ['Pedestrian', 'Cyclist', 'Car', 'motorcycle', 'airplane', 'Van', 'train', 'Truck', 'boat']
bin_size=6
MARKERS_MAX = 100 


class Plot3DBoxBev:
    """Plot 3D bounding box and bird eye view"""
    def __init__(
        self,
        proj_matrix = None, # projection matrix P2
        object_list = ["car", "pedestrian", "truck", "cyclist", "motorcycle", "bus"],
        
    ) -> None:

        self.proj_matrix = proj_matrix
        self.object_list = object_list

        self.fig = plt.figure(figsize=(20, 5), dpi=90)
        plt.subplots_adjust(left=0.001, right=0.999, top=0.999, bottom=0.001)
        gs = GridSpec(1, 4)
        gs.update(wspace=0)
        self.ax = self.fig.add_subplot(gs[0, :3])
        self.ax2 = self.fig.add_subplot(gs[0, 3:])

        self.shape = 900
        self.scale = 15

        self.COLOR = {
            "car": "blue",
            "pedestrian": "green",
            "truck": "yellow",
            "cyclist": "red",
            "motorcycle": "cyan",
            "bus": "magenta",
        }
        plt.close(self.fig)  # Close the figure

        
    def compute_bev(self, dim, loc, rot_y):
        """compute bev"""
        # convert dimension, location and rotation
        h = dim[0] * self.scale
        w = dim[1] * self.scale
        l = dim[2] * self.scale
        x = loc[0] * self.scale
        y = loc[1] * self.scale
        z = loc[2] * self.scale 
        rot_y = np.float64(rot_y*-1)

        # Scale up or down the dimensions to make the object appear closer
        scale_factor = 1.2  # Adjust this value as needed
        h *= scale_factor
        w *= scale_factor
        l *= scale_factor


        # R = np.array([[-np.cos(rot_y), np.sin(rot_y)], [np.sin(rot_y), np.cos(rot_y)]])
        R = np.array([[+np.cos(rot_y), -np.sin(rot_y)], [+np.sin(rot_y), np.cos(rot_y)]])

        t = np.array([x, z]).reshape(1, 2).T
        # x_corners = [0, l, l, 0]  # -l/2
        # z_corners = [w, w, 0, 0]  # -w/2
        x_corners = np.array([0, l, l, 0])  # -l/2
        z_corners = np.array([w, w, 0, 0]) # -w/2
        x_corners += -w / 4
        z_corners += -l / 2

        # bounding box in object coordinate
        corners_2D = np.array([x_corners, z_corners])
        # rotate
        corners_2D = R.dot(corners_2D)
        # translation
        corners_2D = t - corners_2D
        # in camera coordinate
        corners_2D[0] += int(self.shape / 2)
        corners_2D = (corners_2D).astype(np.int16)
        corners_2D = corners_2D.T

        return np.vstack((corners_2D, corners_2D[0, :]))

    global tracking_trajectories
    tracking_trajectories = {}
    
    def draw_bev(self, dim, loc, rot_y, class_object, objId):
        color = self.COLOR[class_object]
        """draw bev"""

        # gt_corners_2d = self.compute_bev(self.gt_dim, self.gt_loc, self.gt_rot_y)
        pred_corners_2d = self.compute_bev(dim, loc, rot_y)

        codes = [Path.LINETO] * pred_corners_2d.shape[0]
        codes[0] = Path.MOVETO
        codes[-1] = Path.CLOSEPOLY
        pth = Path(pred_corners_2d, codes)
        patch = patches.PathPatch(pth, fill=True, color=color, label="prediction")
        self.ax2.add_patch(patch)

        if objId[0] is not None:
            # draw z location of object
            self.ax2.text(
                pred_corners_2d[0, 0],
                pred_corners_2d[0, 1],
                f"z: {loc[2]:.1f}"+' '+str(int(objId[0])),
                fontsize=8,
                color="white",
                bbox=dict(facecolor="green", alpha=0.4, pad=0.5))
            centroids, ids = self.calculate_centroid(pred_corners_2d, int(objId[0]))
            # Append centroid to tracking_points
            if objId[0] is not None and int(objId[0]) not in tracking_trajectories:
                tracking_trajectories[int(objId[0])] = deque(maxlen=4)
            else:
                tracking_trajectories[int(objId[0])].append(centroids)
            # print(tracking_trajectories)

        else:
            # draw z location of object
            self.ax2.text(
                pred_corners_2d[0, 0],
                pred_corners_2d[0, 1],
                f"z: {loc[2]:.1f}",
                fontsize=8,
                color="white",
                bbox=dict(facecolor="green", alpha=0.4, pad=0.5))



    def calculate_centroid(self, vertices, obj_id):
        x_sum = np.sum(vertices[:, 0])
        y_sum = np.sum(vertices[:, 1])
        centroid_x = x_sum / len(vertices)
        centroid_y = y_sum / len(vertices)
        return ((centroid_x, centroid_y), obj_id)



    def compute_3dbox(self, bbox, dim, loc, rot_y):
        """compute 3d box"""
        # 2d bounding box
        xmin, ymin = int(bbox[0]), int(bbox[1])
        xmax, ymax = int(bbox[2]), int(bbox[3])

        # convert dimension, location
        h, w, l = dim[0], dim[1], dim[2]
        x, y, z = loc[0], loc[1], loc[2]

        R = np.array([[np.cos(rot_y), 0, np.sin(rot_y)], [0, 1, 0], [-np.sin(rot_y), 0, np.cos(rot_y)]])
        x_corners = [0, l, l, l, l, 0, 0, 0]  # -l/2
        y_corners = [0, 0, h, h, 0, 0, h, h]  # -h
        z_corners = [0, 0, 0, w, w, w, w, 0]  # -w/2

        # x_corners += -l / 2
        # y_corners += -h
        # z_corners += -w / 2

        x_corners = [x - l / 2 for x in x_corners]
        y_corners = [y - h for y in y_corners]
        z_corners = [z - w / 2 for z in z_corners]

        corners_3D = np.array([x_corners, y_corners, z_corners])
        corners_3D = R.dot(corners_3D)
        corners_3D += np.array([x, y, z]).reshape(3, 1)

        corners_3D_1 = np.vstack((corners_3D, np.ones((corners_3D.shape[-1]))))
        corners_2D = self.proj_matrix.dot(corners_3D_1)
        corners_2D = corners_2D / corners_2D[2]
        corners_2D = corners_2D[:2]

        return corners_2D, corners_3D

    def draw_3dbox_for_BEV(self, class_object, bbox, dim, loc, rot_y):
        """draw 3d box"""
        # color = self.COLOR[class_object]
        color = "green"
        corners_2D = self.compute_3dbox(bbox, dim, loc, rot_y)

        # draw all lines through path
        # https://matplotlib.org/users/path_tutorial.html
        bb3d_lines_verts_idx = [0, 1, 2, 3, 4, 5, 6, 7, 0, 5, 4, 1, 2, 7, 6, 3]
        bb3d_on_2d_lines_verts = corners_2D[:, bb3d_lines_verts_idx]
        verts = bb3d_on_2d_lines_verts.T
        codes = [Path.LINETO] * verts.shape[0]
        codes[0] = Path.MOVETO
        pth = Path(verts, codes)
        patch = patches.PathPatch(pth, fill=False, color=color, linewidth=1)

        width = corners_2D[:, 3][0] - corners_2D[:, 1][0]
        height = corners_2D[:, 2][1] - corners_2D[:, 1][1]
        # put a mask on the front
        front_fill = patches.Rectangle((corners_2D[:, 1]), width, height, fill=True, color=color, alpha=0.2)
        self.ax.add_patch(patch)
        self.ax.add_patch(front_fill)

        # Calculate the center of the rectangle
        center_x = corners_2D[:, 1][0] + width / 2
        center_y = corners_2D[:, 1][1] + height / 2
        radius = min(width, height) / 20
        circle = plt.Circle((center_x, center_y), radius, fill=True, color='white', alpha=0.5)
        self.ax.add_patch(circle)

        # # draw text of location, dimension, and rotation
        # self.ax.text(
        #     corners_2D[:, 1][0],
        #     corners_2D[:, 1][1],
        #     f"Loc: ({loc[0]:.2f}, {loc[1]:.2f}, {loc[2]:.2f})\nDim: ({dim[0]:.2f}, {dim[1]:.2f}, {dim[2]:.2f})\nYaw: {rot_y:.2f}",
        #     fontsize=8,
        #     color="white",
        #     bbox=dict(facecolor=color, alpha=0.2, pad=0.5),
        # )

        # Convert the plot to a numpy array image
        # self.fig.canvas.draw()
        # img = np.frombuffer(self.fig.canvas.tostring_rgb(), dtype=np.uint8)
        # img = img.reshape(self.fig.canvas.get_width_height()[::-1] + (3,))

    def draw_3dbox(self, img, class_object, bbox, dim, loc, rot_y):
        """draw 3d box"""
        color = (0, 255, 0)  # Green color in BGR
        corners_2D, corners_3D = self.compute_3dbox(bbox, dim, loc, rot_y) # 2D projection of the 3D bounding box corners

        # Drawing lines 
        bb3d_lines_verts_idx = [0, 1, 2, 3, 4, 5, 6, 7, 0, 5, 4, 1, 2, 7, 6, 3]
        bb3d_on_2d_lines_verts = corners_2D[:, bb3d_lines_verts_idx]
        verts = bb3d_on_2d_lines_verts.T

        for i in range(len(verts) - 1):
            pt1 = (int(verts[i][0]), int(verts[i][1]))
            pt2 = (int(verts[i + 1][0]), int(verts[i + 1][1]))
            cv2.line(img, pt1, pt2, color, 2)

        # Back Masking 
        width = int(corners_2D[:, 3][0] - corners_2D[:, 1][0])
        height = int(corners_2D[:, 2][1] - corners_2D[:, 1][1])
        front_fill_pts = [(int(corners_2D[:, 1][0]), int(corners_2D[:, 1][1])),
                          (int(corners_2D[:, 1][0] + width), int(corners_2D[:, 1][1])),
                          (int(corners_2D[:, 1][0] + width), int(corners_2D[:, 1][1] + height)),
                          (int(corners_2D[:, 1][0]), int(corners_2D[:, 1][1] + height))]
        cv2.fillPoly(img, [np.array(front_fill_pts)], color)

        # the center of the front face and draws a small circle there
        # center_x = int(corners_2D[:, 1][0] + width / 2)
        # center_y = int(corners_2D[:, 1][1] + height / 2)
        # radius = int(min(width, height) / 20)
        # cv2.circle(img, (center_x, center_y), radius, (255, 0, 0), -1)
    
        return img, corners_3D
        

    def plot(
        self,
        img = None,
        class_object: str = None,
        bbox = None, # bbox 2d [xmin, ymin, xmax, ymax]
        dim = None, # dimension of the box (l, w, h)
        loc = None, # location of the box (x, y, z)
        rot_y = None, # rotation of the box around y-axis
        objId = None, # objids
    ):
        """plot 3d bbox and bev"""
        # initialize bev image
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        bev_img = np.zeros((self.shape, self.shape, 3), np.uint8)
        self.draw_range(img, bev_img) #camera range

        # self.draw_centroid_trajectories(bev_img)
        # loop through all detections
        if class_object in self.object_list:
            # self.draw_3dbox_for_BEV(class_object, bbox, dim, loc, rot_y)
            self.draw_bev(dim, loc, rot_y, class_object, objId)

        return img 



    def save_plot(self, path, name):
        self.fig.savefig(
            os.path.join(path, f"{name}.png"),
            dpi=self.fig.dpi,
            bbox_inches="tight",
            pad_inches=0.0,
        )


    def show_result(self):
        # Draw the Matplotlib figure
        self.fig.canvas.draw()

        # Convert the figure to an image array
        # image = np.fromstring(self.fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
        image = np.frombuffer(self.fig.canvas.tostring_rgb(), dtype=np.uint8)
        image = image.reshape(self.fig.canvas.get_width_height()[::-1] + (3,))
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        # self.ax2.clear()  # Clear the axes for the next plot
     
        return image


    def draw_range(self, img, bev_img):

        # visualize 3D bounding box
        # img_rgb = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        self.ax.imshow(img)
        self.ax.set_xticks([])
        self.ax.set_yticks([])

        # visualize bird eye view (bev)
        self.ax2.imshow(bev_img, origin="lower")
        self.ax2.set_xticks([])
        self.ax2.set_yticks([])

        # Define colors and labels for legend
        legend_colors = [self.COLOR[class_name] for class_name in self.COLOR]
        legend_labels = list(self.COLOR.keys())

        # Create custom legend entries with colored rectangles
        legend_entries = [mpatches.Rectangle((0, 0), 1, 1, fc=color, edgecolor='none', label=label) 
                          for color, label in zip(legend_colors, legend_labels)]



        # Draw the circles on bev_img
        # Define circle parameters
        num_circles = 8
        radius_increment = 0.1
        center = (0.5, 0.05)  # Adjusted y-coordinate
        for i in range(num_circles):
            radius = (i + 1) * radius_increment
            linewidth = (num_circles - i) * 2  # Varying linewidth

            cv2.circle(bev_img, (int(center[0] * self.shape), int(center[1] * self.shape)), int(radius * self.shape), (150, 10, 0), linewidth)
            circle = plt.Circle(center, radius, fill=False, color='blue')
            self.ax2.add_patch(circle)

            text_content = f'{round((i + 1) * 9.6, 2)} m'
            text_position = ( 30 , center[1] * self.shape + radius * self.shape + 10)  # Adjusted position
            # print(text_position)
            self.ax2.text(text_position[0], text_position[1], text_content, color='white', fontsize=6, va='center', ha='center')



        # ## draw  objects trajectories
        # obj_id = None
        # for obj_id, centroids_deque in tracking_trajectories.items():
        #     centroids_list = list(centroids_deque)
        #     for i in range(1, len(centroids_list)):
        #         start_point = tuple(map(int, centroids_list[i - 1]))
        #         end_point = tuple(map(int, centroids_list[i]))
        #         cv2.line(bev_img, start_point, end_point, (0, 255, 0), 2)


        # Get the last three object IDs
        last_three_obj_ids = list(tracking_trajectories.keys())[-3:]
        # Draw trajectories for the last three objects
        for obj_id in last_three_obj_ids:
            centroids_deque = tracking_trajectories[obj_id]
            centroids_list = list(centroids_deque)
            for i in range(1, len(centroids_list)):
                start_point = tuple(map(int, centroids_list[i - 1]))
                end_point = tuple(map(int, centroids_list[i]))
                cv2.line(bev_img, start_point, end_point, (0, 255, 0), 2)

        # Delete centroids of previous objects
        for obj_id in list(tracking_trajectories.keys()):
            if obj_id not in last_three_obj_ids:
                tracking_trajectories[obj_id].clear()


        # plot camera view range
        x1 = np.linspace(0, self.shape / 2)
        x2 = np.linspace(self.shape / 2, self.shape)
        self.ax2.plot(x1, self.shape / 2 - x1, ls="--", color="grey", linewidth=3, alpha=0.3)
        self.ax2.plot(x2, x2 - self.shape / 2, ls="--", color="grey", linewidth=3, alpha=0.3)
        self.ax2.plot(self.shape / 2,40 , marker="+", markersize=16, markeredgecolor="red")

        # Display the image
        self.ax2.imshow(bev_img, origin="lower")
        self.ax2.set_xticks([])
        self.ax2.set_yticks([])
        self.ax2.set_aspect('equal', adjustable='box')
        self.ax2.legend(handles=legend_entries, loc='lower right', fontsize='x-small', framealpha=0.7)


    # def save_plot(self, path1, path2, name):
    #     self.fig.savefig(
    #         os.path.join(path1, f"{name}.png"),
    #         dpi=self.fig.dpi,
    #         bbox_inches="tight",
    #         pad_inches=0.0,
    #     )
    #         # Close the figure
    #     plt.close(self.fig)

    #     self.fig2.savefig(os.path.join(path2, f"{name}.png"))
    #     # Close the figure
    #     plt.close(self.fig2)

    # def show_result(self, path, name):
    #     # Draw the Matplotlib figure
    #     self.fig.canvas.draw()

    #     # Convert the figure to an image array
    #     image = np.fromstring(self.fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
    #     image = image.reshape(self.fig.canvas.get_width_height()[::-1] + (3,))

    #     # Display the image using OpenCV
    #     cv2.imshow('Image', image)
    #     cv2.waitKey(0)
    #     cv2.destroyAllWindows()
    #     # self.fig.close()



class KITTIObject():
    """
    utils for YOLO3D
    detectionInfo is a class that contains information about the detection
    """
    def __init__(self, line = np.zeros(16)):
        self.name = line[0]

        self.truncation = float(line[1])
        self.occlusion = int(line[2])

        # local orientation = alpha + pi/2
        self.alpha = float(line[3])

        # in pixel coordinate
        self.xmin = float(line[4])
        self.ymin = float(line[5])
        self.xmax = float(line[6])
        self.ymax = float(line[7])

        # height, weigh, length in object coordinate, meter
        self.h = float(line[8])
        self.w = float(line[9])
        self.l = float(line[10])

        # x, y, z in camera coordinate, meter
        self.tx = float(line[11])
        self.ty = float(line[12])
        self.tz = float(line[13])

        # global orientation [-pi, pi]
        self.rot_global = float(line[14])

        # score
        self.score = float(line[15])

    def member_to_list(self):
        output_line = []
        for name, value in vars(self).items():
            output_line.append(value)
        return output_line

    def box3d_candidate(self, rot_local, soft_range):
        x_corners = [self.l, self.l, self.l, self.l, 0, 0, 0, 0]
        y_corners = [self.h, 0, self.h, 0, self.h, 0, self.h, 0]
        z_corners = [0, 0, self.w, self.w, self.w, self.w, 0, 0]

        x_corners = [i - self.l / 2 for i in x_corners]
        y_corners = [i - self.h for i in y_corners]
        z_corners = [i - self.w / 2 for i in z_corners]

        corners_3d = np.transpose(np.array([x_corners, y_corners, z_corners]))
        point1 = corners_3d[0, :]
        point2 = corners_3d[1, :]
        point3 = corners_3d[2, :]
        point4 = corners_3d[3, :]
        point5 = corners_3d[6, :]
        point6 = corners_3d[7, :]
        point7 = corners_3d[4, :]
        point8 = corners_3d[5, :]

        # set up projection relation based on local orientation
        xmin_candi = xmax_candi = ymin_candi = ymax_candi = 0

        if 0 < rot_local < np.pi / 2:
            xmin_candi = point8
            xmax_candi = point2
            ymin_candi = point2
            ymax_candi = point5

        if np.pi / 2 <= rot_local <= np.pi:
            xmin_candi = point6
            xmax_candi = point4
            ymin_candi = point4
            ymax_candi = point1

        if np.pi < rot_local <= 3 / 2 * np.pi:
            xmin_candi = point2
            xmax_candi = point8
            ymin_candi = point8
            ymax_candi = point1

        if 3 * np.pi / 2 <= rot_local <= 2 * np.pi:
            xmin_candi = point4
            xmax_candi = point6
            ymin_candi = point6
            ymax_candi = point5

        # soft constraint
        div = soft_range * np.pi / 180
        if 0 < rot_local < div or 2*np.pi-div < rot_local < 2*np.pi:
            xmin_candi = point8
            xmax_candi = point6
            ymin_candi = point6
            ymax_candi = point5

        if np.pi - div < rot_local < np.pi + div:
            xmin_candi = point2
            xmax_candi = point4
            ymin_candi = point8
            ymax_candi = point1

        return xmin_candi, xmax_candi, ymin_candi, ymax_candi

def get_new_alpha(alpha):
    """
    change the range of orientation from [-pi, pi] to [0, 2pi]
    :param alpha: original orientation in KITTI
    :return: new alpha
    """
    new_alpha = float(alpha) + np.pi / 2.
    if new_alpha < 0:
        new_alpha = new_alpha + 2. * np.pi
        # make sure angle lies in [0, 2pi]
    new_alpha = new_alpha - int(new_alpha / (2. * np.pi)) * (2. * np.pi)

    return new_alpha 

def compute_orientation(P2, obj):
    x = (obj.xmax + obj.xmin) / 2
    # compute camera orientation
    u_distance = x - P2[0, 2]
    focal_length = P2[0, 0]
    rot_ray = np.arctan(u_distance / focal_length)
    # global = alpha + ray
    rot_global = obj.alpha + rot_ray

    # local orientation, [0, 2 * pi]
    # rot_local = obj.alpha + np.pi / 2
    rot_local = get_new_alpha(obj.alpha)

    rot_global = round(rot_global, 2)
    return rot_global, rot_local


def translation_constraints(P2, obj, rot_local):
    bbox = [obj.xmin, obj.ymin, obj.xmax, obj.ymax]
    # rotation matrix
    R = np.array([[ np.cos(obj.rot_global), 0,  np.sin(obj.rot_global)],
                  [          0,             1,             0          ],
                  [-np.sin(obj.rot_global), 0,  np.cos(obj.rot_global)]])
    A = np.zeros((4, 3))
    b = np.zeros((4, 1))
    I = np.identity(3)

    xmin_candi, xmax_candi, ymin_candi, ymax_candi = obj.box3d_candidate(rot_local, soft_range=8)

    X  = np.bmat([xmin_candi, xmax_candi,
                  ymin_candi, ymax_candi])
    # X: [x, y, z] in object coordinate
    X = X.reshape(4,3).T

    # construct equation (4, 3)
    for i in range(4):
        matrice = np.bmat([[I, np.matmul(R, X[:,i])], [np.zeros((1,3)), np.ones((1,1))]])
        M = np.matmul(P2, matrice)

        if i % 2 == 0:
            A[i, :] = M[0, 0:3] - bbox[i] * M[2, 0:3]
            b[i, :] = M[2, 3] * bbox[i] - M[0, 3]

        else:
            A[i, :] = M[1, 0:3] - bbox[i] * M[2, 0:3]
            b[i, :] = M[2, 3] * bbox[i] - M[1, 3]
    # solve x, y, z, using method of least square
    Tran = np.matmul(np.linalg.pinv(A), b)

    tx, ty, tz = [float(np.around(tran, 2)) for tran in Tran]
    return tx, ty, tz


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

def euler_to_quaternion(roll, pitch, yaw):
    import tf.transformations
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    return quaternion

class Visualizer:
    def __init__(self, is_bev_plot, is_img_w_3d, is_marker, shape):
        rospy.init_node('visualizer', anonymous=True)
        self.detection_3d_sub = message_filters.Subscriber("detections_3d",VisBoundingBox3DArray)
        self.image_sub = message_filters.Subscriber("/camera/rgb/image_raw", Image)

        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            (self.detection_3d_sub, self.image_sub), 10, 0.5)

        self._synchronizer.registerCallback(self.callback)

        self.BEV_plot = is_bev_plot
        self.is_3d_img = is_img_w_3d 
        self.is_marker = is_marker 
        self.shape = shape
        self.cv_bridge = CvBridge()

        self.marker_pub = rospy.Publisher('/visualization_marker', MarkerArray, queue_size=10)
        self.image_w_3bb = rospy.Publisher('/visualization_img_3bb', Image, queue_size=10)
        self.BEV_pub = rospy.Publisher('/visualization_BEV', Image, queue_size=10)
        self.rate = rospy.Rate(10) 
        self.count = 0 
        self._class_to_color = {}
        rospy.loginfo("Visualize node has successfully initialized")

    def draw_box(self, cv_image:np.array, detection: List, color: Tuple[int]) -> np.array:
        min_pt = (round(detection[0]), round(detection[1]))
        max_pt = (round(detection[2]), round(detection[3]))

        #draw box 
        cv2.rectangle(cv_image, min_pt, max_pt, color, 2)
        return cv_image
    
    def delete_marker(self, marker_id, ns):
        marker = Marker()
        marker.header.frame_id = "base_frame"
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.id = marker_id
        marker.action = Marker.DELETE
        return marker

    def callback(self, detections_3d_msgs: VisBoundingBox3DArray, img:Image):

        marker_array = MarkerArray()

        # Convert ROS Image message to OpenCV image
        try:
            img1 = self.cv_bridge.imgmsg_to_cv2(img, "bgr8") # for BEV 
            # img2 = self.cv_bridge.imgmsg_to_cv2(img, "bgr8") # for 2D-bb image 
            img3 = self.cv_bridge.imgmsg_to_cv2(img, "bgr8") # for 3D-bb image 
        except CvBridgeError as e:
            print(e)
            return []

        for i, detections_3d_msg in enumerate(detections_3d_msgs.boxes): # every detected-person in one image 
            
            plot3dbev = Plot3DBoxBev(P2) 
            orient_reshaped = np.array(detections_3d_msg.orient).reshape(6, 2)
            # if self.BEV_plot:
            #     plot3d(img1, P2, bbox_, dim, alpha, theta_ray)
            
            # initialize object container
            obj = KITTIObject()
            obj.name = str(yolo_classes[int(detections_3d_msg.classes)]) 
            obj.truncation = float(0.00)
            obj.occlusion = int(-1)
            obj.xmin, obj.ymin, obj.xmax, obj.ymax = int(detections_3d_msg.bbox[0]), int(detections_3d_msg.bbox[1]), int(detections_3d_msg.bbox[2]), int(detections_3d_msg.bbox[3])

            obj.alpha = recover_angle(orient_reshaped, detections_3d_msg.conf, bin_size)
            obj.h, obj.w, obj.l = detections_3d_msg.dim[0], detections_3d_msg.dim[1], detections_3d_msg.dim[2]
            
            # compute orientation 
            obj.rot_global, rot_local = compute_orientation(P2, obj)
            obj.tx, obj.ty, obj.tz = translation_constraints(P2, obj, rot_local)

            
            # plot 3d BEV bbox
            rot_y = detections_3d_msg.alpha + detections_3d_msg.theta_ray
            # rospy.loginfo(f"rot_y: {rot_y}") # 1.62 rad ~ 90 degree 

            if  self.is_3d_img:
                img_with_3d_bb, corners_3D = plot3dbev.draw_3dbox(img=img3, class_object=detections_3d_msg.classes, bbox=[obj.xmin, obj.ymin, obj.xmax, obj.ymax], dim=[obj.h, obj.w, obj.l], loc=[obj.tx, obj.ty, obj.tz], rot_y=rot_y)

            # if detections_3d_msg.objID not in self._class_to_color:
            #     r = random.randint(0, 255)
            #     g = random.randint(0, 255)
            #     b = random.randint(0, 255)
            #     self._class_to_color[detections_3d_msg.objID] = (r, g, b)
            
            # color = self._class_to_color[detections_3d_msg.objID]
            # img2 = self.draw_box(img2, [int(detections_3d_msg.bbox[0]), int(detections_3d_msg.bbox[1]), int(detections_3d_msg.bbox[2]), int(detections_3d_msg.bbox[3])], color)
                
            if self.BEV_plot:
                img1 = plot3dbev.plot(img=img1, class_object=obj.name.lower(), 
                    bbox=[obj.xmin, obj.ymin, obj.xmax, obj.ymax],
                    dim=[obj.l, obj.w, obj.h], loc=[obj.tx, obj.ty, obj.tz], rot_y=rot_y, objId=[detections_3d_msg.objID] if not isinstance(detections_3d_msg.objID, list) else detections_3d_msg.objID)

            if self.is_marker:
                # Create a marker for RViz
                marker = Marker()
                marker.header = img.header 
                # marker.header.frame_id = img.header.frame_id
                marker.header.frame_id = "base_link"
                marker.header.stamp = rospy.Time.now()
                
                marker.id = i

                if self.shape == "CUBE":
                    marker.ns = "cube"
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD
                    
                    # # Marker position and orientation
                    # TODO: 
                    # Issue: when the person is detected closer to the left-bottom of the image, the y-value for the object is supposed to be "negative value" but tends to be highly "positive value" 
                    # marker.pose.position.x = detections_3d_msg.location[1]
                    # marker.pose.position.y = detections_3d_msg.location[2]
                    # marker.pose.position.z = detections_3d_msg.location[0]
                    marker.pose.position.x = detections_3d_msg.location[0]
                    marker.pose.position.y = detections_3d_msg.location[1]
                    marker.pose.position.z = detections_3d_msg.location[2]

                    # Orientation of the bounding box
                    # Assuming we convert the orientation angle `alpha` (plus any necessary adjustments) to a quaternion:
                    # quat = euler_to_quaternion(0, detections_3d_msg.alpha, 0)  # roll, pitch, yaw
                    quat = euler_to_quaternion(0, 0, rot_y)  # roll, pitch, yaw
                    marker.pose.orientation.x = quat[0]
                    marker.pose.orientation.y = quat[1]
                    marker.pose.orientation.z = quat[2]
                    marker.pose.orientation.w = quat[3]

                    # Scale of cube: x, y, z (height, l, w) probably??
                    marker.scale.x = detections_3d_msg.dim[0] # shaft diameter =  height 
                    marker.scale.y = detections_3d_msg.dim[1] # head diameter = length
                    marker.scale.z = detections_3d_msg.dim[2] # height for cube =  width
                    
                    # marker.text = str(yolo_classes[int(detections_3d_msg.classes)]) 
                else: # arrow 
                    marker.ns = "arrow"
                    marker.type = Marker.ARROW
                    marker.action = Marker.ADD

                    # set the start and end points of the arrow 
                    start_point = Point(detections_3d_msg.location[0], detections_3d_msg.location[1], detections_3d_msg.location[2])
                    # end_point = Point(detections_3d_msg.location[0] -2 , detections_3d_msg.location[1]-2, detections_3d_msg.location[2])
                    end_point = Point(0.0, 0.0, 0.0)

                    marker.points.append(start_point)
                    marker.points.append(end_point)

                    # Scale of cube: x, y, z (w, l, h)
                    marker.scale.x = 1.0 # shaft 
                    marker.scale.y = 1.0 # head 
                    marker.scale.z = 0.0 # does not affect 



                marker.color.r = 0.1
                marker.color.g = 1.0
                marker.color.b = 0.1
                marker.color.a = 0.5 # transparency
                marker.lifetime = rospy.Duration()

                # if ( self.count> MARKERS_MAX):
                #     marker_array.markers.pop(0)

                marker_array.markers.append(marker)
                
                # renumber the marker IDs
                # id = 0 
                # for m in marker_array.markers:
                #     m.id = id 
                #     id += 1

        self.marker_pub.publish(marker_array)
        # bb_cv_image = self.cv_bridge.cv2_to_imgmsg(img2, "bgr8")

        # # Set the header frame ID, must be set to the same of visualizer Marker message for rviz 
        # bb_cv_image.header = Header()
        # bb_cv_image.header.frame_id = "camera_link"
        # bb_cv_image.header.stamp = rospy.Time.now()

        # self.image_w_bb.publish(bb_cv_image)
        if self.is_3d_img: 
        
            img_with_3d_bb = self.cv_bridge.cv2_to_imgmsg(img_with_3d_bb, "bgr8")

            # Set the header frame ID, must be set to the same of visualizer Marker message for rviz 
            img_with_3d_bb.header = Header()
            img_with_3d_bb.header.frame_id = "camera_link"
            img_with_3d_bb.header.stamp = rospy.Time.now()

            self.image_w_3bb.publish(img_with_3d_bb)
        # self.count += 1 
        
        # Delete markers 
        # 마커가 계속 rivz에 남아있는 것 방지 
        if self.is_marker:
            rospy.sleep(2) # wait for a moment to visualize the markers 
            
            self.is_marker = is_marker
            # delete_cube_marker = self.delete_marker(0, "cube")
            for marker in marker_array.markers:
                marker.action = Marker.DELETE
            # self.marker_pub.publish(delete_cube_marker)
            self.marker_pub.publish(marker_array)
       
        if self.BEV_plot:
            img1 = plot3dbev.show_result()

            # BEV topic message 
                
            BEV_image_message = self.cv_bridge.cv2_to_imgmsg(img1, "bgr8")

            # Set the header frame ID, must be set to the same of visualizer Marker message for rviz 
            BEV_image_message.header = Header()
            BEV_image_message.header.frame_id = "camera_link"
            BEV_image_message.header.stamp = rospy.Time.now()

            self.BEV_pub.publish(BEV_image_message)

        self.rate.sleep()

def main():
    try:
        is_bev_plot = True
        is_img_w_3d = False 
        is_marker = False
        # shape = "arrow" # "CUBE"
        shape = "CUBE" # "CUBE"
        node = Visualizer(is_bev_plot, is_img_w_3d, is_marker, shape)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()