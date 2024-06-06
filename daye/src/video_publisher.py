#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import Header

class VideoPublisher:
    def __init__(self):
        rospy.init_node('video_publisher', anonymous=True)
        self.publisher = rospy.Publisher('/camera/rgb/image_raw', Image, queue_size=10)
        self.cv_bridge = CvBridge()
        self.video = cv2.VideoCapture('/home/ubuntu/catkin_ws/src/my_pkg/data/pedestrian.mov')
        self.timer = rospy.Timer(rospy.Duration(1.0 / 1), self.timer_callback)  # 30 FPS

    def timer_callback(self, event):
        ret, frame = self.video.read()
        if not ret:
            # 비디오 끝에 도달했을 때, 비디오를 다시 시작합니다.
            self.video.set(cv2.CAP_PROP_POS_FRAMES, 0)  # 비디오의 첫 프레임으로 돌아갑니다.
            ret, frame = self.video.read()  # 다시 첫 프레임을 읽습니다.
            if not ret:
                rospy.loginfo("Failed to restart the video")
                return
        
        # 정상적으로 프레임을 읽은 경우 ROS 토픽으로 이미지를 발행합니다.
        image_message = self.cv_bridge.cv2_to_imgmsg(frame, "bgr8")

        # Set the header frame ID, must be set to the same of visualizer Marker message for rviz 
        image_message.header = Header()
        image_message.header.frame_id = "camera_link"
        image_message.header.stamp = rospy.Time.now()

        self.publisher.publish(image_message)
        
def main(args=None):
    rospy.loginfo("Start")
    video_publisher = VideoPublisher()
    rospy.spin()

if __name__ == '__main__':
    main()