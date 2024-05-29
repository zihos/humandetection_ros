#!/usr/bin/env python3

import rospy
import tf
import math 
def broadcast_transform():
    rospy.init_node('tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    # In Rviz, red axis= x (x,y,z축으로 1.0씩 이동하면서 어디가 x인지 알 수 있음), Counter-clock wise
    # while not rospy.is_shutdown():
    #     br.sendTransform((5.0, 5.0, 5.0),  # Translation (x, y, z)
    #                      tf.transformations.quaternion_from_euler(0, math.pi, 0),  # Rotation (roll, pitch, yaw) in radian
    #                      rospy.Time.now(),
    #                      "camera_link", # child frame, camera가 coordinates상에서 위로 image가 찍힌 곳처럼 위로 올라가 있어야 함
    #                      "base_link" # parent frame
    #                      )    
    while not rospy.is_shutdown():
        br.sendTransform((0.0, 0.0, 0.0),  # Translation (x, y, z)
                         tf.transformations.quaternion_from_euler(0, 0, 0),  # Rotation (roll, pitch, yaw) in radian
                         rospy.Time.now(),
                         "camera_link", # child frame, camera가 coordinates상에서 위로 image가 찍힌 곳처럼 위로 올라가 있어야 함
                         "base_link" # parent frame
                         )    
        rate.sleep()

if __name__ == '__main__':
    try:
        broadcast_transform()
    except rospy.ROSInterruptException:
        pass
