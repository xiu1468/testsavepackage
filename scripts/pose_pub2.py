#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose

if __name__ == '__main__':
    rospy.init_node('pose_publisher')
    pub = rospy.Publisher('pose_topic', Pose, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        pose_msg = Pose()
        # Set the values of the pose message
        pose_msg.position.x = 1.0
        pose_msg.position.y = 2.0
        pose_msg.position.z = 0.0
        pose_msg.orientation.x = 0.0
        pose_msg.orientation.y = 0.0
        pose_msg.orientation.z = 0.0
        pose_msg.orientation.w = 1.0

        pub.publish(pose_msg)
        rate.sleep()

