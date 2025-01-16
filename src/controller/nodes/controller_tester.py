#!/usr/bin/env python

"""
This script is used to test many of the control options in panda_ros.
To run this, see README.md.

Notes:
* /panda/cart_pose is WAY to jerky for long-distance movements. 
  It's alright for a couple cms, althoug it is still jerky.

"""

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose


def test_cart_pos():
    pub = rospy.Publisher('/panda/cart_pose', Pose, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        pose = Pose()
        pose.position.x = 0.3
        pose.position.y = 0.1
        pose.position.z = 0.2
        
        rospy.loginfo(pose)
        pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('controller_tester', anonymous=True)
    test_cart_pos()

