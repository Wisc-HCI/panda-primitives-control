#!/usr/bin/env python

"""
This script is used to test many of the control options in panda_ros.
To run this, see README.md.

Notes:
* /panda/cart_pose Goes to caresian pose (x, y, z). This is too jerky for long-distance movements. 
* /panda/velocity_bound_path Goes to array of cartesian pose with specified max velocity 
  (unit uknown but 0.1-0.5 is good speed range).

"""

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from panda_ros_msgs.msg import VelocityBoundPath


def test_cart_pos():
    pub = rospy.Publisher('/panda/cart_pose', Pose, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    
    # For some reason, continuosly sending messages makes it jerky while
    # only sending one does not go through. 5 at 10hz seems optimal
    for i in range(5):
        pose = Pose()
        pose.position.x = 0.5
        pose.position.y = 0.1
        pose.position.z = 0.2
        
        rospy.loginfo(pose)
        pub.publish(pose)
        rate.sleep()


def test_vel_bound_path():

    pub = rospy.Publisher('/panda/velocity_bound_path', VelocityBoundPath, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    
    # For some reason, continuosly sending messages makes it jerky while
    # only sending one does not go through. 5 at 10hz seems optimal
    for i in range(5):
        pose1 = Pose()
        pose1.position.x = 0.3
        pose1.position.y = 0.1
        pose1.position.z = 0.2
        
        pose2 = Pose()
        pose2.position.x = 0.4
        pose2.position.y = 0.2
        pose2.position.z = 0.3

        vel_bound_path = VelocityBoundPath()
        vel_bound_path.poses = [pose1, pose2]
        vel_bound_path.maxV = 0.5 # NOT SURE THE UNITS but 0.1 is slow and 0.5 is fast.

        rospy.loginfo(vel_bound_path)
        pub.publish(vel_bound_path)
        rate.sleep()



if __name__ == '__main__':
    rospy.init_node('controller_tester', anonymous=True)
    #test_cart_pos()
    test_vel_bound_path()

